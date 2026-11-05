/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2026 Chinmay S. Patil
-------------------------------------------------------------------------------
License
    This file is part of OpenFOAM.

    OpenFOAM is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenFOAM is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License
    along with OpenFOAM.  If not, see <http://www.gnu.org/licenses/>.
\*---------------------------------------------------------------------------*/

// ============================================================
//  correctRotorDynamics.H — 6DoF Rotational Corrector Step
// ============================================================

vector mAero = vector::zero;

#if defined(FF_INCOMPRESSIBLE)
tmp<volSymmTensorField> tdevTauRotor = turbulence->devReff();
const volSymmTensorField& devTauRotor = tdevTauRotor();

forAll(patchNames, i)
{
    const label patchi = mesh.boundaryMesh().findPatchID(patchNames[i]);
    if (patchi != -1)
    {
        const fvPatchScalarField& p_rgh_p = p_rgh.boundaryField()[patchi];
        const fvPatchScalarField& gh_p = gh.boundaryField()[patchi];
        const fvPatchSymmTensorField& devTau_p = devTauRotor.boundaryField()[patchi];
        const vectorField& Sf = mesh.boundary()[patchi].Sf();
        const vectorField& Cf = mesh.boundary()[patchi].Cf();

        forAll(p_rgh_p, facei)
        {
            const scalar pFace = includeBuoyancy ? (p_rgh_p[facei] + gh_p[facei]) : p_rgh_p[facei];
            const vector dFp = pFace * Sf[facei] * rhoAtmosphere.value();
            const vector dFv = (devTau_p[facei] & Sf[facei]) * rhoAtmosphere.value();
            mAero += (Cf[facei] - hubOrigin) ^ (dFp + dFv);
        }
    }
}
#endif

reduce(mAero, sumOp<vector>());

// Moment in world coordinates
aeroMoment.value() = mAero;

// Transform total moment (aerodynamic + external) from world to body principal axes: Mb = R^T & (mAero + mExt)
const vector mTotal = mAero + externalMoment.value();
const tensor R = qBody_old.R();
vector Mb = R.T() & mTotal;

// Apply rotation freedom mask
Mb = cmptMultiply(Mb, rotFreedom);

// Apply damping if specified
if (mag(rotorDamping) > 0.0)
{
    Mb -= cmptMultiply(rotorDamping, omegaB_old);
}

const scalar dtCorrect = runTime.deltaTValue();

// Stabilize rotational coupling against explicit added-inertia feedback in dense fluids
if (dtCorrect > 1e-15 && mag(addedInertiaPrincipal) > 0.0)
{
    const vector alpha_old = (omegaB - omegaB_old) / dtCorrect;
    Mb += cmptMultiply(addedInertiaPrincipal, alpha_old);
}

// Solve Euler's rotational equations via RK4 using effective inertia
auto domega = [&](const vector& w) -> vector
{
    return JbEffInv & (Mb - (w ^ (JbEff & w)));
};
const vector k1 = domega(omegaB_old);
const vector k2 = domega(omegaB_old + 0.5*dtCorrect*k1);
const vector k3 = domega(omegaB_old + 0.5*dtCorrect*k2);
const vector k4 = domega(omegaB_old + dtCorrect*k3);

omegaB = omegaB_old + (dtCorrect/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
omegaB = cmptMultiply(omegaB, rotFreedom);

// Correct orientation quaternion from old state
const scalar wMagCorrect = mag(omegaB);
if (wMagCorrect > 1e-12 && dtCorrect > 1e-15)
{
    const quaternion dq(omegaB/wMagCorrect, wMagCorrect*dtCorrect);
    qBody = qBody_old * dq;
    qBody.normalise();
}
else
{
    qBody = qBody_old;
}

// Update registered state fields
attitudeW.value() = qBody.w();
attitudeVec.value() = qBody.v();
angularVelocity.value() = omegaB;

const vector eulerDeg = qBody.eulerAngles(quaternion::XYZ) * (180.0 / constant::mathematical::pi);
eulerAngleDeg.value() = eulerDeg;

const vector rpm = omegaB * (60.0 / (2.0 * constant::mathematical::pi));

if (reportRotor && Pstream::master())
{
    Info<< "freeBody6DoF Motion:"
        << "  Omega = (" << omegaB.x() << " " << omegaB.y() << " " << omegaB.z() << ") rad/s"
        << "  [RPM: (" << rpm.x() << " " << rpm.y() << " " << rpm.z() << ")]"
        << "  Angles = (" << eulerDeg.x() << " " << eulerDeg.y() << " " << eulerDeg.z() << ") deg"
        << nl
        << "  Aero Moment = (" << mAero.x() << " " << mAero.y() << " " << mAero.z() << ") N.m";
    if (mag(externalMoment.value()) > SMALL)
    {
        Info<< nl
            << "  External Moment = (" << externalMoment.value().x() << " "
            << externalMoment.value().y() << " " << externalMoment.value().z() << ") N.m";
    }
    Info<< endl;
}

if (Pstream::master() && tumbleLogPtr)
{
    tumbleLogPtr()
        << runTime.value() << tab
        << omegaB.x() << tab << omegaB.y() << tab << omegaB.z() << tab
        << rpm.x() << tab << rpm.y() << tab << rpm.z() << tab
        << mAero.x() << tab << mAero.y() << tab << mAero.z() << tab
        << eulerDeg.x() << tab << eulerDeg.y() << tab << eulerDeg.z() << tab
        << qBody.w() << tab << qBody.v().x() << tab << qBody.v().y() << tab << qBody.v().z() << nl;
    tumbleLogPtr().flush();
}
