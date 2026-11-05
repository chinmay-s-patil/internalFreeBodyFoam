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
//  predictRotorDynamics.H — 6DoF Rotational Predictor Step
// ============================================================

qBody_old = qBody;
omegaB_old = omegaB;

// Advance orientation by dt using omegaB from previous time step
// so that mesh.controlledUpdate() moves the inner mesh for PIMPLE iterations
const scalar dtPredict = runTime.deltaTValue();
const scalar wMagPredict = mag(omegaB);

if (wMagPredict > 1e-12 && dtPredict > 1e-15)
{
    const quaternion dq(omegaB/wMagPredict, wMagPredict*dtPredict);
    qBody = qBody_old * dq;
    qBody.normalise();
}

// Update uniform registered fields so freeBodyRotationalMotion reads the predicted orientation
attitudeW.value() = qBody.w();
attitudeVec.value() = qBody.v();
