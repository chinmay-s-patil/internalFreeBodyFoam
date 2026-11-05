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

/*---------------------------------*- C++ -*----------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
Class
    Foam::externalForces

Implementation
    Foam::externalForces
\*---------------------------------------------------------------------------*/

#include "externalForces.H"
#include "uniformDimensionedFields.H"

namespace Foam
{
    defineTypeNameAndDebug(externalForces, 0);
}

// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::externalForces::externalForces(const dictionary& dict)
:
    enabled_(false),
    forces_()
{
    if (!dict.found("externalForces"))
    {
        return;
    }

    const dictionary& extDict = dict.subDict("externalForces");
    enabled_ = extDict.getOrDefault<bool>("enabled", true);

    if (!enabled_)
    {
        Info<< "  External forces: disabled" << endl;
        return;
    }

    if (!extDict.found("forces"))
    {
        Info<< "  External forces: enabled, but no 'forces' subdictionary found." << endl;
        return;
    }

    const dictionary& forcesDict = extDict.subDict("forces");
    const wordList forceNames = forcesDict.toc();

    label nDicts = 0;
    forAll(forceNames, i)
    {
        if (forcesDict.isDict(forceNames[i]))
        {
            nDicts++;
        }
    }

    forces_.setSize(nDicts);
    label idx = 0;
    label nActive = 0;

    forAll(forceNames, i)
    {
        const word& name = forceNames[i];
        if (forcesDict.isDict(name))
        {
            const dictionary& subDict = forcesDict.subDict(name);
            forces_.set(idx, externalForceModel::New(name, subDict));

            if (forces_[idx].active())
            {
                nActive++;
            }
            idx++;
        }
    }

    Info<< "  External forces: " << forces_.size() << " configured, "
        << nActive << " active" << endl;
}


// * * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * //

Foam::bodyLoad Foam::externalForces::evaluate
(
    const scalar t,
    const quaternion& q
) const
{
    bodyLoad total;

    if (!enabled_)
    {
        return total;
    }

    forAll(forces_, i)
    {
        if (forces_[i].active())
        {
            total += forces_[i].evaluate(t, q);
        }
    }

    return total;
}

Foam::bodyLoad Foam::externalForces::evaluate
(
    const scalar t,
    const fvMesh& mesh
) const
{
    if (!enabled_)
    {
        return bodyLoad();
    }

    quaternion q = quaternion::I;

    if
    (
        mesh.foundObject<uniformDimensionedScalarField>("attitudeW")
     && mesh.foundObject<uniformDimensionedVectorField>("attitudeVec")
    )
    {
        const scalar w = mesh.lookupObject<uniformDimensionedScalarField>("attitudeW").value();
        const vector v = mesh.lookupObject<uniformDimensionedVectorField>("attitudeVec").value();
        q = quaternion(w, v);
        q.normalise();
    }

    return evaluate(t, q);
}

void Foam::externalForces::printLoads
(
    const scalar t,
    const fvMesh& mesh
) const
{
    if (!enabled_ || forces_.empty())
    {
        return;
    }

    quaternion q = quaternion::I;
    if
    (
        mesh.foundObject<uniformDimensionedScalarField>("attitudeW")
     && mesh.foundObject<uniformDimensionedVectorField>("attitudeVec")
    )
    {
        const scalar w = mesh.lookupObject<uniformDimensionedScalarField>("attitudeW").value();
        const vector v = mesh.lookupObject<uniformDimensionedVectorField>("attitudeVec").value();
        q = quaternion(w, v);
        q.normalise();
    }

    const bodyLoad total = evaluate(t, q);
    Info<< "  Total External Force  = " << total.force << " N" << endl;
    if (mag(total.moment) > SMALL)
    {
        Info<< "  Total External Moment = " << total.moment << " N.m" << endl;
    }

    forAll(forces_, i)
    {
        if (forces_[i].active())
        {
            const bodyLoad load_i = forces_[i].evaluate(t, q);
            Info<< "    [" << forces_[i].name() << "] F = " << load_i.force << " N";
            if (mag(load_i.moment) > SMALL)
            {
                Info<< ", M = " << load_i.moment << " N.m";
            }
            Info << endl;
        }
    }
}

// ************************************************************************* //
