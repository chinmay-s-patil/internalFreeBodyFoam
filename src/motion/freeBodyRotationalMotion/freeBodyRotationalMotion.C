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

Class
    Foam::solidBodyMotionFunctions::freeBodyRotationalMotion

Description
    Implementation of the registry-driven 3D attitude motion function.
\*---------------------------------------------------------------------------*/

#include "freeBodyRotationalMotion.H"
#include "addToRunTimeSelectionTable.H"
#include "polyMesh.H"
#include "uniformDimensionedFields.H"
#include "quaternion.H"
#include "septernion.H"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace solidBodyMotionFunctions
{
    defineTypeNameAndDebug(freeBodyRotationalMotion, 0);
    addToRunTimeSelectionTable
    (
        solidBodyMotionFunction,
        freeBodyRotationalMotion,
        dictionary
    );
    // Backward-compatibility alias for legacy cases specifying tFFRotationalMotion
    addNamedToRunTimeSelectionTable
    (
        solidBodyMotionFunction,
        freeBodyRotationalMotion,
        dictionary,
        tFFRotationalMotion
    );
}
}

// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::solidBodyMotionFunctions::freeBodyRotationalMotion::freeBodyRotationalMotion
(
    const dictionary& SBMFCoeffs,
    const Time& runTime
)
:
    solidBodyMotionFunction(SBMFCoeffs, runTime),
    origin_(SBMFCoeffs.getOrDefault<point>("origin", point::zero))
{}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

Foam::septernion
Foam::solidBodyMotionFunctions::freeBodyRotationalMotion::transformation() const
{
    if (!time_.foundObject<polyMesh>(polyMesh::defaultRegion))
    {
        return septernion::I;
    }

    const polyMesh& mesh = time_.lookupObject<polyMesh>(polyMesh::defaultRegion);

    if
    (
        !mesh.foundObject<uniformDimensionedScalarField>("attitudeW")
     || !mesh.foundObject<uniformDimensionedVectorField>("attitudeVec")
    )
    {
        return septernion::I;
    }

    const scalar w =
        mesh.lookupObject<uniformDimensionedScalarField>("attitudeW").value();
    const vector v =
        mesh.lookupObject<uniformDimensionedVectorField>("attitudeVec").value();

    quaternion q(w, v);
    const scalar magQ = mag(q);
    if (magQ > 1e-12)
    {
        q.normalise();
    }
    else
    {
        q = quaternion::I;
    }

    return septernion(-origin_) * q * septernion(origin_);
}


bool Foam::solidBodyMotionFunctions::freeBodyRotationalMotion::read
(
    const dictionary& SBMFCoeffs
)
{
    if (solidBodyMotionFunction::read(SBMFCoeffs))
    {
        SBMFCoeffs.readIfPresent("origin", origin_);
        return true;
    }

    return false;
}

// ************************************************************************* //
