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
    Foam::externalForceModels::thrustForce

Implementation
    Foam::externalForceModels::thrustForce
\*---------------------------------------------------------------------------*/

#include "thrustForceModel.H"
#include "addToRunTimeSelectionTable.H"

namespace Foam
{
namespace externalForceModels
{
    defineTypeNameAndDebug(thrustForce, 0);
    addToRunTimeSelectionTable(externalForceModel, thrustForce, dictionary);
}
}

// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::externalForceModels::thrustForce::thrustForce
(
    const word& name,
    const dictionary& dict
)
:
    externalForceModel(name, dict),
    direction_(dict.get<vector>("direction")),
    magnitude_(dict.get<scalar>("magnitude")),
    startTime_(dict.getOrDefault<scalar>("startTime", 0.0)),
    endTime_(dict.getOrDefault<scalar>("endTime", GREAT)),
    applicationPoint_(dict.getOrDefault<point>("applicationPoint", point::zero))
{
    const scalar magD = mag(direction_);
    if (magD < SMALL)
    {
        FatalErrorInFunction
            << "Zero length direction vector provided for thrust '" << name << "'."
            << exit(FatalError);
    }
    direction_ /= magD;

    if (endTime_ < startTime_)
    {
        FatalErrorInFunction
            << "endTime (" << endTime_ << " s) is less than startTime ("
            << startTime_ << " s) for thrust '" << name << "'."
            << exit(FatalError);
    }
}


// * * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * //

Foam::bodyLoad Foam::externalForceModels::thrustForce::evaluate
(
    const scalar t,
    const quaternion& /* q */
) const
{
    if (!active_ || t < startTime_ || t > endTime_)
    {
        return bodyLoad();
    }

    const vector F = magnitude_ * direction_;
    const vector moment = applicationPoint_ ^ F;
    return bodyLoad(F, moment);
}

// ************************************************************************* //
