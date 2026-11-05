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
    Foam::meanFlowModels::powerLaw

Implementation
    Foam::meanFlowModels::powerLaw

\*---------------------------------------------------------------------------*/

#include "powerLawMeanFlowModel.H"
#include "addToRunTimeSelectionTable.H"

namespace Foam
{
namespace meanFlowModels
{
    defineTypeNameAndDebug(powerLaw, 0);
    addToRunTimeSelectionTable(meanFlowModel, powerLaw, dictionary);
}
}

Foam::meanFlowModels::powerLaw::powerLaw(const dictionary& dict)
:
    meanFlowModel(dict),
    groundHeight_(dict.getOrDefault<scalar>("groundHeight", 0.0)),
    referenceHeight_(dict.getOrDefault<scalar>("referenceHeight", 10.0)),
    referenceVelocity_(dict.getOrDefault<vector>("referenceVelocity", vector::zero)),
    exponent_(dict.getOrDefault<scalar>("exponent", 0.14))
{
    if (referenceHeight_ <= groundHeight_)
    {
        FatalErrorInFunction
            << "referenceHeight (" << referenceHeight_
            << " m) must be strictly greater than groundHeight (" << groundHeight_
            << " m) in powerLaw meanFlow model!" << exit(FatalError);
    }
}

Foam::vector Foam::meanFlowModels::powerLaw::velocity(const scalar altitude) const
{
    if (altitude <= groundHeight_)
    {
        return vector::zero;
    }

    const scalar hNumerator   = altitude - groundHeight_;
    const scalar hDenominator = max(referenceHeight_ - groundHeight_, SMALL);
    const scalar ratio        = max(hNumerator / hDenominator, 0.0);

    return referenceVelocity_ * pow(ratio, exponent_);
}
