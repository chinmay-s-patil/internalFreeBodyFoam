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

Description
    Power-law atmospheric mean flow model:
    U(z) = U_ref * ((z - z0) / (z_ref - z0))^alpha

\*---------------------------------------------------------------------------*/

#ifndef powerLawMeanFlowModel_H
#define powerLawMeanFlowModel_H

#include "meanFlowModel.H"

namespace Foam
{
namespace meanFlowModels
{

class powerLaw
:
    public meanFlowModel
{
    scalar groundHeight_;
    scalar referenceHeight_;
    vector referenceVelocity_;
    scalar exponent_;

public:

    TypeName("powerLaw");

    powerLaw(const dictionary& dict);
    virtual ~powerLaw() = default;

    virtual vector velocity(const scalar altitude) const override;
};

} // End namespace meanFlowModels
} // End namespace Foam

#endif
