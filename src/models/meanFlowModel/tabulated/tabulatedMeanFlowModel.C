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
    Foam::meanFlowModels::tabulated

Implementation
    Foam::meanFlowModels::tabulated

\*---------------------------------------------------------------------------*/

#include "tabulatedMeanFlowModel.H"
#include "addToRunTimeSelectionTable.H"
#include <algorithm>

namespace Foam
{
namespace meanFlowModels
{
    defineTypeNameAndDebug(tabulated, 0);
    addToRunTimeSelectionTable(meanFlowModel, tabulated, dictionary);
}
}

Foam::meanFlowModels::tabulated::tabulated(const dictionary& dict)
:
    meanFlowModel(dict),
    table_()
{
    dict.readIfPresent("table", table_);

    if (table_.empty())
    {
        IOWarningInFunction(dict)
            << "No 'table' entries found in tabulated meanFlow model dictionary. "
            << "Model will return vector::zero." << nl;
    }
    else if (table_.size() > 1)
    {
        std::sort
        (
            table_.begin(),
            table_.end(),
            [](const Tuple2<scalar, vector>& a, const Tuple2<scalar, vector>& b)
            {
                return a.first() < b.first();
            }
        );
    }
}

Foam::vector Foam::meanFlowModels::tabulated::velocity(const scalar altitude) const
{
    if (table_.empty())
    {
        return vector::zero;
    }

    if (altitude <= table_.first().first())
    {
        return table_.first().second();
    }
    if (altitude >= table_.last().first())
    {
        return table_.last().second();
    }

    for (label i = 0; i < table_.size() - 1; ++i)
    {
        if (altitude >= table_[i].first() && altitude <= table_[i+1].first())
        {
            const scalar z0 = table_[i].first();
            const scalar z1 = table_[i+1].first();
            const vector& u0 = table_[i].second();
            const vector& u1 = table_[i+1].second();

            const scalar frac = (altitude - z0) / max(z1 - z0, SMALL);
            return u0 + frac * (u1 - u0);
        }
    }

    return table_.last().second();
}
