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
    Foam::meanFlowModel

Description
    Abstract base class for mean atmospheric flow velocity models U_mean(z).

\*---------------------------------------------------------------------------*/

#ifndef meanFlowModel_H
#define meanFlowModel_H

#include "typeInfo.H"
#include "runTimeSelectionTables.H"
#include "dictionary.H"
#include "vector.H"
#include "autoPtr.H"

namespace Foam
{

/*---------------------------------------------------------------------------*\
                       Class meanFlowModel Declaration
\*---------------------------------------------------------------------------*/

class meanFlowModel
{
protected:

    // Private Data
        const dictionary& dict_;

public:

    //- Runtime type information
    TypeName("meanFlowModel");

    // Declare run-time constructor selection table
    declareRunTimeSelectionTable
    (
        autoPtr,
        meanFlowModel,
        dictionary,
        (const dictionary& dict),
        (dict)
    );

    // Constructors
        meanFlowModel(const dictionary& dict);

    // Selector
        static autoPtr<meanFlowModel> New(const dictionary& dict);

    //- Destructor
    virtual ~meanFlowModel() = default;

    // Member Functions
        virtual vector velocity(const scalar altitude) const = 0;
};

} // End namespace Foam

#endif
