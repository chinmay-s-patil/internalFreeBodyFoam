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
    Foam::externalForceModel

Description
    Abstract base class for external forces acting on the falling body.
\*---------------------------------------------------------------------------*/

#ifndef externalForceModel_H
#define externalForceModel_H

#include "typeInfo.H"
#include "runTimeSelectionTables.H"
#include "dictionary.H"
#include "vector.H"
#include "quaternion.H"
#include "autoPtr.H"
#include "bodyLoad.H"

namespace Foam
{

/*---------------------------------------------------------------------------*\
                    Class externalForceModel Declaration
\*---------------------------------------------------------------------------*/

class externalForceModel
{
protected:

    // Protected Data

        //- Name of the force model instance
        word name_;

        //- Reference to the model subdictionary
        const dictionary& dict_;

        //- Whether this force is active
        bool active_;


public:

    //- Runtime type information
    TypeName("externalForceModel");

    // Declare run-time constructor selection table
    declareRunTimeSelectionTable
    (
        autoPtr,
        externalForceModel,
        dictionary,
        (const word& name, const dictionary& dict),
        (name, dict)
    );


    // Constructors

        //- Construct from name and dictionary
        externalForceModel(const word& name, const dictionary& dict);


    // Selector

        //- Select model based on "type" keyword in dict
        static autoPtr<externalForceModel> New
        (
            const word& name,
            const dictionary& dict
        );


    //- Destructor
    virtual ~externalForceModel() = default;


    // Member Functions

        //- Name of this force instance
        const word& name() const
        {
            return name_;
        }

        //- Check if active
        bool active() const
        {
            return active_;
        }

        //- Evaluate force and moment in global reference frame at time t
        //  q is body orientation quaternion (body -> world)
        virtual bodyLoad evaluate
        (
            const scalar t,
            const quaternion& q = quaternion::I
        ) const = 0;
};

} // End namespace Foam

#endif
