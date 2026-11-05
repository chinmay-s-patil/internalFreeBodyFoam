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

Description
    Manager class for configurable collections of external body forces
    specified in constant/fallingFrameDict under the "externalForces" subdictionary.
\*---------------------------------------------------------------------------*/

#ifndef externalForces_H
#define externalForces_H

#include "typeInfo.H"
#include "dictionary.H"
#include "PtrList.H"
#include "fvMesh.H"
#include "externalForceModel.H"
#include "bodyLoad.H"

namespace Foam
{

/*---------------------------------------------------------------------------*\
                       Class externalForces Declaration
\*---------------------------------------------------------------------------*/

class externalForces
{
private:

    // Private Data

        //- Master enable switch
        bool enabled_;

        //- List of active external force models
        PtrList<externalForceModel> forces_;


public:

    //- Runtime type information
    TypeName("externalForces");


    // Constructors

        //- Construct from fallingFrameDict or parent dictionary
        externalForces(const dictionary& dict);


    //- Destructor
    virtual ~externalForces() = default;


    // Member Functions

        //- Check if external forces are enabled
        bool enabled() const
        {
            return enabled_;
        }

        //- Number of configured force models
        label size() const
        {
            return forces_.size();
        }

        //- Access to configured force models
        const PtrList<externalForceModel>& forces() const
        {
            return forces_;
        }

        //- Evaluate total load at time t with explicit orientation quaternion
        bodyLoad evaluate
        (
            const scalar t,
            const quaternion& q = quaternion::I
        ) const;

        //- Evaluate total load at time t, querying attitude from mesh if 6DoF
        bodyLoad evaluate
        (
            const scalar t,
            const fvMesh& mesh
        ) const;

        //- Print detailed load summary to Info
        void printLoads
        (
            const scalar t,
            const fvMesh& mesh
        ) const;
};

} // End namespace Foam

#endif
