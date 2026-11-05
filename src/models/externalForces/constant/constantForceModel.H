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
    Foam::externalForceModels::constantForce

Description
    Constant external force vector in global coordinates.

    Dictionary syntax:
    \verbatim
    constantLoad
    {
        type              constantForce;
        active            yes;
        force             (0 0 -50);           // Force vector [N]
        applicationPoint  (0 0 0);             // Optional point [m]
    }
    \endverbatim
\*---------------------------------------------------------------------------*/

#ifndef constantForceModel_H
#define constantForceModel_H

#include "externalForceModel.H"
#include "point.H"

namespace Foam
{
namespace externalForceModels
{

/*---------------------------------------------------------------------------*\
                     Class constantForce Declaration
\*---------------------------------------------------------------------------*/

class constantForce
:
    public externalForceModel
{
private:

    // Private Data

        //- Constant force vector in global coordinates [N]
        vector force_;

        //- Application point relative to centre of mass in global coordinates [m]
        point applicationPoint_;


public:

    //- Runtime type information
    TypeName("constantForce");


    // Constructors

        //- Construct from name and dictionary
        constantForce(const word& name, const dictionary& dict);


    //- Destructor
    virtual ~constantForce() = default;


    // Member Functions

        //- Evaluate force and moment in global coordinates
        virtual bodyLoad evaluate
        (
            const scalar t,
            const quaternion& q = quaternion::I
        ) const override;
};

} // End namespace externalForceModels
} // End namespace Foam

#endif
