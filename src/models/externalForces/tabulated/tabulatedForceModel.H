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
    Foam::externalForceModels::tabulatedForce

Description
    Time-dependent force vector linearly interpolated from a user-prescribed
    time-force table.

    Dictionary syntax:
    \verbatim
    prescribedLoad
    {
        type              tabulatedForce;
        active            yes;
        interpolation     linear;              // Interpolation scheme (default: linear)
        outOfRange        zero;                // Behaviour outside range: zero, hold (default: zero)
        applicationPoint  (0 0 0);             // Optional application point [m]
        table
        (
            (0.0  (0 0 0))
            (0.5  (0 200 0))
            (1.0  (0 0 0))
            (2.0  (0 -50 0))
        );
    }
    \endverbatim
\*---------------------------------------------------------------------------*/

#ifndef tabulatedForceModel_H
#define tabulatedForceModel_H

#include "externalForceModel.H"
#include "List.H"
#include "Tuple2.H"
#include "point.H"

namespace Foam
{
namespace externalForceModels
{

/*---------------------------------------------------------------------------*\
                     Class tabulatedForce Declaration
\*---------------------------------------------------------------------------*/

class tabulatedForce
:
    public externalForceModel
{
private:

    // Private Data

        //- Table of (time, forceVector) pairs
        List<Tuple2<scalar, vector>> table_;

        //- Out of range handling ("zero" or "hold")
        word outOfRange_;

        //- Interpolation scheme ("linear")
        word interpolation_;

        //- Application point in global coordinates [m]
        point applicationPoint_;


public:

    //- Runtime type information
    TypeName("tabulatedForce");


    // Constructors

        //- Construct from name and dictionary
        tabulatedForce(const word& name, const dictionary& dict);


    //- Destructor
    virtual ~tabulatedForce() = default;


    // Member Functions

        //- Evaluate force and moment in global coordinates at time t
        virtual bodyLoad evaluate
        (
            const scalar t,
            const quaternion& q = quaternion::I
        ) const override;
};

} // End namespace externalForceModels
} // End namespace Foam

#endif
