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

Description
    Global frame thrust applied along a fixed global direction over a
    specified time window.

    Dictionary syntax:
    \verbatim
    booster
    {
        type              thrust;
        active            yes;
        direction         (0 1 0);             // Direction in global coordinates
        magnitude         1200;                // Thrust magnitude [N]
        startTime         0.0;                 // Ignition time [s] (default: 0.0)
        endTime           4.0;                 // Shutdown time [s] (default: GREAT)
        applicationPoint  (0 0 0);             // Application point [m] (default: zero)
    }
    \endverbatim
\*---------------------------------------------------------------------------*/

#ifndef thrustForceModel_H
#define thrustForceModel_H

#include "externalForceModel.H"
#include "point.H"

namespace Foam
{
namespace externalForceModels
{

/*---------------------------------------------------------------------------*\
                      Class thrustForce Declaration
\*---------------------------------------------------------------------------*/

class thrustForce
:
    public externalForceModel
{
private:

    // Private Data

        //- Unit direction vector in global coordinates
        vector direction_;

        //- Thrust magnitude [N]
        scalar magnitude_;

        //- Ignition start time [s]
        scalar startTime_;

        //- Shutdown end time [s]
        scalar endTime_;

        //- Application point in global coordinates [m]
        point applicationPoint_;


public:

    //- Runtime type information
    TypeName("thrust");


    // Constructors

        //- Construct from name and dictionary
        thrustForce(const word& name, const dictionary& dict);


    //- Destructor
    virtual ~thrustForce() = default;


    // Member Functions

        //- Evaluate thrust force and moment in global coordinates
        virtual bodyLoad evaluate
        (
            const scalar t,
            const quaternion& q = quaternion::I
        ) const override;
};

} // End namespace externalForceModels
} // End namespace Foam

#endif
