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
    Foam::externalForceModels::sixDoFThrustForce

Description
    Body-fixed frame thrust rotating dynamically with body attitude.
    Thrust direction and application point are defined in the body-fixed
    coordinate frame, and transformed dynamically to global coordinates:
        F_global = q * F_body * q^-1
        r_global = q * r_body * q^-1
        M_global = r_global ^ F_global

    Dictionary syntax:
    \verbatim
    rocketThrust
    {
        type              6DoFThrust;
        active            yes;
        direction         (1 0 0);             // Direction in body coordinates
        magnitude         1200;                // Thrust magnitude [N]
        startTime         0.0;                 // Ignition time [s] (default: 0.0)
        endTime           4.0;                 // Shutdown time [s] (default: GREAT)
        applicationPoint  (-0.45 0 0);         // Point in body coordinates [m] (default: zero)
    }
    \endverbatim
\*---------------------------------------------------------------------------*/

#ifndef sixDoFThrustForceModel_H
#define sixDoFThrustForceModel_H

#include "externalForceModel.H"
#include "point.H"

namespace Foam
{
namespace externalForceModels
{

/*---------------------------------------------------------------------------*\
                   Class sixDoFThrustForce Declaration
\*---------------------------------------------------------------------------*/

class sixDoFThrustForce
:
    public externalForceModel
{
private:

    // Private Data

        //- Unit direction vector in body-fixed coordinates
        vector direction_;

        //- Thrust magnitude [N]
        scalar magnitude_;

        //- Ignition start time [s]
        scalar startTime_;

        //- Shutdown end time [s]
        scalar endTime_;

        //- Application point in body-fixed coordinates [m]
        point applicationPoint_;


public:

    //- Runtime type information
    TypeName("6DoFThrust");


    // Constructors

        //- Construct from name and dictionary
        sixDoFThrustForce(const word& name, const dictionary& dict);


    //- Destructor
    virtual ~sixDoFThrustForce() = default;


    // Member Functions

        //- Evaluate thrust force and moment transformed to global coordinates
        virtual bodyLoad evaluate
        (
            const scalar t,
            const quaternion& q = quaternion::I
        ) const override;
};

} // End namespace externalForceModels
} // End namespace Foam

#endif
