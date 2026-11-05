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

Class
    Foam::solidBodyMotionFunctions::freeBodyRotationalMotion

Description
    Solid-body rotational motion driven by 3D attitude quaternions registered
    in the mesh registry by freeBody6DoFPimpleFoam (attitudeW, attitudeVec).

    Accommodates arbitrary 3D multi-axis tumbling across a spherical AMI
    interface or overset grid without mesh distortion.

SourceFiles
    freeBodyRotationalMotion.C
\*---------------------------------------------------------------------------*/

#ifndef Foam_freeBodyRotationalMotion_H
#define Foam_freeBodyRotationalMotion_H

#include "solidBodyMotionFunction.H"
#include "primitiveFields.H"
#include "point.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{
namespace solidBodyMotionFunctions
{

/*---------------------------------------------------------------------------*\
                 Class freeBodyRotationalMotion Declaration
\*---------------------------------------------------------------------------*/

class freeBodyRotationalMotion
:
    public solidBodyMotionFunction
{
    // Private Data

        //- Center of rotation / hub position
        point origin_;


public:

    //- Runtime type information
    TypeName("freeBodyRotationalMotion");


    // Constructors

        //- Construct from components
        freeBodyRotationalMotion
        (
            const dictionary& SBMFCoeffs,
            const Time& runTime
        );

        //- Construct and return a clone
        virtual autoPtr<solidBodyMotionFunction> clone() const override
        {
            return autoPtr<solidBodyMotionFunction>
            (
                new freeBodyRotationalMotion
                (
                    SBMFCoeffs_,
                    time_
                )
            );
        }

        //- Disallow default bitwise copy construct
        freeBodyRotationalMotion(const freeBodyRotationalMotion&) = delete;

        //- Disallow default bitwise assignment
        void operator=(const freeBodyRotationalMotion&) = delete;


    //- Destructor
    virtual ~freeBodyRotationalMotion() = default;


    // Member Functions

        //- Return the solid-body motion transformation septernion
        virtual septernion transformation() const override;

        //- Update properties from a dictionary
        virtual bool read(const dictionary& SBMFCoeffs) override;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace solidBodyMotionFunctions
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
