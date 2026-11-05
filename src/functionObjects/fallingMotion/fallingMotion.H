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
    Foam::functionObjects::fallingMotion

Description
    Dedicated function object for tracking, displaying, and logging
    dynamic free-fall motion fields (frameVelocity, frameAcceleration, altitude,
    forces, and atmospheric density).

    Allows fine-grained toggling of console stdout output and CSV logging to
    postProcessing/fallingMotion/motion.dat.

\*---------------------------------------------------------------------------*/

#ifndef fallingMotion_H
#define fallingMotion_H

#include "functionObject.H"
#include "IOdictionary.H"
#include "OFstream.H"
#include "fvMesh.H"
#include "uniformDimensionedFields.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

namespace Foam
{
namespace functionObjects
{

/*---------------------------------------------------------------------------*\
                        Class fallingMotion Declaration
\*---------------------------------------------------------------------------*/

class fallingMotion
:
    public functionObject
{
    // Private Data

        //- Time database reference
        const Time& time_;

        //- File output flag
        bool writeCSV_;

        //- Output file stream pointer
        autoPtr<OFstream> motionFilePtr_;

        //- Region name
        word regionName_;

        //- Console print flags
        bool printFrameVel_;
        bool printFrameAcc_;
        bool printAltitude_;
        bool printPressureForce_;
        bool printViscousForce_;
        bool printAeroForce_;
        bool printGravityForce_;
        bool printNetForce_;
        bool printExternalForce_;
        bool printMeanFlow_;
        bool printRelativeVel_;
        bool printSpeedOfSound_;
        bool printMachNumber_;
        bool printMaxLocalMachNumber_;
        bool printDynamicPressure_;


    // Private Member Functions

        //- Open log file and write header
        void openFile();

        //- Output console information
        void printConsole(const fvMesh& mesh);

        //- Write data to motion.dat
        void writeLog(const fvMesh& mesh);

public:

    //- Runtime type information
    TypeName("fallingMotion");

    // Constructors

        //- Construct from time and dictionary
        fallingMotion
        (
            const word& name,
            const Time& runTime,
            const dictionary& dict
        );

    //- Destructor
    virtual ~fallingMotion() = default;

    // Member Functions

        //- Read configuration dictionary
        virtual bool read(const dictionary& dict) override;

        //- Execute at runtime time steps
        virtual bool execute() override;

        //- Write data at write time steps
        virtual bool write() override;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

} // End namespace functionObjects
} // End namespace Foam

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif

// ************************************************************************* //
