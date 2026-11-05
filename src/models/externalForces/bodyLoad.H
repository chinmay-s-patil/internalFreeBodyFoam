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
    Foam::bodyLoad

Description
    Container holding force and moment vectors in the global reference frame.
\*---------------------------------------------------------------------------*/

#ifndef bodyLoad_H
#define bodyLoad_H

#include "vector.H"

namespace Foam
{

/*---------------------------------------------------------------------------*\
                          Class bodyLoad Declaration
\*---------------------------------------------------------------------------*/

class bodyLoad
{
public:

    // Public Data

        //- Force in global reference frame [N]
        vector force;

        //- Moment about centre of mass in global reference frame [N.m]
        vector moment;


    // Constructors

        //- Construct zero load
        inline bodyLoad()
        :
            force(vector::zero),
            moment(vector::zero)
        {}

        //- Construct from force and moment
        inline bodyLoad(const vector& f, const vector& m)
        :
            force(f),
            moment(m)
        {}


    // Member Operators

        inline bodyLoad& operator+=(const bodyLoad& bl)
        {
            force += bl.force;
            moment += bl.moment;
            return *this;
        }

        inline friend bodyLoad operator+(const bodyLoad& a, const bodyLoad& b)
        {
            return bodyLoad(a.force + b.force, a.moment + b.moment);
        }
};

} // End namespace Foam

#endif
