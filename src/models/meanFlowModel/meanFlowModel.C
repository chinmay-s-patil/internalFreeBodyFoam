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

Implementation
    Foam::meanFlowModel

\*---------------------------------------------------------------------------*/

#include "meanFlowModel.H"

namespace Foam
{
    defineTypeNameAndDebug(meanFlowModel, 0);
    defineRunTimeSelectionTable(meanFlowModel, dictionary);
}

Foam::meanFlowModel::meanFlowModel(const dictionary& dict)
:
    dict_(dict)
{}

Foam::autoPtr<Foam::meanFlowModel> Foam::meanFlowModel::New(const dictionary& dict)
{
    const dictionary& meanFlowDict = dict.found("meanFlow") ? dict.subDict("meanFlow") : dict;
    const word modelType = meanFlowDict.getOrDefault<word>("model", "constant");

    Info<< "Selecting meanFlowModel " << modelType << endl;

    auto* ctorPtr = dictionaryConstructorTable(modelType);

    if (!ctorPtr)
    {
        FatalErrorInFunction
            << "Unknown meanFlowModel type " << modelType << nl << nl
            << "Valid meanFlowModel types are:" << nl
            << dictionaryConstructorTablePtr_->toc()
            << exit(FatalError);
    }

    return autoPtr<meanFlowModel>(ctorPtr(meanFlowDict));
}
