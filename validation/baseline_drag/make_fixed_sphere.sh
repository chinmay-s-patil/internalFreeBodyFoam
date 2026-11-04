#!/bin/bash
# make_fixed_sphere.sh <Re> [END_TIME]
# Sets up a fixed sphere case from tutorials/steelBallRun at the specified Reynolds number.
# D = 0.05 m, nu = 1.5e-5 m^2/s, rho = 1.204 kg/m^3.
# V = Re * nu / D
set -euo pipefail
cd "${0%/*}"

RE=${1:?usage: $0 <Re> [END_TIME]}
END=${2:-10.0}
SRC=$(realpath "../../tutorials/steelBallRun")
D=0.05
NU=1.5e-5
RHO=1.204
V=$(python3 -c "print(f'{$RE*$NU/$D:.6f}')")
CASE="fixedSphere_Re${RE}"

echo "=== Building $CASE (Re = $RE, V = $V m/s, D = $D m, endTime = $END s) ==="
rm -rf "$CASE"
mkdir -p "$CASE/constant" "$CASE/0" "$CASE/system"

# 1. Copy mesh and constant files
cp -r "$SRC/constant/polyMesh" "$CASE/constant/"
cp "$SRC/constant/transportProperties" "$CASE/constant/"
cp "$SRC/constant/turbulenceProperties" "$CASE/constant/"

# 2. Configure fallingFrameDict
cat > "$CASE/constant/fallingFrameDict" <<EOF
/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\\\    /   O peration     | Version:  2412
    \\\\  /    A nd           | Website:  www.openfoam.com
\\*---------------------------------------------------------------------------*/
FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    "constant";
    object      fallingFrameDict;
}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

u0                  ( 0 -$V 0 );
acceleration        ( 0 0 0 );
initialAcceleration ( 0 0 0 );

patches             ( sphere );

mass                0.510509;
freedom             ( 0 0 0 );

rhoInf              $RHO;
densityModel        rhoInf;
gravityModel        constant;
groundInterrupt     off;

rotor
{
    hubOrigin         (0 0 0);
    inertiaPrincipal  (1.27627e-04 1.27627e-04 1.27627e-04);
    rotationFreedom   (0 0 0);
    initialOmega      (0 0 0);
    initialAxis       (0 1 0);
    initialAngleDeg   0.0;
    damping           (0 0 0);
    report            false;
}

print
{
    frameVel        on;
    frameAcc        on;
    altitude        off;
    pressureForce   off;
    viscousForce    off;
    aeroForce       on;
    gravityForce    on;
    netForce        on;
    meanFlow        off;
    relativeVel     off;
    dynamicPressure off;
}
EOF

# 3. Configure system files
cp "$SRC/system/fvSchemes" "$CASE/system/"
cp "$SRC/system/fvSolution" "$CASE/system/"
cp "$SRC/system/decomposeParDict" "$CASE/system/"

cat > "$CASE/system/controlDict" <<EOF
/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\\\    /   O peration     | Version:  2412
    \\\\  /    A nd           | Website:  www.openfoam.com
\\*---------------------------------------------------------------------------*/
FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    "system";
    object      controlDict;
}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

application     freeBody6DoFPimpleFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         $END;

deltaT          0.0005;

writeControl    timeStep;
writeInterval   2000;
purgeWrite      2;

writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;

adjustTimeStep  yes;
maxCo           0.8;
maxDeltaT       0.002;

libs
(
    "libfreeBodyRotationalMotion.so"
    "libfallingInletVelocity.so"
);

functions
{
    sphereForces
    {
        type            forces;
        libs            (forces);
        patches         (sphere);
        rho             rhoInf;
        rhoInf          $RHO;
        CofR            (0 0 0);
        pName           p;
        UName           U;
        executeControl  timeStep;
        executeInterval 1;
        writeControl    timeStep;
        writeInterval   1;
    }
}
EOF

# 4. Configure 0 directory
cp "$SRC/0.orig/p_rgh" "$CASE/0/" 2>/dev/null || cp "$SRC/0/p_rgh" "$CASE/0/"
cp "$SRC/0.orig/U" "$CASE/0/" 2>/dev/null || cp "$SRC/0/U" "$CASE/0/"

# Warm start internalField U = (0 V 0), leaving sphere boundary at (0 0 0)
sed -i "s/internalField   uniform (0 0 0);/internalField   uniform (0 $V 0);/g" "$CASE/0/U"

# 5. Create Allrun script
cat > "$CASE/Allrun" <<'EOF'
#!/bin/bash
set -euo pipefail
[ -n "${WM_PROJECT_DIR:-}" ] || . /usr/lib/openfoam/openfoam2412/etc/bashrc
rm -rf processor* log.*
decomposePar > log.decomposePar 2>&1
mpirun -np 8 freeBody6DoFPimpleFoam -parallel > log.freeBody6DoFPimpleFoam 2>&1
reconstructPar -latestTime > log.reconstructPar 2>&1 || true
EOF
chmod +x "$CASE/Allrun"

echo "Built $CASE successfully."
