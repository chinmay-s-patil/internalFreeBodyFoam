#!/usr/bin/env python3
"""
generate_mesh.py
Generates blockMeshDict for 2D rodRoller case with right-handed hex blocks:
- Inner rotatingZone between R_rod and R_ami (8 blocks)
- Outer stationary zone between R_ami and outer rectangular box (8 blocks)
- Circular cyclicAMI interface at R_ami
- 1 cell thick in Z with empty front/back patches
"""

import math

def generate_dict():
    R_rod = 0.05    # Rod radius = 5 cm (diameter = 10 cm)
    R_ami = 0.10    # AMI radius = 10 cm
    X_box = 0.40    # Domain half-width = 40 cm (8D width)
    Y_box = 0.80    # Domain half-height = 80 cm (16D height)
    Z_half = 0.025  # 5 cm depth in Z

    # 8 radial angles (octants)
    angles = [i * math.pi / 4.0 for i in range(8)]
    
    # Outer box 8 boundary points corresponding to octants:
    box_pts = [
        (X_box, 0.0),
        (X_box, Y_box),
        (0.0, Y_box),
        (-X_box, Y_box),
        (-X_box, 0.0),
        (-X_box, -Y_box),
        (0.0, -Y_box),
        (X_box, -Y_box)
    ]
    
    verts = []
    # Layer 0: Z = -Z_half
    # Layer 1: Z = +Z_half
    for z in [-Z_half, Z_half]:
        # Ring 0: Rod vertices (0..7 in layer)
        for a in angles:
            verts.append((R_rod * math.cos(a), R_rod * math.sin(a), z))
        # Ring 1: Inner AMI vertices (8..15 in layer)
        for a in angles:
            verts.append((R_ami * math.cos(a), R_ami * math.sin(a), z))
        # Ring 2: Outer AMI vertices (16..23 in layer)
        for a in angles:
            verts.append((R_ami * math.cos(a), R_ami * math.sin(a), z))
        # Ring 3: Box vertices (24..31 in layer)
        for pt in box_pts:
            verts.append((pt[0], pt[1], z))

    def idx(ring, octant, layer):
        return layer * 32 + ring * 8 + (octant % 8)

    # Resolution
    nr_inner = 15
    nt_inner = 16
    nr_outer = 25
    nt_outer = 16
    nz = 1

    # Right-handed convention:
    # x1: v0 -> v1 (radial outward: ring 0 -> ring 1)
    # x2: v0 -> v3 (tangential: octant i -> octant i+1)
    # x3: v0 -> v4 (axial: layer 0 -> layer 1)
    # (v1-v0) x (v3-v0) . (v4-v0) = (er x etheta) . ez = +1 > 0
    blocks_str = ""
    # Inner zone (rotatingZone)
    for i in range(8):
        i_next = (i + 1) % 8
        v0 = idx(0, i, 0)
        v1 = idx(1, i, 0)
        v2 = idx(1, i_next, 0)
        v3 = idx(0, i_next, 0)
        v4 = idx(0, i, 1)
        v5 = idx(1, i, 1)
        v6 = idx(1, i_next, 1)
        v7 = idx(0, i_next, 1)
        blocks_str += f"    hex ({v0} {v1} {v2} {v3} {v4} {v5} {v6} {v7}) rotatingZone ({nr_inner} {nt_inner} {nz}) simpleGrading (1 1 1)\n"

    blocks_str += "\n"
    # Outer zone (stationary)
    for i in range(8):
        i_next = (i + 1) % 8
        v0 = idx(2, i, 0)
        v1 = idx(3, i, 0)
        v2 = idx(3, i_next, 0)
        v3 = idx(2, i_next, 0)
        v4 = idx(2, i, 1)
        v5 = idx(3, i, 1)
        v6 = idx(3, i_next, 1)
        v7 = idx(2, i_next, 1)
        blocks_str += f"    hex ({v0} {v1} {v2} {v3} {v4} {v5} {v6} {v7}) ({nr_outer} {nt_outer} {nz}) simpleGrading (1 1 1)\n"

    # Edges (arcs for circles)
    edges_str = ""
    for layer in [0, 1]:
        z = -Z_half if layer == 0 else Z_half
        # Rod circle arcs (Ring 0)
        for i in range(8):
            i_next = (i + 1) % 8
            mid_a = (i + 0.5) * math.pi / 4.0
            v_start = idx(0, i, layer)
            v_end = idx(0, i_next, layer)
            mx = R_rod * math.cos(mid_a)
            my = R_rod * math.sin(mid_a)
            edges_str += f"    arc {v_start} {v_end} ({mx:.6f} {my:.6f} {z:.6f})\n"

        # Inner AMI circle arcs (Ring 1)
        for i in range(8):
            i_next = (i + 1) % 8
            mid_a = (i + 0.5) * math.pi / 4.0
            v_start = idx(1, i, layer)
            v_end = idx(1, i_next, layer)
            mx = R_ami * math.cos(mid_a)
            my = R_ami * math.sin(mid_a)
            edges_str += f"    arc {v_start} {v_end} ({mx:.6f} {my:.6f} {z:.6f})\n"

        # Outer AMI circle arcs (Ring 2)
        for i in range(8):
            i_next = (i + 1) % 8
            mid_a = (i + 0.5) * math.pi / 4.0
            v_start = idx(2, i, layer)
            v_end = idx(2, i_next, layer)
            mx = R_ami * math.cos(mid_a)
            my = R_ami * math.sin(mid_a)
            edges_str += f"    arc {v_start} {v_end} ({mx:.6f} {my:.6f} {z:.6f})\n"

    # Boundary definitions:
    # 1. rod (wall): faces at inner radius (v0-v4-v7-v3)
    rod_faces = ""
    for i in range(8):
        i_next = (i + 1) % 8
        v0 = idx(0, i, 0)
        v3 = idx(0, i_next, 0)
        v7 = idx(0, i_next, 1)
        v4 = idx(0, i, 1)
        rod_faces += f"            ({v0} {v4} {v7} {v3})\n"

    # 2. innerAMI (cyclicAMI): outer faces of inner zone (v1-v2-v6-v5)
    inner_ami_faces = ""
    for i in range(8):
        i_next = (i + 1) % 8
        v1 = idx(1, i, 0)
        v2 = idx(1, i_next, 0)
        v6 = idx(1, i_next, 1)
        v5 = idx(1, i, 1)
        inner_ami_faces += f"            ({v1} {v2} {v6} {v5})\n"

    # 3. outerAMI (cyclicAMI): inner faces of outer zone (v0-v4-v7-v3)
    outer_ami_faces = ""
    for i in range(8):
        i_next = (i + 1) % 8
        v0 = idx(2, i, 0)
        v3 = idx(2, i_next, 0)
        v7 = idx(2, i_next, 1)
        v4 = idx(2, i, 1)
        outer_ami_faces += f"            ({v0} {v3} {v7} {v4})\n"

    # 4. inlet (bottom): faces at Y = -Y_box (box octants 5->6, 6->7)
    inlet_faces = ""
    for i in [5, 6]:
        i_next = i + 1
        v1 = idx(3, i, 0)
        v2 = idx(3, i_next, 0)
        v6 = idx(3, i_next, 1)
        v5 = idx(3, i, 1)
        inlet_faces += f"            ({v1} {v2} {v6} {v5})\n"

    # 5. outlet (top): faces at Y = +Y_box (box octants 1->2, 2->3)
    outlet_faces = ""
    for i in [1, 2]:
        i_next = i + 1
        v1 = idx(3, i, 0)
        v2 = idx(3, i_next, 0)
        v6 = idx(3, i_next, 1)
        v5 = idx(3, i, 1)
        outlet_faces += f"            ({v1} {v2} {v6} {v5})\n"

    # 6. sides: box octants 7->0, 0->1 (right) and 3->4, 4->5 (left)
    sides_faces = ""
    for i in [7, 0, 3, 4]:
        i_next = (i + 1) % 8
        v1 = idx(3, i, 0)
        v2 = idx(3, i_next, 0)
        v6 = idx(3, i_next, 1)
        v5 = idx(3, i, 1)
        sides_faces += f"            ({v1} {v2} {v6} {v5})\n"

    # 7. back (z = -Z_half) and front (z = +Z_half)
    back_faces = ""
    front_faces = ""
    # Inner zone
    for i in range(8):
        i_next = (i + 1) % 8
        v0 = idx(0, i, 0)
        v1 = idx(1, i, 0)
        v2 = idx(1, i_next, 0)
        v3 = idx(0, i_next, 0)
        back_faces += f"            ({v0} {v3} {v2} {v1})\n"
        v4 = idx(0, i, 1)
        v5 = idx(1, i, 1)
        v6 = idx(1, i_next, 1)
        v7 = idx(0, i_next, 1)
        front_faces += f"            ({v4} {v5} {v6} {v7})\n"

    # Outer zone
    for i in range(8):
        i_next = (i + 1) % 8
        v0 = idx(2, i, 0)
        v1 = idx(3, i, 0)
        v2 = idx(3, i_next, 0)
        v3 = idx(2, i_next, 0)
        back_faces += f"            ({v0} {v3} {v2} {v1})\n"
        v4 = idx(2, i, 1)
        v5 = idx(3, i, 1)
        v6 = idx(3, i_next, 1)
        v7 = idx(2, i_next, 1)
        front_faces += f"            ({v4} {v5} {v6} {v7})\n"

    verts_str = "\n".join([f"    ( {v[0]:.6f} {v[1]:.6f} {v[2]:.6f} ) // {k}" for k, v in enumerate(verts)])

    content = f"""/*--------------------------------*- C++ -*----------------------------------*\\
| =========                 |                                                 |
| \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox           |
|  \\\\    /   O peration     | Version:  2412                                  |
|   \\\\  /    A nd           | Website:  www.openfoam.com                      |
|    \\\\/     M anipulation  |                                                 |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      blockMeshDict;
}}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

scale   1.0;

vertices
(
{verts_str}
);

blocks
(
{blocks_str}
);

edges
(
{edges_str}
);

boundary
(
    rod
    {{
        type            wall;
        inGroups        1(wall);
        faces
        (
{rod_faces}        );
    }}
    innerAMI
    {{
        type            cyclicAMI;
        inGroups        1(cyclicAMI);
        matchTolerance  0.001;
        neighbourPatch  outerAMI;
        transform       noOrdering;
        faces
        (
{inner_ami_faces}        );
    }}
    outerAMI
    {{
        type            cyclicAMI;
        inGroups        1(cyclicAMI);
        matchTolerance  0.001;
        neighbourPatch  innerAMI;
        transform       noOrdering;
        faces
        (
{outer_ami_faces}        );
    }}
    inlet
    {{
        type            patch;
        faces
        (
{inlet_faces}        );
    }}
    outlet
    {{
        type            patch;
        faces
        (
{outlet_faces}        );
    }}
    sides
    {{
        type            patch;
        faces
        (
{sides_faces}        );
    }}
    front
    {{
        type            empty;
        inGroups        1(empty);
        faces
        (
{front_faces}        );
    }}
    back
    {{
        type            empty;
        inGroups        1(empty);
        faces
        (
{back_faces}        );
    }}
);

// ************************************************************************* //
"""
    return content

if __name__ == "__main__":
    with open("tutorials/rodRoller/system/blockMeshDict", "w") as f:
        f.write(generate_dict())
    print("Successfully generated tutorials/rodRoller/system/blockMeshDict")
