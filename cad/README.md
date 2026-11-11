# Delta cell CAD, v2

Generated from `config/robot.yaml` by the `cad/delta_cad` package (CadQuery). Units mm.
World frame = robot frame: origin at the robot centre in the plane of the arm axes, z up, x along the belt flow,
arms at 0 / 120 / 240 deg. v1 (`build_cad_v1.py`) is kept for reference only.

## What is in `out/`

| Path | Content |
|---|---|
| `robot_assembly.step` | Robot at the pick pose (0, 0, -240), one **sub-assembly per rigid body**: `base`, `arm1..3`, `rod1a..rod3b`, `platform` (+ springs and vacuum tube) |
| `cell_assembly.step` | Robot + frame, conveyor and its drive, bins, chute, camera head, beam sensor, electronics tray, e-stop, sample caps |
| `parts/M*.step, .stl` | Every designed part in its own frame (print or machine from these) |
| `mujoco/delta_cell.xml` | MuJoCo model built from the CAD (meshes per body, CAD masses and inertias, joints, loop closures, springs, motors, sensors) |
| `mujoco/bodies.json` | Every body: parent, frame, joint type, axis, anchor, range, mass, centre of mass, inertia, sites (SI units) |
| `mujoco/validation.json` | Result of `validate_mujoco.py` |
| `drawings/Delta_Cell_CAD_v2_Drawings.pdf` | Sheet 0 = axial stack of an arm module; one sheet per designed part with views and the dimensions that matter |
| `renders/*.png` | Views of the robot, drive, joints, platform, section through the arm axis, cell |
| `report.json`, `clearance.json` | Part masses, body masses, drive geometry; minimum clearances over 20 poses |

## Open it in SolidWorks

1. File > Open, type STEP, pick `robot_assembly.step` (or `cell_assembly.step`).
2. Options: keep "Import multiple bodies as parts" and "Assembly structure". Each rigid body arrives as its own
   sub-assembly, so the moving groups are already separated.
3. Run Import Diagnostics if SolidWorks offers it, then Save As `.SLDASM`.
4. To make it move: fix `base`; float the others; add the mates from the joint table below
   (Concentric + Coincident for the hinges, a Coincident point-to-point mate at each ball centre, limit angles for the stops).
5. Parts arrive without a feature tree. For small edits use Direct Editing (Move Face) or FeatureWorks; for dimension changes,
   edit `delta_cad/dims.py` (or `robot.yaml`) and regenerate. The drawings give everything needed to remodel a part natively.

## Joints and moving bodies

| Joint | Type | Parent -> child | Where | Axis | Range |
|---|---|---|---|---|---|
| j1, j2, j3 | hinge | base -> arm i | arm axis: radius 110 at 0/120/240 deg, z = 0 | tangential (+y of the arm frame); positive = arm down | -25 to +80 deg (hard stops) |
| ball{i}{a,b} (6) | ball | arm i -> rod | elbow ball centres: x = 170 on the arm, y = +-20 | - | cup swing limit 123 deg |
| close{i}{a,b} (6) | connect (loop closure) | rod -> platform | platform ball centres: radius 30, +-20 | - | - |
| platform | free (closed by the 6 connects) | world -> platform | - | - | stays level by the parallelograms |
| spring{i}_{top,bottom} (6) | spatial tendon | rod a <-> rod b | spring pins 15 mm from each ball | - | 1.2 N/mm, 12 N preload |
| m1..m3 | motor | on j1..j3 | gear 6 x 0.95 | - | +-0.59 N m |

The rotor and the 20T pulley spin 6x faster than the arm; their inertia is in the hinge `armature` (x 36).
Body frames: arm = arm frame rotated by the reference angle (35.54 deg), so the hinge reads the absolute theta;
rod frame = origin at the elbow ball, z' along the rod, y' = the ball-pair axis.

```bash
python3 -m mujoco.viewer --mjcf cad/out/mujoco/delta_cell.xml   # look at it
python3 cad/validate_mujoco.py                                   # loop closure, hold, moves vs the kinematics
```

## Regenerate

```bash
pip install cadquery pyyaml numpy pyvista mujoco playwright   # inside the delta venv
python3 cad/build.py                    # all outputs, robot at (0, 0, -240)  (~6 min)
python3 cad/build.py --pose 75 86.5 -215 --no-cell
python3 cad/drawings.py                 # drawing PDF
python3 -c "import sys; sys.path.insert(0,'cad'); from delta_cad import check; print(check.check_pose((0,0,-240))[0])"
```

## Designed parts

| Ref | Part | Qty | Material, process |
|---|---|---|---|
| M01 | Base plate 340 x 340 x 6, R15 corners | 1 | 6061-T6, waterjet or CNC + drill |
| M02 | Pivot bracket with integrated motor plate and gussets | 3 | PETG-CF print |
| M04 | Arm hub: D-bore split clamp (M3 + nut trap), bonded tube socket, hard-stop tail | 3 | PETG-CF |
| M05 | Elbow block: bonded socket, 2 M3 inserts for the ball studs | 3 | PETG-CF |
| M06 | Upper arm tube 12 x 10 x 135 | 3 | carbon |
| M07 | Rod end: spherical cup, bonded socket, clevis for the spring pin | 12 (6 mirrored) | PETG-CF |
| M08 | Rod tube 6 x 4 x 242 (260.0 ball to ball after bonding) | 6 | carbon |
| M09 | Platform: 3 lugs with M3 inserts, M5 through-insert for cup + fitting, ADXL345 posts | 1 | PETG-CF |
| M10 | Magnet holder | 3 | PLA |
| M11 | GT2 120T pulley, real tooth profile, 10 mm belt | 3 | PETG-CF (or buy aluminium) |
| M22 | Camera head (camera + 2 LED bars) | 1 | PETG-CF |
| M23 | Beam sensor brackets | 2 | PETG-CF |
| M25 | Electronics tray | 1 | aluminium 3 mm |
| B02-B05 | Bin tray, tray brackets, overflow chute, tote | | print / aluminium / PP |
| C01-C16 | Conveyor: side plates, rollers, shafts, KFL08, tensioners, bed, angles, cross members, motor bracket | | aluminium |

## Purchased parts modelled with catalogue dimensions

NEMA 17 17HS19-2004S1 (42.3 sq x 48, boss 22 x 2, shaft 5 x 24 with D-flat, M3 on 31) x4 ·
608-2RS bearing 8 x 22 x 7 x6 · 8 mm h6 shaft x 84 with two D-flats x3 · shaft collar 8 x 16 x 8 x3 ·
spacers 12 x 8.2 (8.5, 3.0, 2.5 long) · GT2 20T 10 mm 5 mm bore x3 · GT2 10 mm closed belt 300 (150 T) x3 ·
ball stud 8 mm ball, hex 7 AF, M3 x 6 x12 · extension spring 5 OD x 0.6 wire x6 · dowel 3 x 12 x12 · dowel 6 x 50 x6 ·
heat-set inserts M2.5 / M3 / M4 / M5 · ISO 4762 screws · AS5600 module 23 x 23 + 6 x 2.5 diametric magnet x3 ·
ADXL345 GY-291 · 10 mm bellows cup on M5 holder · M5 push-in fitting for 4 mm tube ·
conveyor: KFL08 x4, GT2 20T/60T 6 mm + 200 mm belt, rollers 30 x 156 · M18 through-beam pair ·
OV9281 USB camera, 2 LED bars 100 mm · Mean Well LRS-150-24, BlackPill F411, 4 x TMC2209, 12 V diaphragm pump,
3/2 valve, LM2596 buck · 2020 slot-6 extrusion, corner brackets, T-nuts · 22 mm e-stop in a box.

## Checks done on this version

- Every rod is 260.000 mm ball to ball at the build pose (built from the inverse kinematics).
- GT2 drive: belt teeth sit in the grooves of both pulleys (belt/pulley overlap 0.00 and 0.09 mm^3);
  exact centre distance for 150 teeth is **72.94 mm** (the 73.07 used before came from the approximate belt formula).
- Clearances at 20 poses (centre, top, bottom, radius 120 at 6 angles, the 4 pick-lane corners, bins, lifted bins),
  exact B-rep distances, worst case over all poses:

  | Pair | Min (mm) |
  |---|---|
  | Rod end vs its own elbow stud (swing) | 2.06 |
  | Rod end vs its own platform stud (swing) | 2.34 |
  | Rod vs own elbow block | 2.24 |
  | Rod vs platform | 2.10 |
  | Arm moving parts vs bracket, motor, encoder | 1.50 (the designed magnet-to-chip air gap) |
  | Arm vs hard-stop pins away from the stops | 1.01 (6 deg before the +80 stop) |
  | Springs vs elbow block / platform / studs | 6.03 / 3.85 / 8.93 |
  | Rods of different arms | 17.96 |
- MuJoCo: closure residual 0 at the reference; under PD hold and four moves the platform lands within 0.03 mm of the
  kinematic FK of the measured arm angles, tilt below 0.07 deg.

## Check before printing or buying

- Print the bearing-seat coupon and the ball/cup coupon first; adjust the 22.00 seat and the 8.1 cup in `dims.py`.
- Print a 20-tooth sector of M11 and try it with the belt before printing the full pulley.
- AS5600 and OV9281 modules vary between sellers: measure your boards' hole positions and change
  `POST_XZ` / the camera-head holes if they differ.
- Rod length is set by the bonding jig, not by the tube length: bond all six with the jig at 260.00.
