"""Build the delta cell CAD v2 from config/robot.yaml.

Run from the repo root:  python3 cad/build.py [--pose x y z] [--no-cell] [--no-render]
Outputs in cad/out/:
  parts/        every designed part (M..) as STEP and STL in its own frame, ready for SolidWorks or slicing
  robot_assembly.step   robot at the pose, one sub-assembly per rigid body (base, arm1..3, rod1a..3b, platform)
  cell_assembly.step    robot + cell
  mujoco/       delta_cell.xml (MJCF), meshes/ per body and colour, bodies.json (frames, joints, masses, inertia)
  report.json   masses, body properties, drive geometry
  renders/      PNG views
"""
import argparse
import json
import os
import sys
import time

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from delta_cad import cell, mjcf, parts as P, robot, vendor as Vd  # noqa: E402
from delta_cad.dims import CFG, C_BELT, N_BELT, ROOT  # noqa: E402
from delta_cad.lib import DENSITY, PRINT_FILL, color, mass_props, place, write_stl, tessellate  # noqa: E402

OUT = os.path.join(ROOT, "cad", "out")

DESIGNED = {  # ref: (function, material, how it is made)
    "M01_base_plate": (P.m01_base_plate, "alu", "6061 plate 6 mm, CNC or waterjet + drill"),
    "M02_pivot_bracket": (P.m02_pivot_bracket, "petg_cf", "print, flange down, 5 walls, 40 % gyroid"),
    "M04_arm_hub": (P.m04_arm_hub, "petg_cf", "print, axis vertical, 100 % around the bore"),
    "M05_elbow_block": (P.m05_elbow_block, "petg_cf", "print, socket axis vertical"),
    "M06_upper_tube": (P.m06_upper_tube, "carbon", "pultruded tube 12 x 10, cut to 135"),
    "M07_rod_end": (P.m07_rod_end, "petg_cf", "print x12 (6 plain + 6 mirrored), seat up"),
    "M08_rod_tube": (P.m08_rod_tube, "carbon", "pultruded tube 6 x 4, cut to 242"),
    "M09_platform": (P.m09_platform, "petg_cf", "print, flat"),
    "M10_magnet_holder": (P.m10_magnet_holder, "pla", "print"),
    "M11_pulley_120T": (P.m11_pulley_120t, "petg_cf", "print flat, 0.12 mm layers, check the profile"),
}


def trsf(M):
    t = gp_Trsf()
    t.SetValues(*M[0, :4], *M[1, :4], *M[2, :4])
    return cq.Location(t)


def _unique(names):
    seen = {}
    out = []
    for n in names:
        k = seen.get(n, 0)
        seen[n] = k + 1
        out.append(n if k == 0 else f"{n}_{k}")
    return out


def assembly(bodies, extras, cell_items, name):
    top = cq.Assembly(name=name)
    for b in bodies.values():
        sub = cq.Assembly(name=b.name)
        for pname, (_, shp, col, mat, _) in zip(_unique([p[0] for p in b.parts]), b.parts):
            sub.add(shp, name=pname, color=color(col))
        top.add(sub, name=b.name, loc=trsf(b.M))
    ex = cq.Assembly(name="springs_and_tube")
    for k, (pname, shp, col, mat, _) in enumerate(extras):
        ex.add(shp, name=f"{pname}_{k}", color=color(col))
    top.add(ex, name="springs_and_tube")
    if cell_items:
        cs = cq.Assembly(name="cell")
        for pname, (_, shp, col, mat, _) in zip(_unique([p[0] for p in cell_items]), cell_items):
            cs.add(shp, name=pname, color=color(col))
        top.add(cs, name="cell")
    return top


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", nargs=3, type=float, default=[0.0, 0.0, 1000 * CFG["cell"]["pick_z"]])
    ap.add_argument("--no-cell", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--cache", action="store_true", help="reuse out/.cache.pkl of the robot and cell (same pose)")
    ap.add_argument("--no-step", action="store_true", help="skip the part and assembly STEP exports")
    a = ap.parse_args()
    t0 = time.time()
    for d in ("parts", "mujoco", "renders"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)

    report = {"pose_mm": a.pose, "belt": dict(teeth=N_BELT, pitch_length_mm=2.0 * N_BELT,
                                                 centre_distance_mm=round(C_BELT, 3)), "parts": {}}
    print("parts ...", flush=True)
    for ref, (fn, mat, how) in DESIGNED.items():
        wp = fn()
        if not a.no_step:
            cq.exporters.export(wp, os.path.join(OUT, "parts", ref + ".step"))
            cq.exporters.export(wp, os.path.join(OUT, "parts", ref + ".stl"), tolerance=0.02, angularTolerance=0.1)
        m_solid = wp.val().Volume() * DENSITY[mat]
        bb = wp.val().BoundingBox()
        report["parts"][ref] = dict(material=mat, made=how, volume_cm3=round(wp.val().Volume() / 1000, 2),
                                    mass_solid_g=round(m_solid, 1),
                                    mass_printed_g=round(m_solid * PRINT_FILL, 1) if mat in mjcf.PRINTED else None,
                                    bbox_mm=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)])
    print(f"  {time.time() - t0:.0f} s; robot ...", flush=True)
    import pickle
    cache = os.path.join(OUT, ".cache.pkl")
    if a.cache and os.path.exists(cache):
        bodies, th, extras, cell_items = pickle.load(open(cache, "rb"))
    else:
        bodies, th = robot.build(a.pose, with_belts=True)
        extras = robot.spring_shapes(bodies) + robot.vacuum_tube(bodies)
        cell_items = [] if a.no_cell else cell.build_cell()
        pickle.dump((bodies, th, extras, cell_items), open(cache, "wb"))
    report["theta_deg"] = [round(float(v), 3) for v in th]
    print(f"  {time.time() - t0:.0f} s; STEP ...", flush=True)
    if not a.no_step:
        assembly(bodies, extras, [], "robot_assembly").save(os.path.join(OUT, "robot_assembly.step"))
    if cell_items and not a.no_step:
        assembly(bodies, extras, cell_items, "cell_assembly").save(os.path.join(OUT, "cell_assembly.step"))

    print(f"  {time.time() - t0:.0f} s; MuJoCo ...", flush=True)
    # rotor inertia: datasheet rotor + 20T pulley from the CAD (aluminium)
    p20 = Vd.gt2_pulley_20t_10mm()[0][1]
    _, _, I20 = mass_props(p20, DENSITY["alu"])
    J20 = I20[1, 1] * 1e-9
    belt_m = 0.0037                                                  # 10 mm GT2, 300 mm, kg
    J_belt_motor_side = belt_m * (robot.gt2.pd(20) / 2000) ** 2
    J_rot = CFG["masses"]["J_rotor"] + J20 + J_belt_motor_side
    xml, rep = mjcf.export(bodies, cell_items, os.path.join(OUT, "mujoco"), J_rot, robot.springs(bodies))
    report["mujoco"] = dict(xml=os.path.relpath(xml, ROOT), rotor_side_inertia_kgm2=J_rot,
                            bodies={k: dict(mass_g=round(v["mass"] * 1000, 2)) for k, v in rep["bodies"].items()})
    m_moving = sum(v["mass"] for k, v in rep["bodies"].items()) * 1000
    report["mujoco"]["moving_mass_g"] = round(m_moving, 1)

    # one STL per designed part already; plus whole-robot STL for quick viewing
    write_stl(os.path.join(OUT, "robot_preview.stl"),
              [tessellate(s, 0.1, 0.3) for b in bodies.values() for _, s, _, _, _ in b.world_parts()])
    json.dump(report, open(os.path.join(OUT, "report.json"), "w"), indent=1)

    if not a.no_render:
        print(f"  {time.time() - t0:.0f} s; renders ...", flush=True)
        from render_views import render_all
        render_all(bodies, extras, cell_items, os.path.join(OUT, "renders"))
    print(f"done in {time.time() - t0:.0f} s", flush=True)
    print(json.dumps({k: v for k, v in report.items() if k != "parts"}, indent=1))


if __name__ == "__main__":
    main()
