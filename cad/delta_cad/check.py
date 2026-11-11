"""Interference and clearance checks of the robot over a set of poses (exact B-rep distances)."""
import math

import cadquery as cq
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

from . import robot
from .dims import TH_MAX, TH_MIN

POSES = [(0, 0, -240), (0, 0, -175), (0, 0, -320)] + \
        [(120 * math.cos(math.radians(a)), 120 * math.sin(math.radians(a)), -240) for a in range(0, 360, 60)] + \
        [(-80, -86.5, -240), (80, -86.5, -240), (-80, 9.5, -240), (80, 9.5, -240),
         (-75, 86.5, -240), (75, 86.5, -240), (-75, 86.5, -215), (75, 86.5, -215), (0, 86.5, -215)]


def dist(a, b):
    ca = cq.Compound.makeCompound(a) if isinstance(a, list) else a
    cb = cq.Compound.makeCompound(b) if isinstance(b, list) else b
    d = BRepExtrema_DistShapeShape(ca.wrapped, cb.wrapped)
    d.Perform()
    return d.Value() if d.IsDone() else float("nan")


def groups(bodies):
    """Named shape groups in world coordinates."""
    W = {}
    for b in bodies.values():
        for name, shp, colour, mat, mass in b.world_parts():
            W[name] = shp
    from .lib import cyl
    sp = {n: cyl(5.5, p0, p1).val() for n, p0, p1, pin in robot.springs(bodies)}   # envelope of the coil
    g = {}
    for i in (1, 2, 3):
        g[f"arm{i}_moving"] = [W[k] for k in W if k.startswith(f"arm{i}/") and "ball_stud" not in k
                               and "bearing_inner" not in k and "spacer_in" not in k and "P03_shaft" not in k and "M11" not in k
                               and "pulley" not in k]
        g[f"arm{i}_link"] = [W[k] for k in W if k.startswith(f"arm{i}/") and any(
            s in k for s in ("M06", "M05", "ball_stud_elbow", "insert_M3_elbow"))]
        g[f"elbow{i}"] = [W[k] for k in W if k.startswith(f"arm{i}/") and ("M05" in k or "insert_M3_elbow" in k)]
        g[f"static{i}"] = [W[k] for k in W if k.startswith("base/") and f"_{i}" in k.split("/")[1]
                           and not any(x in k for x in ("belt", "bearing", "stop_pin", "M01", "screw_M4", "insert_M4"))]
        g[f"stops{i}"] = [W[k] for k in W if k.startswith(f"base/P05_stop_pin_{i}")]
        for ab in "ab":
            rod = [W[k] for k in W if k.startswith(f"rod{i}{ab}/")]
            g[f"rod{i}{ab}"] = rod
            g[f"rodend{i}{ab}"] = [W[k] for k in W if k.startswith(f"rod{i}{ab}/M07") or k.startswith(f"rod{i}{ab}/P10")]
            g[f"stud_e{i}{ab}"] = [W[k] for k in W if k == f"arm{i}/P08_ball_stud_elbow_{ab}_stud"]
            g[f"stud_p{i}{ab}"] = [W[k] for k in W if k == f"platform/P08_ball_stud_{i}{ab}_stud"]
        g[f"springs{i}"] = [sp[k] for k in sp if k.startswith(f"P09_spring_{i}_")]
    g["platform_body"] = [W[k] for k in W if k.startswith("platform/") and "ball_stud" not in k
                          and "insert_M3" not in k]
    return g


def check_pose(x):
    bodies, th = robot.build(x, with_belts=False)
    g = groups(bodies)
    res = {}

    def put(key, val):
        res[key] = min(res.get(key, 1e9), val)
    for i in (1, 2, 3):
        put("arm moving parts vs bracket, motor, encoder", dist(g[f"arm{i}_moving"], g[f"static{i}"]))
        if TH_MIN + 1.0 < th[i - 1] < TH_MAX - 1.0:
            put("arm vs hard-stop pins (away from the stops)", dist(g[f"arm{i}_moving"], g[f"stops{i}"]))
        for ab in "ab":
            put("rod end vs its own elbow stud (swing)", dist(g[f"rodend{i}{ab}"], g[f"stud_e{i}{ab}"]))
            put("rod end vs its own platform stud (swing)", dist(g[f"rodend{i}{ab}"], g[f"stud_p{i}{ab}"]))
            put("rod vs own elbow block", dist(g[f"rod{i}{ab}"], g[f"elbow{i}"]))
            put("rod vs platform body", dist(g[f"rod{i}{ab}"], g["platform_body"]))
            for j in (1, 2, 3):
                if j != i:
                    put("rod vs other arms' upper arms", dist(g[f"rod{i}{ab}"], g[f"arm{j}_link"]))
                    for cd in "ab":
                        if (j, cd) > (i, ab):
                            put("rod vs rods of other arms", dist(g[f"rod{i}{ab}"], g[f"rod{j}{cd}"]))
        put("springs vs elbow block", dist(g[f"springs{i}"], g[f"elbow{i}"]))
        put("springs vs platform body", dist(g[f"springs{i}"], g["platform_body"]))
        put("springs vs studs", dist(g[f"springs{i}"], g[f"stud_e{i}a"] + g[f"stud_e{i}b"] +
                                     g[f"stud_p{i}a"] + g[f"stud_p{i}b"]))
    return res, th
