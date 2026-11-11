"""Standard renders of the robot and the cell (PyVista, offscreen)."""
import os

import cadquery as cq
import numpy as np

from delta_cad.lib import box, place
from delta_cad.render import render


def _sel(items, pred):
    return [(s, c) for n, s, c, m, ms in items if pred(n)]


def section(items, M, keep_neg_z=True):
    """Cut every shape with the half-space z<0 of frame M (world 4x4)."""
    hs = place(box(-400, 400, -400, 400, -400, 0), M)
    out = []
    for s, c in items:
        try:
            r = s.intersect(hs)
            if r.Volume() > 1e-3:
                out.append((r, c))
        except Exception:
            pass
    return out


def render_all(bodies, extras, cell_items, out):
    robot = []
    for b in bodies.values():
        robot += list(b.world_parts())
    robot_ex = robot + extras
    r_all = _sel(robot_ex, lambda n: True)
    render(r_all, os.path.join(out, "robot_iso.png"), view=((650, -650, 260), (0, 0, -70), (0, 0, 1)))
    render(r_all, os.path.join(out, "robot_front.png"), view=((0, -1200, -110), (0, 0, -110), (0, 0, 1)), parallel=True, pscale=250)
    if cell_items:
        allp = r_all + _sel(cell_items, lambda n: True)
        render(allp, os.path.join(out, "cell_iso.png"), view=((950, -1150, 650), (-90, 0, -130), (0, 0, 1)))
        render(allp, os.path.join(out, "cell_top.png"), view=((-90, 0, 1500), (-90, 0, -130), (0, 1, 0)), parallel=True, pscale=380)
        conv = _sel(cell_items, lambda n: n[0] in "BCS" or n.startswith("E14") or n.startswith("M23"))
        render(conv, os.path.join(out, "conveyor.png"), view=((420, -650, 40), (-90, -20, -310), (0, 0, 1)))
        drive = _sel(cell_items, lambda n: n.startswith("C") and not n.startswith("C06"))
        render(drive, os.path.join(out, "conveyor_drive.png"), view=((330, -330, -250), (175, -130, -320), (0, 0, 1)))
        cam = _sel(cell_items, lambda n: n.startswith("M22") or n.startswith("E15") or n.startswith("E16") or "standoff_" in n
                   or "camera" in n)
        render(cam, os.path.join(out, "camera_head.png"), view=((-80, -260, -260), (-192.5, -38.5, -125), (0, 0, 1)))
        elec = _sel(cell_items, lambda n: n.startswith("E0") or n.startswith("E1") and not n.startswith("E14")
                    and not n.startswith("E15") and not n.startswith("E16") or n.startswith("M25") or "carrier" in n)
        render(elec, os.path.join(out, "electronics.png"), view=((-60, 20, -150), (-220, 240, -350), (0, 0, 1)))

    def arm1(n):
        last = n.split("/")[-1]
        if n.startswith("arm1/") or n.startswith("rod1"):
            return True
        toks = set(last.split("_"))
        return n.startswith("base/") and "M01" not in last and bool(toks & {"1", "1a", "1b", "10", "11", "12", "13"})
    mod = _sel(robot_ex, lambda n: arm1(n) and "screw_M4" not in n)
    render(mod, os.path.join(out, "drive_motor_side.png"), view=((300, 230, 170), (110, 30, 40), (0, 0, 1)))
    render(mod, os.path.join(out, "drive_encoder_side.png"), view=((260, -230, 90), (110, -20, 30), (0, 0, 1)))
    render(_sel(robot_ex, lambda n: n.startswith("arm1/") or n.startswith("rod1") or n.startswith("P09_spring_1")),
           os.path.join(out, "elbow_joint.png"), view=((400, -170, 20), (250, 0, -80), (0, 0, 1)))
    plat = _sel(robot_ex, lambda n: n.startswith("platform") or n.startswith("rod") or n.startswith("P09_spring"))
    render(plat, os.path.join(out, "platform.png"), view=((150, -170, -150), (0, 0, -245), (0, 0, 1)))
    # section through arm 1: plane containing the arm axis and the upper arm
    Aq = bodies["arm1"].M
    no_belt = [(s, c) for s, c in mod if c != "belt"]
    sec = section(no_belt, Aq)
    n = Aq[:3, 2]
    eye = Aq[:3, 3] + 420 * n + np.array([0, 0, 0])
    render(sec, os.path.join(out, "section_arm_axis.png"),
           view=(tuple(eye), tuple(Aq[:3, 3] + 25 * Aq[:3, 1] + 30 * Aq[:3, 0]), tuple(Aq[:3, 0])), parallel=True, pscale=62)
