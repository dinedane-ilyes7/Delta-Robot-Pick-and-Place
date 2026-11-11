"""The cell around the robot: 2020 frame, belt conveyor with its drive, bins and tray, overflow chute,
camera head with LED bars, through-beam sensor, electronics tray, e-stop and sample caps.
World frame = robot frame (origin at the robot centre in the arm-axis plane, z up, x along the belt flow)."""
import math

import cadquery as cq
import numpy as np

from . import gt2, vendor as Vd
from .dims import CELL, Z_PLATE, BASE_PLATE, FRAME_RAIL_Y
from .lib import (box, cone, cyl, frame_from_z, hex_prism, place, rbox, rx, ry, rz, safe_chamfer, safe_fillet,
                  sphere, tr, xcyl, ycyl, zcyl)

BZ = 1000 * CELL["belt_z"]                 # belt surface -287
YC = 1000 * CELL["belt_centre_y"]          # -38.5
BW = 1000 * CELL["belt_width"]             # 150
X_IDLER, X_DRIVE = (1000 * v for v in CELL["belt_x_range"])   # -400, 200
TABLE = -367.0
ROLLER_D = 30.0
BELT_T = 1.5
Z_ROLL = BZ - BELT_T - ROLLER_D / 2        # -303.5
PLATE_IN = 80.0                            # side plate inner face at YC +- 80
PLATE_T = 5.0
FRAME_X = (-440.0, 260.0)
FRAME_Y = (-300.0, 300.0)
FRAME_TOP = Z_PLATE + BASE_PLATE["t"] + 20.0   # 126
CAM_X = 1000 * CELL["camera_pos"][0]
CAM_Z = 1000 * CELL["camera_pos"][2]
BEAM_X = 1000 * CELL["beam_x"]

_cache = {}


def _c(key, fn):
    if key not in _cache:
        _cache[key] = fn()
    return _cache[key]


# ------------------------------------------------------------------ 2020 extrusion (slot 6)
def profile_2020():
    sq = cq.Workplane("XY").rect(20, 20).extrude(1)
    sq = safe_fillet(sq, "|Z", 0.6)
    slot = (cq.Workplane("XY").polyline([(-3.1, 10.5), (-3.1, 8.2), (-5.6, 8.2), (-5.6, 6.6), (-3.0, 4.0),
                                         (3.0, 4.0), (5.6, 6.6), (5.6, 8.2), (3.1, 8.2), (3.1, 10.5)]).close()
            .extrude(3).translate((0, 0, -1)))
    for k in range(4):
        sq = sq.cut(slot.rotate((0, 0, 0), (0, 0, 1), 90 * k))
    sq = sq.cut(zcyl(4.2, -1, 2))
    return sq.faces("<Z").val()


def ext2020(p0, p1, xhint=None):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    face = _c("prof2020", profile_2020)
    solid = _c(f"ext{L:.2f}", lambda: cq.Workplane("XY").newObject([cq.Solid.extrudeLinear(face, cq.Vector(0, 0, L))]))
    return place(solid, frame_from_z(p0, p1 - p0, xhint))


def corner_bracket():
    """Cast 2020 corner bracket 20 x 20 x 20, legs along +x and +z from the origin, gusset in y."""
    b = box(0, 20, -10, 10, 0, 3).union(box(0, 3, -10, 10, 0, 20))
    g = (cq.Workplane("XZ", origin=(0, 1.5, 0)).polyline([(3, 3), (18, 3), (3, 18)]).close().extrude(3))
    b = b.union(g)
    for p0, p1 in (((12, 0, -1), (12, 0, 4)), ((-1, 0, 12), (4, 0, 12))):
        b = b.cut(cyl(5.3, p0, p1))
    return b


# ------------------------------------------------------------------ frame
def frame():
    items = []
    x0, x1 = FRAME_X
    y0, y1 = FRAME_Y
    zb, zt = TABLE, FRAME_TOP
    al = "alu"
    for X in (x0, x1):
        for Y in (y0, y1):
            items.append((f"F01_leg_{X:+.0f}_{Y:+.0f}", ext2020((X, Y, zb), (X, Y, zt)), al, "alu"))
    for Y in (y0, y1):
        for Z, nm in ((zb + 10, "bottom"), (zt - 10, "top")):
            items.append((f"F02_rail_x_{nm}_{Y:+.0f}", ext2020((x0 + 10, Y, Z), (x1 - 10, Y, Z), (0, 0, 1)), al, "alu"))
    for X in (x0, x1):
        for Z, nm in ((zb + 10, "bottom"), (zt - 10, "top")):
            items.append((f"F03_rail_y_{nm}_{X:+.0f}", ext2020((X, y0 + 10, Z), (X, y1 - 10, Z), (0, 0, 1)), al, "alu"))
    for Y in (-FRAME_RAIL_Y, FRAME_RAIL_Y):                              # rails carrying the base plate
        items.append((f"F04_plate_rail_{Y:+.0f}", ext2020((x0 + 10, Y, zt - 10), (x1 - 10, Y, zt - 10), (0, 0, 1)),
                      al, "alu"))
    for Y in (y0, y1):                                                   # camera posts and beam
        items.append((f"F05_camera_post_{Y:+.0f}", ext2020((CAM_X, Y, zb + 20), (CAM_X, Y, zt - 20)), al, "alu"))
    items.append(("F06_camera_beam", ext2020((CAM_X, y0 + 10, -105), (CAM_X, y1 - 10, -105), (0, 0, 1)), al, "alu"))
    # diagonals: both short faces and the back face
    for X in (x0, x1):
        items.append((f"F07_diagonal_{X:+.0f}", ext2020((X, y0 + 25, zb + 32), (X, y1 - 25, zt - 32), (1, 0, 0)), al, "alu"))
    items.append(("F07_diagonal_back", ext2020((x1 - 25, y1, zb + 32), (CAM_X + 25, y1, zt - 32), (0, 1, 0)), al, "alu"))
    # corner brackets at every rail-to-leg joint (inside corners)
    cb = _c("cb", corner_bracket)
    k = 0
    for X, sx in ((x0, 1), (x1, -1)):
        for Y, sy in ((y0, 1), (y1, -1)):
            for Z, sz in ((zb + 20, 1), (zt - 20, -1)):
                # bracket in the x-z plane (rail along x) and in the y-z plane (rail along y)
                Mx = tr(X + sx * 10, Y, Z) @ np.diag([sx, 1, sz, 1]) if sx * sz > 0 else \
                    tr(X + sx * 10, Y, Z) @ np.diag([sx, -1, sz, 1])
                My = tr(X, Y + sy * 10, Z) @ rz(90) @ (np.diag([sy, 1, sz, 1]) if sy * sz > 0 else np.diag([sy, -1, sz, 1]))
                items.append((f"F08_corner_bracket_{k}", place(cb, Mx), "alu_dark", "zinc"))
                items.append((f"F08_corner_bracket_{k + 1}", place(cb, My), "alu_dark", "zinc"))
                k += 2
    foot = _c("foot", lambda: zcyl(30, 0, 6).union(zcyl(8, 6, 16)).union(hex_prism(13, (0, 0, 9), (0, 0, 15))))
    for X in (x0, x1):
        for Y in (y0, y1):
            items.append((f"F09_foot_{X:+.0f}_{Y:+.0f}", place(foot, tr(X, Y, zb - 16)), "rubber", "rubber"))
    return items


# ------------------------------------------------------------------ conveyor
def conveyor():
    items = []
    yf, yb = YC - PLATE_IN - PLATE_T, YC + PLATE_IN                      # plate y starts
    xa, xb = X_IDLER - 25, X_DRIVE + 25
    za, zb_ = -352.0, BZ + 10.0

    def side_plate(front):
        p = rbox(xa, xb, 0, PLATE_T, za, zb_, 5.0, "|Y")
        for X, slot in ((X_IDLER, True), (X_DRIVE, False)):
            if slot:                                                     # idler: slotted for tension, +-8
                p = p.cut(cq.Workplane("XZ", origin=(X, PLATE_T + 1, Z_ROLL)).slot2D(16 + 9, 9, 0).extrude(PLATE_T + 2))
                for dz in (-18.5, 18.5):
                    p = p.cut(cq.Workplane("XZ", origin=(X, PLATE_T + 1, Z_ROLL + dz)).slot2D(16 + 5.5, 5.5, 0)
                              .extrude(PLATE_T + 2))
            else:
                p = p.cut(ycyl(10, -1, PLATE_T + 1, X, Z_ROLL))
                for dz in (-18.5, 18.5):
                    p = p.cut(ycyl(5.5, -1, PLATE_T + 1, X, Z_ROLL + dz))
        p = p.cut(ycyl(8.0, -1, PLATE_T + 1, BEAM_X, BZ + 5))          # through-beam window
        for X in (-300, -100, 100):                                      # cross members
            p = p.cut(ycyl(5.5, -1, PLATE_T + 1, X, -342))
        for X in (-330, -130, 70):                                       # lightening windows
            p = p.cut(cq.Workplane("XZ", origin=(X, PLATE_T + 1, -321)).slot2D(110, 22, 0).extrude(PLATE_T + 2))
        return p
    items.append(("C01_side_plate_front", side_plate(True).translate((0, yf, 0)).val(), "alu", "alu"))
    items.append(("C01_side_plate_back", side_plate(False).translate((0, yb, 0)).val(), "alu", "alu"))
    # rollers and shafts
    for X, nm in ((X_IDLER, "idler"), (X_DRIVE, "drive")):
        roll = ycyl(ROLLER_D, YC - 78, YC + 78, X, Z_ROLL)
        roll = safe_chamfer(roll, "%CIRCLE", 1.0)
        roll = roll.cut(ycyl(20, YC - 79, YC - 75, X, Z_ROLL)).cut(ycyl(20, YC + 75, YC + 79, X, Z_ROLL))
        items.append((f"C02_roller_{nm}", roll.val(), "alu", "alu"))
        caps = ycyl(20, YC - 78, YC - 75, X, Z_ROLL).union(ycyl(20, YC + 75, YC + 78, X, Z_ROLL))
        items.append((f"C02_roller_caps_{nm}", caps.val(), "black", "pp"))
        ext = 30.0 if nm == "drive" else 0.0
        sh = ycyl(8, yf - 13 - ext, yb + PLATE_T + 13, X, Z_ROLL)
        items.append((f"C03_shaft_{nm}", safe_chamfer(sh, "%CIRCLE", 0.4).val(), "chrome", "steel"))
        for y, M in ((yf, rx(90)), (yb + PLATE_T, rx(-90))):
            for suffix, wp, col, mat in _c("kfl", Vd.kfl08):
                items.append((f"C04_KFL08_{nm}_{'front' if y == yf else 'back'}_{suffix}",
                              place(wp, tr(X, y, Z_ROLL) @ M @ rz(90)), col, mat))
    # tension screws for the idler (M5 through a block, pushing the KFL08)
    # (the block sits downstream of the KFL08 and pushes it upstream)
    for y in (yf - 6, yb + PLATE_T + 6):
        xb0 = X_IDLER + 22
        blk = rbox(xb0, xb0 + 10, y - 6, y + 6, Z_ROLL - 7, Z_ROLL + 7, 1.5, "|Y")
        items.append((f"C05_tension_block_{y:+.0f}", blk.cut(xcyl(5, xb0 - 1, xb0 + 11, y, Z_ROLL)).val(),
                      "print", "petg_cf"))
        scr = xcyl(5, X_IDLER + 13.5, xb0 + 10, y, Z_ROLL).union(xcyl(8.5, xb0 + 10, xb0 + 15, y, Z_ROLL))
        items.append((f"C05_tension_screw_{y:+.0f}", scr.val(), "oxide", "steel"))
    # belt: flat 1.5 mm PVC loop
    L = X_DRIVE - X_IDLER
    xm = (X_DRIVE + X_IDLER) / 2
    outer = cq.Workplane("XZ", origin=(xm, YC + BW / 2, Z_ROLL)).slot2D(L + ROLLER_D + 2 * BELT_T, ROLLER_D + 2 * BELT_T, 0).extrude(BW)
    inner = cq.Workplane("XZ", origin=(xm, YC + BW / 2 + 1, Z_ROLL)).slot2D(L + ROLLER_D, ROLLER_D, 0).extrude(BW + 2)
    items.append(("C06_belt_PVC", outer.cut(inner).val(), "conv_belt", "pvc"))
    # slider bed and its angle supports
    items.append(("C07_slider_bed", rbox(X_IDLER + 20, X_DRIVE - 20, YC - 76, YC + 76, BZ - BELT_T - 2, BZ - BELT_T, 1.0, "|Z").val(),
                  "alu_dark", "alu"))
    for s in (-1, 1):
        y_in = YC + s * PLATE_IN
        ang = box(X_IDLER + 22, X_DRIVE - 22, min(y_in, y_in - s * 2), max(y_in, y_in - s * 2), BZ - BELT_T - 17, BZ - BELT_T - 2) \
            .union(box(X_IDLER + 22, X_DRIVE - 22, min(y_in, y_in - s * 15), max(y_in, y_in - s * 15),
                       BZ - BELT_T - 4, BZ - BELT_T - 2))
        items.append((f"C08_bed_angle_{'front' if s < 0 else 'back'}", ang.val(), "alu", "alu"))
    for X in (-300, -100, 100):
        items.append((f"C09_cross_2020_{X:+.0f}", ext2020((X, YC - PLATE_IN, -342), (X, YC + PLATE_IN, -342), (0, 0, 1)),
                      "alu", "alu"))
    for X in (X_IDLER, X_DRIVE):
        for y in (YC - PLATE_IN - PLATE_T / 2, YC + PLATE_IN + PLATE_T / 2):
            items.append((f"C10_foot_{X:+.0f}_{y:+.0f}", zcyl(22, TABLE, za).val(), "rubber", "rubber"))
    # drive: NEMA17 + GT2 20T -> 60T, 6 mm belt, 100 teeth, motor 30 deg below and upstream of the drive shaft
    C = gt2.solve_centre_distance(60, 20, 100)
    mx, mz = X_DRIVE - C * math.cos(math.radians(30)), Z_ROLL - C * math.sin(math.radians(30))
    y_teeth = (-143.0 + YC + 38.5, -136.0 + YC + 38.5)                     # teeth zone along y
    plate_y = (y_teeth[0] - 13.0, y_teeth[0] - 8.0)                       # motor bracket plate
    brk = rbox(mx - 28, mx + 28, plate_y[0], plate_y[1], mz - 28, mz + 34, 3.0, "|Y")
    brk = brk.cut(ycyl(22.4, plate_y[0] - 1, plate_y[1] + 1, mx, mz))
    for dx in (-15.5, 15.5):
        for dz in (-15.5, 15.5):
            brk = brk.cut(ycyl(3.4, plate_y[0] - 1, plate_y[1] + 1, mx + dx, mz + dz))
    items.append(("C11_motor_bracket", brk.val(), "alu", "alu"))
    for dx, dz in ((-23, 25), (-23, -12), (23, -12)):
        items.append((f"C12_standoff_{dx:+d}_{dz:+d}", ycyl(10, plate_y[1], yf, mx + dx, mz + dz).val(), "alu", "alu"))
    for suffix, wp, col, mat in _c("nema", Vd.nema17_17hs19):
        items.append((f"C13_conveyor_motor_{suffix}", place(wp, tr(mx, plate_y[0], mz) @ rx(90)), col, mat))
    path = gt2.BeltPath((X_DRIVE, Z_ROLL), 60, (mx, mz), 20)
    belt, ph2 = gt2.belt_solid(path, 6.0, 0.0)
    items.append(("C14_belt_GT2_6mm_200", place(belt, tr(0, y_teeth[1] - 0.5, 0) @ rx(90)), "belt", "neoprene"))
    p60 = Vd.gt2_pulley_generic(60, 7.0, 8.0, 22.0, 7.0, 42.0)
    for suffix, wp, col, mat in p60:
        items.append(("C15_pulley_60T", place(wp, tr(X_DRIVE, y_teeth[1], Z_ROLL) @ rz(180)), col, mat))
    p20 = _rim20(180.0 - ph2)
    items.append(("C16_pulley_20T_6mm", place(p20, tr(mx, y_teeth[1], mz) @ rz(180)), "alu", "alu"))
    return items


def _rim20(phase):
    rim = cq.Workplane("XY").newObject([place(gt2.pulley_rim(20, 7.0, phase), tr(0, 7.0, 0) @ rx(90))])
    p = rim.union(ycyl(16, -1, 0)).union(ycyl(16, 7, 8)).union(ycyl(13, 8, 15))
    return p.cut(ycyl(5, -2, 16))


# ------------------------------------------------------------------ bins, tray, chute
def bin_box(outer=46.0, h=45.0, wall=1.6, colour="bin"):
    """Tapered bin, top at z=0, 2 deg draft, rounded corners, rim lip, label recess on -y face."""
    top = (cq.Workplane("XY", origin=(0, 0, -h)).sketch().rect(outer - 2 * h * math.tan(math.radians(2)),
                                                                 outer - 2 * h * math.tan(math.radians(2)))
           .vertices().fillet(4).finalize().extrude(h, taper=-2))
    inner = (cq.Workplane("XY", origin=(0, 0, -h + wall)).sketch()
             .rect(outer - 2 * wall - 2 * h * math.tan(math.radians(2)), outer - 2 * wall - 2 * h * math.tan(math.radians(2)))
             .vertices().fillet(3).finalize().extrude(h, taper=-2))
    b = top.cut(inner)
    lip = (cq.Workplane("XY", origin=(0, 0, -1.5)).sketch().rect(outer + 2, outer + 2).vertices().fillet(5)
           .finalize().extrude(1.5))
    lip = lip.cut(cq.Workplane("XY", origin=(0, 0, -2)).sketch().rect(outer - 2 * wall, outer - 2 * wall).vertices()
                  .fillet(3).finalize().extrude(3))
    b = b.union(lip)
    return b


def bins():
    items = []
    s = 1000 * CELL["bin_size"]
    Y = 1000 * CELL["bins_y"]
    tray_z = BZ - 45.0
    for k, bx in enumerate(CELL["bins_x"]):
        X = 1000 * bx
        col = "bin_grey" if k == len(CELL["bins_x"]) - 1 else "bin"
        items.append((f"B01_bin_{k + 1}", place(_c("bin", lambda: bin_box(s)), tr(X, Y, BZ)), col, "pp"))
    tray = rbox(-112, 112, Y - 31, Y + 31, tray_z - 3, tray_z, 3.0, "|Z")
    for k, bx in enumerate(CELL["bins_x"]):                               # locating pockets 1 mm
        X = 1000 * bx
        tray = tray.cut(rbox(X - s / 2 + 1.0, X + s / 2 - 1.0, Y - s / 2 + 1.0, Y + s / 2 - 1.0, tray_z - 1, tray_z + 1, 3.0, "|Z"))
    items.append(("B02_bin_tray", tray.val(), "print2", "petg_cf"))
    yb = YC + PLATE_IN + PLATE_T
    for X in (-95, 95):
        a = box(X - 12, X + 12, yb, yb + 3, tray_z - 18, tray_z - 3).union(box(X - 12, X + 12, yb, Y + 28, tray_z - 6, tray_z - 3))
        g = (cq.Workplane("YZ", origin=(X - 1.5, 0, 0)).polyline([(yb + 3, tray_z - 6), (yb + 3, tray_z - 17), (yb + 28, tray_z - 6)])
             .close().extrude(3))
        items.append((f"B03_tray_bracket_{X:+d}", a.union(g).val(), "alu", "alu"))
    # overflow chute and tote
    x0 = X_DRIVE + ROLLER_D / 2 + 2
    chute_top, chute_end = (x0, BZ - 4), (x0 + 95, BZ - 42)
    pts = [(chute_top[0], chute_top[1]), (chute_end[0], chute_end[1]), (chute_end[0], chute_end[1] - 3),
           (chute_top[0], chute_top[1] - 3)]
    ch = cq.Workplane("XZ", origin=(0, YC + 78, 0)).polyline(pts).close().extrude(156)
    for s_ in (-1, 1):
        wall = (cq.Workplane("XZ", origin=(0, YC + s_ * 78 + (2 if s_ > 0 else 0), 0))
                .polyline([(chute_top[0], chute_top[1]), (chute_end[0], chute_end[1]), (chute_end[0], chute_end[1] + 30),
                           (chute_top[0], chute_top[1] + 30)]).close().extrude(2))
        ch = ch.union(wall)
    items.append(("B04_overflow_chute", ch.val(), "print2", "petg_cf"))
    tx0 = chute_end[0] - 20
    tote = rbox(tx0, tx0 + 110, YC - 85, YC + 85, TABLE, TABLE + 55, 4.0, "|Z")
    tote = tote.cut(rbox(tx0 + 2, tx0 + 108, YC - 83, YC + 83, TABLE + 2, TABLE + 56, 3.0, "|Z"))
    items.append(("B05_overflow_tote", tote.val(), "bin_red", "pp"))
    return items


# ------------------------------------------------------------------ camera head, LEDs, beam sensor
def camera_head():
    items = []
    plate_z = (-119.0, -115.0)
    pl = rbox(CAM_X - 55, CAM_X + 55, YC - 75, YC + 75, *plate_z, 4.0, "|Z")
    for dy in (-38, 38):
        pl = pl.cut(rbox(CAM_X - 40, CAM_X + 40, YC + dy - 17, YC + dy + 17, plate_z[0] - 1, plate_z[1] + 1, 4.0, "|Z"))
    pl = pl.cut(zcyl(16, plate_z[0] - 1, plate_z[1] + 1, CAM_X, YC))
    for dx in (-15.5, 15.5):
        for dy in (-15.5, 15.5):
            pl = pl.cut(zcyl(2.4, plate_z[0] - 1, plate_z[1] + 1, CAM_X + dx, YC + dy))
    pl = pl.cut(zcyl(5.5, plate_z[0] - 1, plate_z[1] + 1, CAM_X, YC - 55)).cut(zcyl(5.5, plate_z[0] - 1, plate_z[1] + 1, CAM_X, YC + 55))
    items.append(("M22_camera_head", pl.val(), "print", "petg_cf"))
    for suffix, wp, col, mat in Vd.ov9281_camera():
        items.append((f"E15_camera_{suffix}", place(wp, tr(CAM_X, YC, CAM_Z)), col, mat))
    for dx in (-15.5, 15.5):
        for dy in (-15.5, 15.5):
            items.append((f"M2_standoff_{dx:+.0f}{dy:+.0f}", zcyl(4, CAM_Z + 22.1, plate_z[0], CAM_X + dx, YC + dy).val(),
                          "brass", "brass"))
    for dy in (-62, 62):
        for suffix, wp, col, mat in Vd.led_bar(100):
            items.append((f"E16_led_bar_{dy:+d}_{suffix}", place(wp, tr(CAM_X, YC + dy, plate_z[0] - 8)), col, mat))
    for y in (YC - 55, YC + 55):
        for suffix, wp, col, mat in Vd.shcs(5, 10):
            items.append((f"screw_M5x10_camera_{y:+.0f}", place(wp, tr(CAM_X, y, plate_z[0]) @ rx(180)), col, mat))
    return items


def beam_sensor():
    items = []
    yf, yb = YC - PLATE_IN - PLATE_T, YC + PLATE_IN + PLATE_T
    for y, M, nm in ((yf, rx(90), "emitter"), (yb, rx(-90), "receiver")):
        for suffix, wp, col, mat in _c("m18", lambda: Vd.m18_sensor(60)):
            items.append((f"E14_beam_{nm}_{suffix}", place(wp, tr(BEAM_X, y, BZ + 5) @ M), col, mat))
        s = -1 if nm == "emitter" else 1
        y0, y1 = sorted((y + s * 22, y + s * 27))
        brk = rbox(BEAM_X - 20, BEAM_X + 20, y0, y1, BZ + 5 - 20, BZ + 5 + 15, 3.0, "|Y")
        brk = brk.cut(ycyl(18.5, y0 - 1, y1 + 1, BEAM_X, BZ + 5))
        legs = box(BEAM_X - 20, BEAM_X - 14, min(y, y + s * 22), max(y, y + s * 22), BZ - 15, BZ + 20) \
            .union(box(BEAM_X + 14, BEAM_X + 20, min(y, y + s * 22), max(y, y + s * 22), BZ - 15, BZ + 20))
        items.append((f"M23_beam_bracket_{nm}", brk.union(legs).val(), "print2", "petg_cf"))
    return items


# ------------------------------------------------------------------ electronics, e-stop, caps
def electronics():
    items = []
    x0, y0, z0 = -420.0, 190.0, TABLE
    base = rbox(x0, x0 + 400, y0, y0 + 100, z0 + 8, z0 + 11, 4.0, "|Z")
    for X in (x0 + 15, x0 + 385):
        for Y in (y0 + 15, y0 + 85):
            base = base.cut(zcyl(4.5, z0 + 7, z0 + 12, X, Y))
            items.append((f"E00_tray_foot_{X:+.0f}{Y:+.0f}", zcyl(12, z0, z0 + 8, X, Y).val(), "rubber", "rubber"))
    items.append(("M25_electronics_tray", base.val(), "alu", "alu"))
    zt = z0 + 11
    groups = [("E01_PSU_LRS150_24", Vd.psu_lrs150(), tr(x0 + 85, y0 + 50, zt)),
              ("E02_BlackPill", Vd.blackpill(), tr(x0 + 215, y0 + 30, zt + 14)),
              ("E10_buck_12V", Vd.buck_converter(), tr(x0 + 365, y0 + 78, zt + 6)),
              ("E11_vacuum_pump", Vd.vacuum_pump(), tr(x0 + 300, y0 + 30, zt)),
              ("E12_valve", Vd.solenoid_valve(), tr(x0 + 365, y0 + 30, zt))]
    carrier = box(x0 + 175, x0 + 275, y0 + 12, y0 + 88, zt + 5, zt + 6.6)
    items.append(("E03_driver_carrier", carrier.val(), "pcb_green", "pcb"))
    for k in range(4):
        groups.append((f"E03_TMC2209_{k + 1}", Vd.tmc2209_module(), tr(x0 + 190 + 22 * k, y0 + 66, zt + 15)))
    for name, parts, M in groups:
        for suffix, wp, col, mat in parts:
            items.append((f"{name}_{suffix}", place(wp, M), col, mat))
    for X in (x0 + 180, x0 + 270):
        for Y in (y0 + 17, y0 + 83):
            items.append((f"standoff_carrier_{X:+.0f}{Y:+.0f}", zcyl(5, zt, zt + 5, X, Y).val(), "brass", "brass"))
    return items


def estop():
    out = []
    for suffix, wp, col, mat in Vd.estop():
        out.append((f"E13_estop_{suffix}", place(wp, tr(FRAME_X[1] + 10, FRAME_Y[0], -60) @ ry(90)), col, mat))
    return out


def sample_caps():
    def cap():
        c = zcyl(30, 0, 12).cut(zcyl(27.6, -1, 10.8))
        for k in range(48):
            a = 360.0 * k / 48
            c = c.cut(box(14.6, 16, -0.5, 0.5, 1.0, 11.0).rotate((0, 0, 0), (0, 0, 1), a))
        return safe_chamfer(c, ">Z", 0.6)
    cp = _c("cap", cap)
    colours = ["red", "pcb_blue", "white", "pcb_green", "yellow", "red", "white", "pcb_blue", "pcb_green"]
    pos = [(-360, -86.5), (-360, -38.5), (-360, 9.5), (-250, -38.5), (-230, 9.5), (-140, -86.5), (-40, -38.5),
           (-20, 9.5), (30, -86.5)]
    return [(f"S01_cap_{k + 1}", place(cp, tr(x, y, BZ)), colours[k], "pp") for k, (x, y) in enumerate(pos)]


def build_cell():
    items = []
    for fn in (frame, conveyor, bins, camera_head, beam_sensor, electronics, estop, sample_caps):
        items += [(n, s, c, m, None) for n, s, c, m in fn()]
    return items
