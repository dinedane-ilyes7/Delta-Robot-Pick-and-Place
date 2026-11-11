"""Designed (made) parts of the robot. Arm parts are in the arm-local frame (see dims.py),
rod ends in the rod frame (ball centre at the origin, rod along +z', cup on the +y' side),
the platform in its own frame (centre of the ball plane, z up, arm 1 along +x)."""
import math

import cadquery as cq
import numpy as np

from . import gt2
from .dims import *  # noqa: F401,F403
from .lib import (bore_with_leadin, box, cone, cyl, hex_prism, place, rbox, rx, rz, safe_chamfer, safe_fillet,
                  sphere, tr, xcyl, ycyl, zcyl)
from .vendor import INSERT


def stop_pin_positions():
    """(x, z) of the two hard-stop pins: the tail face touches the pin at TH_MIN and TH_MAX."""
    out = []
    for th, side in ((TH_MIN, -1), (TH_MAX, +1)):
        t = math.radians(th)
        a = np.array([math.cos(t), -math.sin(t)])
        n = np.array([math.sin(t), math.cos(t)])
        out.append(-STOP_R * a + side * (TAIL_H + STOP_PIN_D / 2) * n)
    return out


def _insert_hole(d, p_top, direction, extra=0.3, thread_extra=0.0):
    """Hole for a heat-set insert from p_top along direction, plus an optional thread-clearance hole beyond."""
    D, L, hole = INSERT[d]
    p_top = np.asarray(p_top, float)
    u = np.asarray(direction, float) / np.linalg.norm(direction)
    t = bore_with_leadin(hole, p_top, p_top + (L + extra) * u, 0.3)
    if thread_extra > 0:
        t = t.union(cyl(d + 0.2, p_top, p_top + (L + extra + thread_extra) * u))
    return t


# ================================================================== M01 base plate
def m01_base_plate():
    s, t, rc = BASE_PLATE["size"], BASE_PLATE["t"], BASE_PLATE["corner"]
    p = (cq.Workplane("XY", origin=(0, 0, Z_PLATE)).sketch().rect(s, s).vertices().fillet(rc).finalize()
         .extrude(t))
    p = p.cut(zcyl(BASE_PLATE["centre_hole"], Z_PLATE - 1, Z_PLATE + t + 1))
    for phi in PHI:
        M = rz(phi) @ tr(R, 0, 0)
        for hx, hy in FLANGE_HOLES:                                   # M4 clearance, screws from the top
            q = M @ np.array([hx, hy, 0, 1])
            p = p.cut(zcyl(4.5, Z_PLATE - 1, Z_PLATE + t + 1, q[0], q[1]))
        q = M @ np.array([-38.0, 78.0, 0, 1])                         # cable slot for motor + encoder
        slot = (cq.Workplane("XY", origin=(q[0], q[1], Z_PLATE - 1)).slot2D(26, 9, phi + 90).extrude(t + 2))
        p = p.cut(slot)
    for x in (-150.0, -50.0, 50.0, 150.0):                            # M5 to the 2020 rails
        for y in (-FRAME_RAIL_Y, FRAME_RAIL_Y):
            p = p.cut(zcyl(5.5, Z_PLATE - 1, Z_PLATE + t + 1, x, y))
    p = safe_chamfer(p, ">Z or <Z", 0.5)
    return p


# ================================================================== M02 pivot bracket (motor plate merged)
def _cheek_profile(y0, y1):
    """Cheek outline in x-z: tangent hull of a circle R=CHEEK_RC around the axis and the flange underside."""
    rc, xt, zt = CHEEK_RC, CHEEK_TOP_X, FLANGE_Z[0] + 0.01

    def tangent(px, pz, side):
        d = math.hypot(px, pz)
        a = math.atan2(pz, px)
        b = math.acos(rc / d)
        ang = a - b if side > 0 else a + b
        return rc * math.cos(ang), rc * math.sin(ang)
    zk = 60.0                                                         # straight sides from the flange down to zk
    tR, tL = tangent(xt, zk, +1), tangent(-xt, zk, -1)
    wp = (cq.Workplane("XZ", origin=(0, y1, 0)).moveTo(-xt, zt).lineTo(xt, zt).lineTo(xt, zk)
          .lineTo(*tR).threePointArc((0, -rc), tL).lineTo(-xt, zk).close())
    return wp.extrude(y1 - y0)


def m02_pivot_bracket():
    c0, c1 = CHEEK
    b = _cheek_profile(c0, c1).union(_cheek_profile(-c1, -c0))
    # flange
    fl = rbox(FLANGE_X[0], FLANGE_X[1], FLANGE_Y[0], FLANGE_Y[1], FLANGE_Z[0], FLANGE_Z[1], 4.0, "|Z")
    b = b.union(fl)
    # back wall between the cheeks, trimmed to the cheek outline
    wall = box(BACKWALL_X[0], BACKWALL_X[1], -c1, c1, 25.0, FLANGE_Z[0] + 0.1).intersect(_cheek_profile(-c1, c1))
    b = b.union(wall)
    # integrated motor plate
    mp = box(FLANGE_X[0], FLANGE_X[1], MPLATE_Y[0], MPLATE_Y[1], Z_MOTOR - 27.0, FLANGE_Z[0] + 0.1)
    mp = safe_fillet(mp, "|Y and <Z", 4.0)
    b = b.union(mp)
    # gussets
    for sx in (-1, 1):
        x0, x1 = sorted((sx * GUSSET_X, sx * (GUSSET_X + 4.0)))
        g = (cq.Workplane("YZ", origin=(x0, 0, 0)).polyline([(c1 - 0.1, FLANGE_Z[0] + 0.1), (MPLATE_Y[0] + 0.1, FLANGE_Z[0] + 0.1),
                                                           (MPLATE_Y[0] + 0.1, 50.0)]).close().extrude(x1 - x0))
        b = b.union(g)
    # bosses under the flange for the rear inserts
    for hx, hy in FLANGE_HOLES:
        if hy > 0:
            b = b.union(zcyl(11, FLANGE_Z[0] - 4, FLANGE_Z[0] + 0.1, hx, hy))
    # encoder posts and rib (-y side)
    for x, z in POST_XZ:
        b = b.union(ycyl(7.0, BOARD_Y, -c1 + 0.1, x, z))
    b = b.union(box(POST_XZ[0][0], POST_XZ[1][0], BOARD_Y + 2.0, -c1 + 0.1, POST_XZ[0][1] - 3.5, POST_XZ[0][1] - 1.0))

    # ---- cuts
    for s in (-1, 1):                                                 # bearing seats with lead-in + shoulder
        y_in, y_seat = s * c0, s * (BRG_Y + BRG_W / 2)
        b = b.cut(bore_with_leadin(22.0, (0, y_in, 0), (0, y_seat, 0), 0.6))
        b = b.cut(ycyl(SHOULDER_D, min(y_seat, s * (c1 + 1)), max(y_seat, s * (c1 + 1))))
    for x, z in stop_pin_positions():
        b = b.cut(ycyl(STOP_PIN_D, -c1 - 1, c1 + 1, x, z))
        for s in (-1, 1):
            b = b.cut(cone(STOP_PIN_D + 1.6, STOP_PIN_D - 0.01, (x, s * (c1 + 0.01), z), (x, s * (c1 - 0.8), z)))
    for hx, hy in FLANGE_HOLES:                                       # M4 heat-set inserts from the top face
        b = b.cut(_insert_hole(4, (hx, hy, FLANGE_Z[1]), (0, 0, -1), 0.5, 2.0))
    for x, z in POST_XZ:                                              # M2.5 inserts in the posts
        b = b.cut(_insert_hole(2.5, (x, BOARD_Y, z), (0, 1, 0), 0.5, 1.5))
    # motor plate: boss slot and 4 M3 slots, +-MOTOR_SLOT along z
    yb = MPLATE_Y[0] - 1
    lp = MPLATE_Y[1] - MPLATE_Y[0] + 2
    b = b.cut(cq.Workplane("XZ", origin=(0, MPLATE_Y[1] + 1, Z_MOTOR)).slot2D(22.6 + 2 * MOTOR_SLOT, 22.6, 90)
              .extrude(lp))
    for dx in (-15.5, 15.5):
        for dz in (-15.5, 15.5):
            b = b.cut(cq.Workplane("XZ", origin=(dx, MPLATE_Y[1] + 1, Z_MOTOR + dz))
                      .slot2D(3.4 + 2 * MOTOR_SLOT, 3.4, 90).extrude(lp))
    # clearance for the 120T pulley and belt behind the motor plate is guaranteed by layout (checked in robot.py)
    return b


# ================================================================== M04 arm hub
def m04_arm_hub():
    h = ycyl(24, -HUB_Y, HUB_Y)
    h = safe_chamfer(h, "%CIRCLE", 0.6)
    boss = xcyl(19, 7, HUB_SOCKET_X[1])
    boss = safe_chamfer(boss, ">X", 0.6)
    h = h.union(boss)
    tail = rbox(-TAIL_L, 0, -HUB_Y, HUB_Y, -TAIL_H, TAIL_H, 2.0, "|Y")
    h = h.union(tail)
    # D-bore (flat on +z) and clamp slit through the tail
    dbore = ycyl(8.05, -HUB_Y - 1, HUB_Y + 1).cut(box(-5, 5, -HUB_Y - 2, HUB_Y + 2, 3.55, 5))
    h = h.cut(dbore)
    for s in (-1, 1):
        h = h.cut(cone(10.0, 8.0, (0, s * (HUB_Y + 0.01), 0), (0, s * (HUB_Y - 1.0), 0)))
    h = h.cut(box(-TAIL_L - 1, -3.5, -HUB_Y - 1, HUB_Y + 1, -0.6, 0.6))
    # clamp screw M3 along z at x=-10: counterbore on +z, nut trap on -z
    h = h.cut(zcyl(3.4, -TAIL_H - 1, TAIL_H + 1, -10, 0))
    h = h.cut(zcyl(6.0, TAIL_H - 3.2, TAIL_H + 1, -10, 0))
    h = h.cut(hex_prism(5.6, (-10, 0, -TAIL_H - 1), (-10, 0, -TAIL_H + 2.6), 30))
    # tube socket: bore with lead-in, glue groove, vent
    h = h.cut(bore_with_leadin(12.1, (HUB_SOCKET_X[1], 0, 0), (HUB_SOCKET_X[0], 0, 0), 0.6))
    h = h.cut(xcyl(12.9, 24, 26))
    h = h.cut(zcyl(2.0, 0, 10, 17.0, 0))
    return h


# ================================================================== M05 elbow block
def m05_elbow_block():
    e = xcyl(16, ELBOW_SOCKET_X[0], ELBOW_BLOCK["x"][0] + 2)
    e = safe_chamfer(e, "<X", 0.6)
    x0, x1 = ELBOW_BLOCK["x"]
    yb, zb = ELBOW_BLOCK["y"], ELBOW_BLOCK["z"]
    blk = rbox(x0, x1, -LUG_CORE, LUG_CORE, -zb, zb, 2.0)
    e = e.union(blk)
    for sgn in (-1, 1):                                               # stud spot-face bosses, retract the block
        boss = ycyl(10.0, *sorted((sgn * (LUG_CORE - 0.5), sgn * yb)), L1, 0)
        e = e.union(safe_chamfer(boss, "%CIRCLE", 0.4))
    e = e.cut(bore_with_leadin(12.1, (ELBOW_SOCKET_X[0], 0, 0), (ELBOW_SOCKET_X[1], 0, 0), 0.6))
    e = e.cut(xcyl(12.9, 133, 135))
    e = e.cut(zcyl(2.0, 0, 9, 148.0, 0))
    for s in (-1, 1):                                                 # M3 inserts for the ball studs
        e = e.cut(_insert_hole(3, (L1, s * yb, 0), (0, -s, 0), 0.3, 2.0))
    for s in (-1, 1):                                                 # lightening pockets top and bottom
        e = e.cut(rbox(153.0, 163.0, -5.0, 5.0, (zb - 2.5) if s > 0 else -zb - 1, (zb + 1) if s > 0 else -zb + 2.5,
                       1.5, "|Z"))
    return e


# ================================================================== M06 / M08 tubes
def m06_upper_tube():
    t = xcyl(TUBE_UP[0], *TUBE_UP_X).cut(xcyl(TUBE_UP[1], TUBE_UP_X[0] - 1, TUBE_UP_X[1] + 1))
    return safe_chamfer(t, "%CIRCLE", 0.3)


def m08_rod_tube():
    t = zcyl(TUBE_LO[0], *ROD_TUBE_Z).cut(zcyl(TUBE_LO[1], ROD_TUBE_Z[0] - 1, ROD_TUBE_Z[1] + 1))
    return safe_chamfer(t, "%CIRCLE", 0.2)


# ================================================================== M07 rod end
def m07_rod_end():
    """Rod frame: ball centre at origin, rod along +z', cup covers the ball from the +y' side, opening -y'.
    Spring pin (3 x 12 dowel along z') in a clevis lug at SPRING_POST (x' = -8, z' = 15)."""
    zb, zm = ROD_SOCKET
    boss = zcyl(10, zb + 2.5, zm)
    boss = safe_chamfer(boss, ">Z", 0.5)
    boss = boss.union(cone(7.8, 10.0, (0, 0, zb - 0.5), (0, 0, zb + 2.5)))   # tapered root: more swing
    cup = sphere(14.0).intersect(box(-8, 8, CUP_RIM, 8, -8, 8))
    neck = rbox(-4.0, 4.0, CUP_RIM, 6.5, 0.0, zb + 0.5, 1.5, "|Z")
    xs, zs = SPRING_POST
    sg = 1.0 if xs > 0 else -1.0
    lug = rbox(*sorted((sg * 4.0, sg * 11.5)), -3.0, 3.0, zb + 0.5, zb + 12.0, 1.0)
    f = boss.union(cup).union(neck).union(lug)
    f = f.cut(sphere(BALL_D + 0.1))                                   # spherical seat
    f = f.cut(cone(BALL_D + 1.0, BALL_D - 2.0, (0, CUP_RIM - 0.01, 0), (0, CUP_RIM + 1.5, 0)))   # rim break
    f = f.cut(bore_with_leadin(TUBE_LO[0] + 0.1, (0, 0, zm), (0, 0, zb), 0.4))
    f = f.cut(zcyl(TUBE_LO[0] + 0.7, 15.5, 17.0))                     # glue groove
    f = f.cut(zcyl(1.5, 3.0, zb + 0.1))                               # vent into the cup
    f = f.cut(box(*sorted((xs - sg * 2.5, sg * 12.5)), -4, 4, zs - 1.5, zs + 1.5))   # hook slot
    f = f.cut(zcyl(SPRING_PIN_D - 0.05, zb + 0.4, zb + 12.1, xs, 0))  # dowel (press fit)
    return f


# ================================================================== M09 platform
def m09_platform():
    hub = zcyl(PLAT_HUB["d"], *PLAT_HUB["z"])
    hub = safe_chamfer(hub, "%CIRCLE", 0.6)
    p = hub
    u0, u1 = PLAT_LUG["u"]
    tl, zl = PLAT_LUG["t"], PLAT_LUG["z"]
    for phi in PHI:
        lug = rbox(u0, u1, -LUG_CORE, LUG_CORE, -zl, zl, 2.0)
        for sgn in (-1, 1):
            lug = lug.union(safe_chamfer(cyl(10.0, (r, sgn * (LUG_CORE - 0.5), 0), (r, sgn * tl, 0)), "%CIRCLE", 0.4))
        spoke = rbox(PLAT_HUB["d"] / 2 - 2, u0 + 1, -4.0, 4.0, -3.0, 3.0, 1.2, "|X")
        q = lug.union(spoke)
        q = q.cut(_insert_hole(3, (r, tl, 0), (0, -1, 0), 0.3, 2.0))
        q = q.cut(_insert_hole(3, (r, -tl, 0), (0, 1, 0), 0.3, 2.0))
        p = p.union(q.rotate((0, 0, 0), (0, 0, 1), phi))
    for k, phi in enumerate(PHI):                                     # perimeter bars between lugs
        a0, a1 = math.radians(phi), math.radians(PHI[(k + 1) % 3])
        P1 = np.array([24 * math.cos(a0) - 7 * math.sin(a0), 24 * math.sin(a0) + 7 * math.cos(a0), 0])
        P2 = np.array([24 * math.cos(a1) + 7 * math.sin(a1), 24 * math.sin(a1) - 7 * math.cos(a1), 0])
        L = np.linalg.norm(P2 - P1)
        bar = rbox(-L / 2 - 1, L / 2 + 1, -2.0, 2.0, -2.5, 2.5, 1.0, "|X")
        ang = math.degrees(math.atan2(*(P2 - P1)[1::-1]))
        p = p.union(bar.rotate((0, 0, 0), (0, 0, 1), ang).translate(tuple((P1 + P2) / 2)))
    # ADXL345 posts on spoke 1 and lug 1
    for u in (10.4, 25.6):
        p = p.union(zcyl(5.0, 2.0, 7.0, u, 0))
        p = p.cut(_insert_hole(2.5, (u, 0, 7.0), (0, 0, -1), 0.3, 1.0))
    # M5 insert through the hub (cup below, push-in fitting above)
    p = p.cut(bore_with_leadin(INSERT[5][2], (0, 0, PLAT_HUB["z"][1]), (0, 0, PLAT_HUB["z"][0] - 0.1), 0.4))
    return p


# ================================================================== M10 magnet holder
def m10_magnet_holder():
    y0, y1 = MAG_HOLDER_Y
    h = ycyl(13, y0, y1)
    h = safe_chamfer(h, "%CIRCLE", 0.5)
    h = h.cut(ycyl(8.05, SHAFT_Y[0], y1 + 1))                         # onto the shaft end (glue)
    h = h.cut(ycyl(6.1, y0 - 1, MAGNET_Y[1]))                         # magnet pocket from the far face
    return h


# ================================================================== M11 120T pulley
def m11_pulley_120t():
    rim = cq.Workplane("XY").newObject([place(gt2.pulley_rim(N_BIG, TEETH_Y[1] - TEETH_Y[0], 0.0),
                                              tr(0, TEETH_Y[1], 0) @ rx(90))])
    od = gt2.od(N_BIG)
    rim = rim.cut(ycyl(68.0, TEETH_Y[0] - 1, TEETH_Y[1] + 1))
    fl = ycyl(od + 5.0, 0, 1.0).cut(ycyl(68.0, -1, 2))
    fl = safe_chamfer(fl, "%CIRCLE", 0.3)
    p = rim.union(fl.translate((0, TEETH_Y[0] - 1.0, 0))).union(fl.translate((0, TEETH_Y[1], 0)))
    web = ycyl(69.0, 38.0, 41.0)
    for k in range(6):
        a = 2 * math.pi * (k + 0.5) / 6
        web = web.cut(ycyl(17.0, 37, 42, 23 * math.cos(a), 23 * math.sin(a)))
    p = p.union(web)
    hub = ycyl(22.0, *PULLEY_HUB_Y)
    hub = safe_chamfer(hub, "%CIRCLE", 0.6)
    p = p.union(hub)
    dbore = ycyl(8.05, PULLEY_HUB_Y[0] - 1, PULLEY_HUB_Y[1] + 1).cut(box(-5, 5, 25, 47, 3.55, 5))
    p = p.cut(dbore)
    for d in ((0, 0, 1), (1, 0, 0)):                                  # 2 x M3 inserts for set screws, at y = 30.5
        d = np.array(d, float)
        top = np.array([0, 30.5, 0]) + 11.0 * d
        p = p.cut(_insert_hole(3, top, -d, 0.3, 3.0))
    return p


# ================================================================== P12 cup assembly (holder + stem + bellows cup)
def p12_cup():
    """Platform frame. M5 thread enters the hub from below; the lip is at z = -TCP."""
    z_hub = PLAT_HUB["z"][0]
    thread = zcyl(4.8, z_hub, z_hub + 4.5)
    hx = hex_prism(8.0, (0, 0, z_hub - 4.0), (0, 0, z_hub))
    stem = zcyl(6.0, -TCP + 13.0, z_hub - 4.0)
    holder = thread.union(hx).union(stem).cut(zcyl(2.0, -TCP, z_hub + 5))
    lip = -TCP
    cup = cone(10.0, 9.0, (0, 0, lip), (0, 0, lip + 1.5))
    z = lip + 1.5
    for k in range(2):                                                # 1.5 bellows
        cup = cup.union(cone(9.0, 6.5, (0, 0, z), (0, 0, z + 2.0))).union(cone(6.5, 9.0, (0, 0, z + 2.0), (0, 0, z + 4.0)))
        z += 4.0
    cup = cup.union(cone(9.0, 7.0, (0, 0, z), (0, 0, z + 2.0))).union(zcyl(7.0, z + 2.0, -TCP + 15.0))
    cup = cup.cut(cone(8.6, 4.0, (0, 0, lip - 0.01), (0, 0, lip + 4.0))).cut(zcyl(4.0, lip + 3, -TCP + 13.0))
    cup = cup.cut(zcyl(6.0, -TCP + 13.0, -TCP + 15.1))
    return [("holder", holder, "brass", "brass"), ("cup", cup, "cup", "silicone")]
