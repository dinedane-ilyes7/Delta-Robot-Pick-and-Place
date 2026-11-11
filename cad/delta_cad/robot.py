"""Robot assembly at a pose, organised as rigid bodies with explicit joints (for MuJoCo).

Bodies (reference pose = the pose the assembly is built at):
  base          static: base plate, 3 brackets, motors, belts, bearings' outer rings, encoders, stop pins
  arm1..3       hinge about the arm axis (+y of the arm frame), angle theta absolute (ref = theta at the pose)
  rod{i}{a,b}   ball joint at the elbow ball centre; rod frame z' along the rod
  platform      free body, closed by 6 connect constraints at the platform ball centres
"""
import math
import os
import sys

import numpy as np

from . import gt2, parts as P, vendor as Vd
from .dims import *  # noqa: F401,F403
from .lib import MIRROR_Y, frame_from_z, place, rx, ry, rz, tr

sys.path.insert(0, os.path.join(ROOT, "analysis"))
from kinematics import Delta  # noqa: E402


class Body:
    def __init__(self, name, parent, M, joint=None):
        self.name, self.parent, self.M, self.joint = name, parent, np.asarray(M, float), joint
        self.parts = []          # (name, shape in body frame, colour, material, mass_override_g)
        self.sites = {}          # name -> position in body frame (mm)

    def add(self, name, wp, M_local, colour, material, mass=None):
        self.parts.append((name, place(wp, M_local), colour, material, mass))

    def add_vendor(self, name, items, M_local, mass=None):
        for k, (suffix, wp, colour, material) in enumerate(items):
            self.add(name + ("_" + suffix if suffix else ""), wp, M_local, colour, material,
                     mass if (mass is not None and k == 0) else (0.0 if mass is not None else None))

    def world_parts(self):
        for name, shp, colour, material, mass in self.parts:
            yield f"{self.name}/{name}", place(shp, self.M), colour, material, mass


# catalogue masses (g) for purchased parts whose geometry is simplified
MASS_CATALOGUE = {"motor": 400.0, "bearing": 12.0, "ball_stud": 2.7}


def solve_pose(x_mm):
    d = Delta(G["R"], G["r"], G["L1"], G["L2"])
    th, ok = d.ik(np.asarray(x_mm, float) / 1000.0)
    if not ok:
        raise ValueError(f"pose {x_mm} is not reachable")
    return np.degrees(th)


def arm_frame(phi):
    return rz(phi) @ tr(R, 0, 0)


def rod_frame(E, Pp, t):
    zq = (Pp - E) / np.linalg.norm(Pp - E)
    yq = t - (t @ zq) * zq
    yq /= np.linalg.norm(yq)
    xq = np.cross(yq, zq)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = xq, yq, zq, E
    return M


_CACHE = {}


def _c(key, fn):
    if key not in _CACHE:
        _CACHE[key] = fn()
    return _CACHE[key]


def build(x_mm=(0.0, 0.0, -240.0), with_belts=True):
    th = solve_pose(x_mm)
    x = np.asarray(x_mm, float)
    bodies = {}
    base = Body("base", None, np.eye(4))
    bodies["base"] = base

    # ---------------- base plate and its screws
    base.add("M01_base_plate", _c("m01", P.m01_base_plate), np.eye(4), "alu", "alu")
    m4 = _c("shcs4_14", lambda: Vd.shcs(4, 14))
    ins4 = _c("ins4", lambda: Vd.insert(4))
    m3_8 = _c("shcs3_8", lambda: Vd.shcs(3, 8))
    m25 = _c("shcs25_6", lambda: Vd.shcs(2.5, 6))
    ins25 = _c("ins25", lambda: Vd.insert(2.5))
    ins3 = _c("ins3", lambda: Vd.insert(3))
    brg = _c("608", Vd.bearing_608)
    stud = _c("stud", Vd.ball_stud_8)

    rod_end = _c("m07", P.m07_rod_end)
    rod_end_m = _c("m07m", lambda: P.m07_rod_end().mirror("XZ"))
    rod_tube = _c("m08", P.m08_rod_tube)
    dowel3 = _c("dowel3", lambda: Vd.dowel(3.0, 12.0))

    for i, phi in enumerate(PHI):
        A = arm_frame(phi)
        n = i + 1
        # ---------------- static parts of this arm (in base)
        base.add(f"M02_bracket_{n}", _c("m02", P.m02_pivot_bracket), A, "print", "petg_cf")
        for k, (hx, hy) in enumerate(FLANGE_HOLES):
            base.add_vendor(f"insert_M4_{n}{k}", ins4, A @ tr(hx, hy, FLANGE_Z[1]))
            base.add_vendor(f"screw_M4x14_{n}{k}", m4, A @ tr(hx, hy, Z_PLATE + BASE_PLATE["t"]))
        Mm = A @ tr(0, MOTOR_FACE_Y, Z_MOTOR) @ rx(-90)
        base.add_vendor(f"P01_motor_{n}", _c("nema", Vd.nema17_17hs19), Mm, mass=MASS_CATALOGUE["motor"])
        for k, (dx, dz) in enumerate(((-15.5, -15.5), (-15.5, 15.5), (15.5, -15.5), (15.5, 15.5))):
            base.add_vendor(f"screw_M3x8_motor_{n}{k}", m3_8, A @ tr(dx, MPLATE_Y[0], Z_MOTOR + dz) @ rx(90))
        for s in (-1, 1):
            Mb = A @ tr(0, s * BRG_Y, 0) @ rx(-90)
            base.add(f"P02_bearing_outer_{n}{'ab'[s > 0]}", brg[0][1], Mb, "chrome", "steel")
            base.add(f"P02_bearing_seal_{n}{'ab'[s > 0]}", brg[2][1], Mb, "black", "rubber")
        for k, (sx, sz) in enumerate(P.stop_pin_positions()):
            base.add_vendor(f"P05_stop_pin_{n}{k}", _c("pin6", lambda: Vd.dowel(6.0, 2 * CHEEK[1])),
                            A @ tr(sx, -CHEEK[1], sz))
        base.add_vendor(f"E04_AS5600_{n}", _c("as5600", Vd.as5600_board), A @ tr(0, CHIP_FACE_Y, 0))
        for k, (px, pz) in enumerate(POST_XZ):
            base.add_vendor(f"insert_M2.5_enc_{n}{k}", ins25, A @ frame_from_z((px, BOARD_Y, pz), (0, -1, 0)))
            base.add_vendor(f"screw_M2.5x6_enc_{n}{k}", m25, A @ tr(px, BOARD_Y - 1.6, pz) @ rx(90))
        if with_belts:
            path = _c("beltpath", lambda: gt2.BeltPath((0, 0), N_BIG, (0, Z_MOTOR), N_SMALL))
            belt, phase2 = gt2.belt_solid(path, BELT_W, -th[i])
            base.add(f"P07_belt_{n}", belt, A @ tr(0, BELT_Y[1], 0) @ rx(90), "belt", "neoprene")
        else:
            phase2 = 0.0
        base.add_vendor(f"P06_pulley_20T_{n}", Vd.gt2_pulley_20t_10mm(phase2), A @ tr(0, TEETH_Y[0], Z_MOTOR))

        # ---------------- arm body
        Aq = A @ ry(th[i])
        arm = Body(f"arm{n}", "base", Aq, joint=dict(type="hinge", axis=(0, 1, 0), ref_deg=float(th[i]),
                                                     range_deg=(TH_MIN, TH_MAX)))
        bodies[arm.name] = arm
        I4 = np.eye(4)
        arm.add("M04_hub", _c("m04", P.m04_arm_hub), I4, "print", "petg_cf")
        arm.add("M06_upper_tube", _c("m06", P.m06_upper_tube), I4, "carbon", "carbon")
        arm.add("M05_elbow", _c("m05", P.m05_elbow_block), I4, "print", "petg_cf")
        arm.add("M11_pulley_120T", _c("m11", P.m11_pulley_120t), I4, "print", "petg_cf")
        arm.add("M10_magnet_holder", _c("m10", P.m10_magnet_holder), I4, "print", "pla")
        arm.add_vendor("P11_magnet", _c("mag", Vd.magnet_6x2_5), tr(0, MAGNET_Y[0], 0))
        arm.add_vendor("P03_shaft", _c("shaft", lambda: Vd.shaft8(
            SHAFT_Y[1] - SHAFT_Y[0], tuple((a - SHAFT_Y[0], b - SHAFT_Y[0]) for a, b in SHAFT_FLATS))),
            tr(0, SHAFT_Y[0], 0))
        for nm, (y0, y1) in (("in_a", (-SPACER_IN[1], -SPACER_IN[0])), ("in_b", SPACER_IN),
                             ("out_a", SPACER_NEG), ("out_b", SPACER_POS)):
            arm.add_vendor(f"P04_spacer_{nm}", _c(f"sl{y1 - y0:.1f}", lambda L=y1 - y0: Vd.sleeve(12.0, 8.2, L)),
                           tr(0, y0, 0))
        arm.add_vendor("P13_collar", _c("collar", Vd.shaft_collar_8), tr(0, COLLAR_Y[0], 0))
        for s in (-1, 1):
            arm.add(f"P02_bearing_inner_{'ab'[s > 0]}", brg[1][1], tr(0, s * BRG_Y, 0) @ rx(-90), "chrome", "steel")
        arm.add_vendor("screw_M3x10_clamp", _c("shcs3_10", lambda: Vd.shcs(3, 10)), tr(-10, 0, TAIL_H - 3.2))
        arm.add_vendor("nut_M3_clamp", _c("nut3", lambda: Vd.hex_nut(3)), tr(-10, 0, -TAIL_H + 0.2) @ rz(30))
        for k, d in enumerate(((0, 0, 1), (1, 0, 0))):
            d = np.array(d, float)
            arm.add_vendor(f"insert_M3_pulley_{k}", ins3, frame_from_z(np.array([0, 30.5, 0]) + 11.0 * d, d))
            arm.add_vendor(f"setscrew_M3x5_pulley_{k}", _c("ss3", lambda: Vd.set_screw(3, 5)),
                           frame_from_z(np.array([0, 30.5, 0]) + (3.5 if k == 0 else 4.0) * d, d))
        for s in (-1, 1):
            ab = "ab"[s > 0]
            arm.add_vendor(f"insert_M3_elbow_{ab}", ins3,
                           frame_from_z((L1, s * ELBOW_BLOCK["y"], 0), (0, s, 0)))
            arm.add_vendor(f"P08_ball_stud_elbow_{ab}", stud, frame_from_z((L1, s * W / 2, 0), (0, s, 0)),
                           mass=MASS_CATALOGUE["ball_stud"])
            arm.sites[f"elbow_{ab}"] = np.array([L1, s * W / 2, 0.0])

        # ---------------- rods
        a = math.radians(phi)
        u, t = np.array([math.cos(a), math.sin(a), 0]), np.array([-math.sin(a), math.cos(a), 0])
        tq = math.radians(th[i])
        E0 = (R + L1 * math.cos(tq)) * u + np.array([0, 0, -L1 * math.sin(tq)])
        Mbot = tr(0, 0, L2) @ ry(180)
        for s in (-1, 1):
            ab = "ab"[s > 0]
            E = E0 + s * W / 2 * t
            Pp = x + r * u + s * W / 2 * t
            Fq = rod_frame(E, Pp, t)
            rod = Body(f"rod{n}{ab}", arm.name, Fq, joint=dict(type="ball"))
            bodies[rod.name] = rod
            end = rod_end if s > 0 else rod_end_m
            rod.add("M07_rod_end_top", end, np.eye(4), "print", "petg_cf")
            rod.add("M08_rod_tube", rod_tube, np.eye(4), "carbon", "carbon")
            rod.add("M07_rod_end_bottom", end, Mbot, "print", "petg_cf")
            xs, zs = SPRING_POST
            rod.add_vendor("P10_spring_pin_top", dowel3, tr(xs, 0, ROD_SOCKET[0] + 0.5) @ rx(90))
            rod.add_vendor("P10_spring_pin_bottom", dowel3, Mbot @ tr(xs, 0, ROD_SOCKET[0] + 0.5) @ rx(90))
            rod.sites["top"] = np.zeros(3)
            rod.sites["bottom"] = np.array([0, 0, L2])
            rod.sites["spring_top"] = np.array([xs, 0, zs])
            rod.sites["spring_bottom"] = (Mbot @ np.array([xs, 0, zs, 1]))[:3]

    # ---------------- platform
    plat = Body("platform", "base", tr(*x), joint=dict(type="free"))
    bodies["platform"] = plat
    plat.add("M09_platform", _c("m09", P.m09_platform), np.eye(4), "print", "petg_cf")
    for i, phi in enumerate(PHI):
        a = math.radians(phi)
        u, t = np.array([math.cos(a), math.sin(a), 0]), np.array([-math.sin(a), math.cos(a), 0])
        for s in (-1, 1):
            ab = "ab"[s > 0]
            plat.add_vendor(f"insert_M3_{i + 1}{ab}", ins3, frame_from_z(r * u + s * PLAT_LUG["t"] * t, s * t))
            plat.add_vendor(f"P08_ball_stud_{i + 1}{ab}", stud, frame_from_z(r * u + s * W / 2 * t, s * t),
                            mass=MASS_CATALOGUE["ball_stud"])
            plat.sites[f"ball_{i + 1}{ab}"] = r * u + s * W / 2 * t
    plat.add_vendor("insert_M5_hub", _c("ins5", lambda: Vd.insert(5)), tr(0, 0, PLAT_HUB["z"][1]))
    plat.add_vendor("P12_cup", _c("cup", P.p12_cup), np.eye(4))
    plat.add_vendor("P14_pushin_M5", _c("fit", Vd.m5_pushin_straight), tr(0, 0, PLAT_HUB["z"][1]))
    plat.add_vendor("E05_ADXL345", _c("adxl", Vd.adxl345_gy291), tr(18.0, -5.3, 7.0))
    for k, uu in enumerate((10.4, 25.6)):
        plat.add_vendor(f"insert_M2.5_adxl_{k}", ins25, tr(uu, 0, 7.0))
        plat.add_vendor(f"screw_M2.5x6_adxl_{k}", m25, tr(uu, 0, 8.6))
    plat.sites["tcp"] = np.array([0, 0, -TCP])
    plat.sites["imu"] = np.array([18.0 - 1.5 + 1.5, -5.3 - 1.0, 7.0 + 2.1])

    return bodies, th


def springs(bodies):
    """World geometry of the 6 extension springs: (name, p0, p1, pin_direction)."""
    out = []
    for i in range(3):
        n = i + 1
        ra, rb = bodies[f"rod{n}a"], bodies[f"rod{n}b"]
        for end in ("top", "bottom"):
            p0 = (ra.M @ np.append(ra.sites[f"spring_{end}"], 1))[:3]
            p1 = (rb.M @ np.append(rb.sites[f"spring_{end}"], 1))[:3]
            out.append((f"P09_spring_{n}_{end}", p0, p1, ra.M[:3, 2]))
    return out


def spring_shapes(bodies):
    items = []
    for name, p0, p1, pin in springs(bodies):
        L = float(np.linalg.norm(p1 - p0))
        ax = (p1 - p0) / L
        Ms = frame_from_z(p0, ax, xhint=np.cross(pin, ax))
        sp = _c(f"spring{L:.2f}", lambda L=L: Vd.extension_spring(L))
        for suffix, wp, colour, material in sp:
            items.append((name, place(wp, Ms), colour, material, None))
    return items


def vacuum_tube(bodies):
    """Visual 4 mm PU tube from the push-in fitting straight up to the base plate centre hole."""
    from .lib import cyl
    p0 = (bodies["platform"].M @ np.array([0, 0, PLAT_HUB["z"][1] + 14.0, 1]))[:3]
    p1 = np.array([0, 0, Z_PLATE + BASE_PLATE["t"] + 30.0])
    return [("P15_vacuum_tube", cyl(4.0, p0, p1).cut(cyl(2.5, p0, p1)).val(), "white", "pvc", None)]


def all_world_parts(bodies, extras=True):
    items = []
    for b in bodies.values():
        items += list(b.world_parts())
    if extras:
        items += spring_shapes(bodies)
        items += vacuum_tube(bodies)
    return items
