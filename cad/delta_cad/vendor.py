"""Purchased parts with their catalogue dimensions. Each function returns a list of
(suffix, Workplane, colour, material) in the part's own frame, documented per function."""
import math

import cadquery as cq

from . import gt2
from .lib import place, rx, tr, box, cone, cyl, hex_prism, rbox, safe_chamfer, safe_fillet, sphere, xcyl, ycyl, zcyl


# ------------------------------------------------------------------ fasteners
SHCS = {  # ISO 4762 socket head cap screw: head dia, head height, hex key
    2.5: (4.5, 2.5, 2.0), 3: (5.5, 3.0, 2.5), 4: (7.0, 4.0, 3.0), 5: (8.5, 5.0, 4.0), 6: (10.0, 6.0, 5.0)}
INSERT = {  # heat-set insert: outer dia, length, hole to print
    2.5: (3.6, 4.0, 3.2), 3: (4.6, 5.7, 4.0), 4: (6.3, 8.1, 5.6), 5: (7.0, 9.5, 6.4)}
NUT = {3: (5.5, 2.4), 4: (7.0, 3.2), 5: (8.0, 4.0)}   # ISO 4032 across flats, thickness


def shcs(d, length):
    """Socket head cap screw. Head on +z from z=0 (bearing face), shank toward -z."""
    D, H, s = SHCS[d]
    head = zcyl(D, 0, H)
    head = safe_chamfer(head, ">Z", 0.3)
    head = head.cut(hex_prism(s, (0, 0, H - 0.6 * d), (0, 0, H + 0.1)))
    shank = zcyl(d * 0.92, -length, 0)
    shank = safe_chamfer(shank, "<Z", 0.25 * d / 3)
    return [("", head.union(shank), "oxide", "steel")]


def set_screw(d, length):
    """Cup-point set screw along +z from z=0, socket at z=length."""
    s = zcyl(d * 0.92, 0, length).cut(hex_prism(SHCS[d][2] * 0.6, (0, 0, length - 0.5 * d), (0, 0, length + 0.1)))
    return [("", s, "oxide", "steel")]


def insert(d):
    """Brass heat-set insert, axis z, top face at z=0, body toward -z."""
    D, L, _ = INSERT[d]
    body = zcyl(D, -L, 0).cut(zcyl(d, -L - 1, 1))
    for k in range(3):                                                # knurl rings
        z = -L * (0.25 + 0.25 * k)
        body = body.cut(zcyl(D + 1, z - 0.35, z + 0.35).cut(zcyl(D - 0.5, z - 1, z + 1)))
    return [("", body, "brass", "brass")]


def hex_nut(d):
    af, m = NUT[d]
    n = hex_prism(af, (0, 0, 0), (0, 0, m)).cut(zcyl(d, -1, m + 1))
    return [("", safe_chamfer(n, "not |Z", 0.3), "steel", "steel")]


def tnut_m5():
    """Drop-in T-nut for 2020 slot 6: body 10 x 6 x 4.5 below z=0."""
    t = box(-5, 5, -3, 3, -4.5, 0).union(box(-5, 5, -5.5, 5.5, -4.5, -2.5)).cut(zcyl(5, -5, 1))
    return [("", t, "steel", "steel")]


# ------------------------------------------------------------------ motion parts
def nema17_17hs19():
    """17HS19-2004S1 (2 A, 0.59 Nm, 48 mm). Mounting face at z=0, body toward +z, shaft toward -z.
    42.3 square, 4 x M3 x 4.5 deep on 31 mm, boss 22 x 2, shaft 5 x 24 with 0.5 mm D-flat over 20 mm.
    Flat faces +x."""
    s = 42.3
    def chamf(b):
        return safe_chamfer(b, "|Z", 4.2)
    cap_f = chamf(box(-s / 2, s / 2, -s / 2, s / 2, 0, 9.0))
    cap_r = chamf(box(-s / 2, s / 2, -s / 2, s / 2, 39.0, 48.0))
    stator = chamf(box(-s / 2 + 0.4, s / 2 - 0.4, -s / 2 + 0.4, s / 2 - 0.4, 9.0, 39.0))
    for z in (9.0, 39.0):                                             # thin dark seams between parts
        stator = stator.cut(chamf(box(-s / 2, s / 2, -s / 2, s / 2, z - 0.3, z + 0.3)).cut(
            chamf(box(-s / 2 + 1.2, s / 2 - 1.2, -s / 2 + 1.2, s / 2 - 1.2, z - 1, z + 1))))
    cap_f = safe_chamfer(cap_f, "<Z", 0.5)
    cap_r = safe_chamfer(cap_r, ">Z", 0.8)
    for x in (-15.5, 15.5):
        for y in (-15.5, 15.5):
            cap_f = cap_f.cut(zcyl(2.5, -0.1, 4.5, x, y))
            cap_r = cap_r.cut(zcyl(5.5, 47.0, 48.1, x, y)).cut(zcyl(3.0, 44.0, 48.1, x, y))   # through-bolt heads
    boss = zcyl(22, -2, 0)
    boss = safe_chamfer(boss, "<Z", 0.4)
    shaft = zcyl(5, -24, -2).cut(box(2.0, 3, -3, 3, -24.1, -4))
    shaft = safe_chamfer(shaft, "<Z", 0.4)
    conn = box(-8, 8, s / 2, s / 2 + 5.5, 39.5, 46.0).cut(box(-7, 7, s / 2 + 1.0, s / 2 + 5.6, 40.3, 45.2))
    return [("cap_front", cap_f, "alu_dark", "alu"), ("stator", stator, "black", "steel"),
            ("cap_rear", cap_r, "alu_dark", "alu"), ("boss", boss, "alu_dark", "alu"),
            ("shaft", shaft, "chrome", "steel"), ("connector", conn, "white", "abs")]


def bearing_608():
    """608-2RS deep-groove bearing 8 x 22 x 7, axis z, centred at z=0."""
    w = 7.0
    outer = zcyl(22, -w / 2, w / 2).cut(zcyl(19.0, -5, 5))
    inner = zcyl(12.1, -w / 2, w / 2).cut(zcyl(8.0, -5, 5))
    outer = safe_chamfer(outer, "%CIRCLE", 0.3)
    inner = safe_chamfer(inner, "%CIRCLE", 0.3)
    seal = zcyl(19.0, -w / 2 + 0.4, w / 2 - 0.4).cut(zcyl(12.1, -5, 5))
    return [("outer", outer, "chrome", "steel"), ("inner", inner, "chrome", "steel"),
            ("seals", seal, "black", "rubber")]


def kfl08():
    """KFL08 two-bolt flange bearing unit, 8 mm bore. Flange face z=0 (against the plate), hub toward +z.
    Flange 48 x 27 rhombus, bolts on 37 mm, thickness 5, hub 26 dia to z=11."""
    L, J, A, T = 48.0, 37.0, 27.0, 5.0
    fl = (cq.Workplane("XY").slot2D(L, A, 0).extrude(T))
    fl = fl.union(cq.Workplane("XY").rect(J * 0.6, A + 4).extrude(T).edges("|Z").fillet(6))
    hub = zcyl(26, 0, 11.0)
    unit = fl.union(hub)
    for x in (-J / 2, J / 2):
        unit = unit.cut(zcyl(5.2, -1, T + 1, x, 0))
    unit = unit.cut(zcyl(16, 1.0, 12))
    brg = zcyl(16, 1.0, 11.0).cut(zcyl(8, 0, 12))
    return [("housing", unit, "alu_dark", "zinc"), ("insert", brg, "chrome", "steel")]


def shaft8(length, flats=()):
    """8 mm h6 ground shaft along +y from y=0, 0.5 x 45 chamfers; D-flats 0.5 deep on +z over (y0, y1)."""
    s = ycyl(8, 0, length)
    s = safe_chamfer(s, "%CIRCLE", 0.5)
    for y0, y1 in flats:
        s = s.cut(box(-5, 5, y0, y1, 3.5, 5))
    return [("", s, "chrome", "steel")]


def dowel(d, length):
    """Hardened dowel pin d x length (ISO 8734 m6) along y from 0 to length."""
    p = ycyl(d, 0, length)
    return [("", safe_chamfer(p, "%CIRCLE", 0.12 * d), "chrome", "steel")]


def shaft_collar_8():
    """Set-screw shaft collar 8 x 16 x 8, M4 set screw. Axis y, from y=0 to y=8."""
    c = ycyl(16, 0, 8).cut(ycyl(8, -1, 9))
    c = safe_chamfer(c, "%CIRCLE", 0.4)
    c = c.cut(zcyl(4, 3.5, 9, 0, 4))
    ss = zcyl(3.7, 4.0, 7.6, 0, 4).cut(hex_prism(2.0, (0, 4, 6.0), (0, 4, 7.7)))
    return [("", c, "oxide", "steel"), ("set_screw", ss, "oxide", "steel")]


def sleeve(od, idd, length):
    """Spacer sleeve along y from 0 to length."""
    s = ycyl(od, 0, length).cut(ycyl(idd, -1, length + 1))
    return [("", safe_chamfer(s, "%CIRCLE", 0.2), "alu", "alu")]


def _rim_y(teeth, width, phase_deg):
    """Toothed rim along y in [0, width]; groove angle measured in the x-z plane from +x toward +z."""
    return cq.Workplane("XY").newObject([place(gt2.pulley_rim(teeth, width, phase_deg), tr(0, width, 0) @ rx(90))])


def gt2_pulley_20t_10mm(phase_deg=0.0):
    """GT2 20T for 10 mm belt, 5 mm bore, 2 x M3 set screws. Axis y.
    Teeth y in [0, 11], flanges 18 x 1 at y -1..0 and 11..12, hub 16 x 7 at y 12..19."""
    rim = _rim_y(20, 11.0, phase_deg)
    p = rim.union(ycyl(18, -1, 0)).union(ycyl(18, 11, 12)).union(ycyl(16, 12, 19))
    p = p.cut(ycyl(5, -2, 20))
    for a in (0, 90):
        hole = zcyl(3, 2, 9, 0, 15.5).rotate((0, 0, 0), (0, 1, 0), a)
        p = p.cut(hole)
    return [("", p, "alu", "alu")]


def gt2_pulley_generic(teeth, width, bore, hub_d, hub_l, flange_d):
    """Aluminium GT2 pulley, axis y, teeth y in [0, width], flanges 1 mm, hub on the +y side."""
    rim = _rim_y(teeth, width, 0.0)
    p = rim.union(ycyl(flange_d, -1, 0)).union(ycyl(flange_d, width, width + 1))
    if hub_l > 0:
        p = p.union(ycyl(hub_d, width + 1, width + 1 + hub_l))
    p = p.cut(ycyl(bore, -2, width + hub_l + 2))
    return [("", p, "alu", "alu")]


def ball_stud_8():
    """Ball stud: 8 mm chrome ball, neck 4.5, hex flange 7 AF x 3, M3 x 6 thread.
    Ball centre at origin, stud toward -z. Bearing face of the hex at z = -9."""
    b = sphere(8.0)
    neck = zcyl(4.5, -6.0, -2.5)
    hx = hex_prism(7.0, (0, 0, -9.0), (0, 0, -6.0))
    hx = safe_chamfer(hx, ">Z or <Z", 0.3)
    th = zcyl(2.9, -15.0, -9.0)
    th = safe_chamfer(th, "<Z", 0.3)
    return [("ball", b, "chrome", "steel"), ("stud", neck.union(hx).union(th), "steel", "steel")]


def extension_spring(length, od=5.0, wire=0.6, hook_r=2.0):
    """Extension spring with machine hooks. Hook loop centres on the z axis at z=0 and z=length;
    the loops lie in the x-z plane (they wrap a pin along y)."""
    rm = (od - wire) / 2
    z0, z1 = hook_r + wire, length - hook_r - wire
    body_len = z1 - z0
    turns = max(6, int(body_len / 0.8))
    helix = cq.Wire.makeHelix(body_len / turns, body_len, rm)
    coil = (cq.Workplane("XZ").center(rm, 0).circle(wire / 2)
            .sweep(cq.Workplane("XY").newObject([helix]), isFrenet=True)).translate((0, 0, z0))
    for zc, sgn in ((0.0, 1), (length, -1)):
        loop = cq.Workplane("XY").newObject([cq.Solid.makeTorus(hook_r, wire / 2, cq.Vector(0, 0, zc),
                                                               cq.Vector(0, 1, 0))])
        leg = cyl(wire, (rm, 0, zc + sgn * (hook_r + wire + 0.3)), (hook_r * 0.7, 0, zc + sgn * hook_r * 0.7))
        coil = coil.union(loop).union(leg)
    return [("", coil, "spring", "stainless")]


# ------------------------------------------------------------------ electronics / pneumatics
def as5600_board():
    """AS5600 magnetic encoder module 23 x 23 x 1.6, chip side at y=0 facing -y... see placement.
    Frame: board in the x-z plane, chip face toward +y at y=0, board y in [-1.6-1.5, -1.5]."""
    pcb = box(-11.5, 11.5, -3.1, -1.5, -11.5, 11.5)
    for x, z in ((-9.5, -9.5), (9.5, -9.5)):
        pcb = pcb.cut(ycyl(2.6, -4, 0, x, z))
    chip = box(-2.5, 2.5, -1.5, 0.0, -2.0, 2.0)
    hdr = box(-8.9, 8.9, -3.1 - 2.5, -3.1, 8.0, 10.5)
    pins = None
    for k in range(7):
        x = -7.62 + 2.54 * k
        p = box(x - 0.32, x + 0.32, -3.1 - 8.5, -1.5 + 3.0, 8.93, 9.57)
        pins = p if pins is None else pins.union(p)
    return [("pcb", pcb, "pcb_purple", "pcb"), ("chip", chip, "chip", "pcb"),
            ("header", hdr, "black", "abs"), ("pins", pins, "brass", "brass")]


def magnet_6x2_5():
    """Diametric NdFeB magnet 6 x 2.5, axis y, from y=0 to y=2.5. Halves N (red) / S (blue tint)."""
    m = ycyl(6, 0, 2.5)
    n = m.intersect(box(0, 4, -1, 4, -4, 4))
    s = m.intersect(box(-4, 0, -1, 4, -4, 4))
    return [("N", n, "red", "magnet"), ("S", s, "pcb_blue", "magnet")]


def adxl345_gy291():
    """GY-291 board 20.3 x 15.7 x 1.6, holes 3 dia. Board on z=0 plane (bottom face), components up."""
    pcb = box(-10.15, 10.15, -7.85, 7.85, 0, 1.6)
    for x in (-7.6, 7.6):
        pcb = pcb.cut(zcyl(3.0, -1, 3, x, 5.3))
    chip = box(-1.5, 1.5, -2.5, 0.5, 1.6, 2.6)
    hdr = box(-10.15 + 1.0, 10.15 - 1.0, -7.85, -5.3, 1.6, 4.1)
    return [("pcb", pcb, "pcb_blue", "pcb"), ("chip", chip, "chip", "pcb"), ("header", hdr, "black", "abs")]


def bellows_cup_10():
    """Vacuum cup 10 mm, 1.5 bellows, silicone, on an M5 male holder with stem.
    Frame: M5 thread top at z=0, cup lip (TCP) at z = -tip_depth (set by caller via stem length)."""
    return None   # built in parts.py where the stem length is known


def m5_pushin_straight():
    """Straight push-in fitting M5 to 4 mm tube. Thread below z=0 (4 mm), hex 8 AF x 3.5, body 10 dia x 13."""
    th = zcyl(4.8, -4.0, 0)
    hx = hex_prism(8.0, (0, 0, 0), (0, 0, 3.5))
    body = zcyl(9.0, 3.5, 13.0)
    collet = zcyl(8.0, 13.0, 15.0).cut(zcyl(4.0, 12, 16))
    return [("body", th.union(hx).union(body), "chrome", "brass"), ("collet", collet, "led", "pp")]


def m18_sensor(length=60.0):
    """M18 cylindrical through-beam photoelectric sensor, optical face at z=0 facing -z, body toward +z.
    Two M18 nuts at the bracket."""
    body = zcyl(18, 0, length)
    for k in range(int(length / 1.5)):                                # thread grooves (visual)
        z = 3 + 1.5 * k
        if z > length - 10:
            break
        body = body.cut(zcyl(19, z, z + 0.4).cut(zcyl(17.2, z - 1, z + 1)))
    lens = zcyl(14, -0.5, 0.2)
    cap = zcyl(16, length, length + 12).union(zcyl(6, length + 12, length + 30))
    nuts = []
    for z in (18, 27):
        n = hex_prism(24, (0, 0, z), (0, 0, z + 4)).cut(zcyl(18, z - 1, z + 5))
        nuts.append(n)
    return [("body", body, "chrome", "brass"), ("lens", lens, "bin_red", "pp"),
            ("cap", cap, "black", "pp"), ("nut_a", nuts[0], "steel", "steel"), ("nut_b", nuts[1], "steel", "steel")]


def ov9281_camera():
    """Arducam-style OV9281 USB module 38 x 38 board, M12 lens. Optical axis -z, lens tip at z=0.
    Board at z 20.5..22.1, lens holder 14 tall, lens 14 dia x 6.5 below the holder."""
    pcb = box(-19, 19, -19, 19, 20.5, 22.1)
    for x in (-15.5, 15.5):
        for y in (-15.5, 15.5):
            pcb = pcb.cut(zcyl(2.2, 20, 23, x, y))
    holder = box(-8, 8, -8, 8, 6.5, 20.5)
    holder = safe_fillet(holder, "|Z", 1.0)
    lens = zcyl(14, 0.0, 15.5)
    lens = lens.cut(zcyl(8, -0.1, 0.8))
    glass = zcyl(8, 0.4, 0.8)
    usb = box(-5, 5, 12, 19, 22.1, 25.3)
    return [("pcb", pcb, "pcb_green", "pcb"), ("holder", holder, "black", "abs"), ("lens", lens, "black", "alu"),
            ("glass", glass, "glass", "polycarb"), ("usb", usb, "chrome", "steel")]


def led_bar(length=100.0):
    """Aluminium LED bar 15 x 8, diffuser facing -z, centred at origin, long along x."""
    prof = box(-length / 2, length / 2, -7.5, 7.5, 0, 8)
    prof = prof.cut(box(-length / 2 - 1, length / 2 + 1, -5.5, 5.5, -0.1, 2.5))
    diff = box(-length / 2 + 0.5, length / 2 - 0.5, -5.5, 5.5, 0.2, 2.5)
    caps = box(-length / 2 - 2, -length / 2, -7.5, 7.5, 0, 8).union(box(length / 2, length / 2 + 2, -7.5, 7.5, 0, 8))
    return [("profile", prof, "alu", "alu"), ("diffuser", diff, "led", "polycarb"), ("caps", caps, "black", "abs")]


def psu_lrs150():
    """Mean Well LRS-150-24: 159 x 97 x 30, base on z=0, terminals on the -x end."""
    b = box(-79.5, 79.5, -48.5, 48.5, 0, 30)
    holes = None
    for i in range(14):
        for j in range(7):
            h = zcyl(3.2, 29, 31, -45 + 7.5 * i, -30 + 8 * j)
            holes = h if holes is None else holes.union(h)
    b = b.cut(holes)
    term = box(-79.5, -64, -40, 40, 0, 18).cut(box(-80, -70, -38, 38, 6, 19))
    return [("case", b, "alu", "alu"), ("terminals", term, "black", "abs")]


def blackpill():
    """WeAct BlackPill STM32F411: 53 x 21 x 1.6 + USB-C, pin headers down."""
    pcb = box(-26.5, 26.5, -10.5, 10.5, 0, 1.6)
    mcu = box(-3.5, 3.5, -3.5, 3.5, 1.6, 2.6).rotate((0, 0, 0), (0, 0, 1), 45)
    usb = box(-26.5 - 1, -26.5 + 6.5, -4.5, 4.5, 1.6, 4.8)
    hdr = box(-24, 24, -10.2, -7.7, -8.5, 0).union(box(-24, 24, 7.7, 10.2, -8.5, 0))
    return [("pcb", pcb, "black", "pcb"), ("mcu", mcu, "chip", "pcb"), ("usb", usb, "chrome", "steel"),
            ("headers", hdr, "black", "abs")]


def tmc2209_module():
    """TMC2209 StepStick 15 x 20 with heatsink 9 x 9 x 5, pins down."""
    pcb = box(-7.6, 7.6, -10, 10, 0, 1.6)
    hs = box(-4.5, 4.5, -4.5, 4.5, 1.6, 6.6)
    fins = None
    for k in range(4):
        f = box(-4.6 + 2.6 * k + 1.2, -4.6 + 2.6 * k + 2.2, -4.6, 4.6, 3.0, 6.7)
        fins = f if fins is None else fins.union(f)
    hs = hs.cut(fins)
    return [("pcb", pcb, "pcb_purple", "pcb"), ("heatsink", hs, "pcb_blue", "alu")]


def vacuum_pump():
    """12 V diaphragm vacuum pump (370 type): motor 27.5 dia x 38, head 40 x 26 x 26 with two 5 mm barbs.
    Base on z=0, long axis x."""
    motor = xcyl(27.5, 0, 38, 0, 15)
    head = rbox(-30, 0, -13, 13, 2, 28, 2.0)
    barbs = zcyl(5, 28, 36, -22, 5).union(zcyl(5, 28, 36, -10, 5))
    foot = box(-30, 38, -14, 14, 0, 2)
    return [("motor", motor, "chrome", "steel"), ("head", head, "black", "abs"),
            ("barbs", barbs, "white", "pp"), ("foot", foot, "black", "abs")]


def solenoid_valve():
    """3/2 normally-closed 12 V mini solenoid valve, 30 x 20 body + 22 x 22 x 26 coil, ports 5 mm. Base z=0."""
    body = box(-15, 15, -10, 10, 0, 12)
    coil = box(-11, 11, -11, 11, 12, 38)
    ports = xcyl(5, -22, -15, 0, 6).union(xcyl(5, 15, 22, 0, 6)).union(ycyl(5, 10, 16, 0, 6))
    return [("body", body, "alu", "alu"), ("coil", coil, "black", "abs"), ("ports", ports, "brass", "brass")]


def buck_converter():
    """LM2596 buck module 43 x 21, for the 12 V pump rail."""
    pcb = box(-21.5, 21.5, -10.5, 10.5, 0, 1.6)
    ind = zcyl(12, 1.6, 8.6, -6, 0)
    cap = zcyl(8, 1.6, 12.6, 12, 4).union(zcyl(8, 1.6, 12.6, -17, -4))
    return [("pcb", pcb, "pcb_blue", "pcb"), ("parts", ind.union(cap), "black", "abs")]


def estop():
    """22 mm emergency stop in a 68 x 68 x 70 yellow box, mounting face z=0, button on +z."""
    housing = rbox(-34, 34, -34, 34, 0, 62, 4.0, "|Z")
    head = zcyl(28, 62, 70).union(zcyl(40, 70, 82))
    head = safe_fillet(head, ">Z", 3.0)
    return [("box", housing, "yellow", "abs"), ("button", head, "red", "abs")]
