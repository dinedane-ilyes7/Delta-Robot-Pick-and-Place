"""Drawing sheets for every designed part: three orthographic views + isometric (hidden lines grey)
and a table of the dimensions that matter, with fits. Also the axial stack of the arm module.
  python3 cad/drawings.py   ->  cad/out/drawings/Delta_Cell_CAD_v2_Drawings.pdf
"""
import math
import os
import sys

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from delta_cad import gt2, parts as P  # noqa: E402
from delta_cad.dims import *  # noqa: E402,F401,F403
from delta_cad.vendor import INSERT  # noqa: E402

OUT = os.path.join(ROOT, "cad", "out", "drawings")


def svg(shape, direction, w=330, h=250):
    opt = {"width": w, "height": h, "marginLeft": 8, "marginTop": 8, "showAxes": False, "projectionDir": direction,
           "strokeWidth": 0.35, "strokeColor": (20, 20, 20), "hiddenColor": (170, 170, 170), "showHidden": True}
    s = cq.occ_impl.exporters.svg.getSVG(shape, opt)
    return s[s.index("<svg"):]


pins = ", ".join(f"({x:.2f}, {z:.2f})" for x, z in P.stop_pin_positions())
SHEETS = [
    ("M01", "Base plate", P.m01_base_plate, "6061-T6 aluminium, 6 mm", [
        ("Outline", "340.0 x 340.0 x 6.0, corners R15, edges 0.5 x 45", "+-0.2"),
        ("Flatness", "0.10 over the plate", "check on a granite plate"),
        ("Centre hole", "dia 40 (vacuum tube, cables)", "+-0.2"),
        ("Bracket holes", "12 x dia 4.5 through, 4 per arm at arm-local (+-27, -20) and (+-27, 50) about (R=110, phi)", "position +-0.05"),
        ("Frame holes", "8 x dia 5.5 at x = +-50, +-150 and y = +-150 (M5 into T-nuts of the 2020 rails)", "+-0.1"),
        ("Cable slots", "3 x 26 x 9 at arm-local (-38, 78)", ""),
    ]),
    ("M02", "Pivot bracket with motor plate (x3)", P.m02_pivot_bracket, "PETG-CF, 5 walls, 40 % gyroid, print flange down", [
        ("Arm axis to flange top", f"{Z_PLATE:.2f}", "+0 / +0.10: sets R and the arm height"),
        ("Cheeks", f"10 thick at |y| {CHEEK[0]:.0f}..{CHEEK[1]:.0f}, gap 30.0", "+-0.1"),
        ("Bearing seats (608)", f"dia 22.00, depth 8.5 from the inner face, lead-in 0.6 x 45, shoulder dia {SHOULDER_D:.0f}",
         "print a coupon: target light press, 0 to -0.02"),
        ("Bearing seats coaxial", "both seats on one axis", "0.05: ream both from one side with a 22 mm reamer"),
        ("Hard-stop pin holes", f"2 x dia 6.0 H7 through both cheeks at (x, z) = {pins}", "drill 5.8, ream 6.0"),
        ("Motor plate", f"4.0 thick at y {MPLATE_Y[0]}..{MPLATE_Y[1]}, boss slot 22.6 x +-{MOTOR_SLOT}", ""),
        ("Motor screws", f"4 x slot 3.4 x +-{MOTOR_SLOT} on 31.0 square about the motor axis", ""),
        ("Motor axis above arm axis", f"{C_BELT:.2f} nominal (300 mm GT2 belt), adjustable +-{MOTOR_SLOT}", "set by belt tension"),
        ("Flange inserts", f"4 x M4 heat-set (hole dia {INSERT[4][2]}, depth 8.6) from the top face", ""),
        ("Encoder posts", f"2 x dia 7 to y = {BOARD_Y}, M2.5 inserts (hole {INSERT[2.5][2]})", "board-to-magnet gap 1.5"),
    ]),
    ("M04", "Arm hub (x3)", P.m04_arm_hub, "PETG-CF, 100 % infill within 6 mm of the bores", [
        ("Width", f"{2 * HUB_Y:.1f} (y -8..8)", "+-0.05: spacers take up the rest"),
        ("Shaft bore", "dia 8.05 with D-flat at 3.55 from the axis", "slide fit on 8 h6"),
        ("Clamp", "slit 1.2 through the tail, M3 x 10 at x = -10, counterbore dia 6, nut trap 5.6 AF", ""),
        ("Tube socket", f"dia 12.10, x {HUB_SOCKET_X[0]:.0f}..{HUB_SOCKET_X[1]:.0f} (bond length 25), lead-in 0.6 x 45, glue groove at x 24..26",
         "0.05 radial glue gap; epoxy (Araldite 2011 or similar)"),
        ("Vent", "dia 2 from the socket bottom to the top", "lets air and glue escape"),
        ("Hard-stop tail", f"to x = -{TAIL_L:.0f}, faces at z = +-{TAIL_H:.0f}", "touches the pins at -25 and +80 deg"),
    ]),
    ("M05", "Elbow block (x3)", P.m05_elbow_block, "PETG-CF", [
        ("Tube socket", f"dia 12.10, x {ELBOW_SOCKET_X[0]:.0f}..{ELBOW_SOCKET_X[1]:.0f} (bond 27)", "coaxial with the hub socket via the bonding jig"),
        ("Ball-stud axis", f"x = {L1:.1f} from the arm axis (sets L1)", "+-0.10 after bonding (jig)"),
        ("Stud faces", f"y = +-{ELBOW_BLOCK['y']:.1f}; ball centres at y = +-{W / 2:.1f} (w = {W:.0f})", "+-0.05"),
        ("Inserts", f"2 x M3 heat-set, hole dia {INSERT[3][2]} x 6.0, then dia 3.2 to y = +-3.0", "stud thread M3 x 6 + Loctite 243"),
        ("Block", f"x {ELBOW_BLOCK['x'][0]:.0f}..{ELBOW_BLOCK['x'][1]:.0f}, 22 x 12, R2 edges, 2 pockets 10 x 14 x 2.5", ""),
    ]),
    ("M06", "Upper arm tube (x3)", P.m06_upper_tube, "pultruded carbon tube 12 x 10", [
        ("Length", f"{TUBE_UP_X[1] - TUBE_UP_X[0]:.1f}", "+-0.2, ends square and deburred"),
        ("Bonding", "abrade the ends 25 / 27 mm, clean with IPA", ""),
    ]),
    ("M07", "Rod end (x12: 6 as drawn, 6 mirrored)", P.m07_rod_end, "PETG-CF, seat up, 0.12 mm layers", [
        ("Seat", f"sphere dia {BALL_D + 0.1:.1f} about the ball centre, rim plane {CUP_RIM} past the centre", "test print with a ball first"),
        ("Socket", f"dia {TUBE_LO[0] + 0.1:.1f}, z' {ROD_SOCKET[0]:.0f}..{ROD_SOCKET[1]:.0f} (bond 14), groove at 15.5..17", ""),
        ("Spring pin", f"3 x 12 dowel, hole dia 2.95, at x' = {SPRING_POST[0]:.0f}; hook slot 3 wide at z' = {SPRING_POST[1]:.0f}", "press fit"),
        ("Swing", "cup rim clears the stud neck up to 123 deg between rod and stud axis", "checked in the CAD at 20 poses"),
    ]),
    ("M08", "Rod tube (x6)", P.m08_rod_tube, "pultruded carbon tube 6 x 4", [
        ("Length", f"{ROD_TUBE_Z[1] - ROD_TUBE_Z[0]:.1f}", "+-0.2"),
        ("Ball-to-ball after bonding", f"{L2:.2f}", "+-0.05, all six equal within 0.05: bond in a jig with two 8 mm balls"),
    ]),
    ("M09", "Platform", P.m09_platform, "PETG-CF, print flat", [
        ("Ball centres", f"pairs {W:.0f} apart at radius {r:.0f}, at 0 / 120 / 240 deg, all in z = 0", "+-0.05"),
        ("Stud faces", f"t = +-{PLAT_LUG['t']:.0f} on each lug", ""),
        ("Inserts", f"6 x M3 (hole {INSERT[3][2]}), 1 x M5 through the hub (hole {INSERT[5][2]}), 2 x M2.5 for the ADXL345", ""),
        ("Cup", f"M5 cup holder from below, push-in fitting from above; lip at z = -{TCP:.0f} (TCP)", ""),
    ]),
    ("M10", "Magnet holder (x3)", P.m10_magnet_holder, "PLA (non-magnetic)", [
        ("Bore", f"dia 8.05 x 4 on the shaft end", "glue"),
        ("Magnet pocket", "dia 6.1 x 2.5 for a 6 x 2.5 diametric magnet, 3.0 from the steel shaft", ""),
    ]),
    ("M11", "Pulley GT2 120T, 10 mm belt (x3)", P.m11_pulley_120t, "PETG-CF, 0.12 mm layers, flat", [
        ("Teeth", f"120 x GT2, PD {gt2.pd(120):.3f}, OD {gt2.od(120):.3f}, tooth depth 0.764, width 1.494 at the land", "print a 20-tooth sector and test with the belt"),
        ("Tooth width", f"{TEETH_Y[1] - TEETH_Y[0]:.1f} (y {TEETH_Y[0]:.0f}..{TEETH_Y[1]:.0f}), flanges dia {gt2.od(120) + 5:.1f} x 1.0", ""),
        ("Hub", f"dia 22, y {PULLEY_HUB_Y[0]}..{PULLEY_HUB_Y[1]}, D-bore 8.05, 2 x M3 inserts at y = 30.5 (on the flat and at 90 deg)", ""),
        ("Alternative", "buy an aluminium GT2 120T 8 mm bore pulley with the same tooth zone", ""),
    ]),
]

STACK = [
    ("Encoder board face", CHIP_FACE_Y, CHIP_FACE_Y), ("Magnet", *MAGNET_Y), ("Magnet holder", *MAG_HOLDER_Y),
    ("Shaft collar 8 x 16 x 8", *COLLAR_Y), ("Spacer 12 x 8.2 x 2.5", *SPACER_NEG),
    ("Bearing 608-2RS (a)", -BRG_Y - BRG_W / 2, -BRG_Y + BRG_W / 2), ("Cheek (a)", -CHEEK[1], -CHEEK[0]),
    ("Spacer 12 x 8.2 x 8.5", -SPACER_IN[1], -SPACER_IN[0]), ("Arm hub", -HUB_Y, HUB_Y),
    ("Spacer 12 x 8.2 x 8.5", *SPACER_IN), ("Cheek (b)", *CHEEK),
    ("Bearing 608-2RS (b)", BRG_Y - BRG_W / 2, BRG_Y + BRG_W / 2), ("Spacer 12 x 8.2 x 3.0", *SPACER_POS),
    ("120T pulley hub", *PULLEY_HUB_Y), ("Belt teeth zone (both pulleys)", *TEETH_Y), ("20T pulley hub", 46.0, 53.0),
    ("Motor plate", *MPLATE_Y), ("Motor face", MOTOR_FACE_Y, MOTOR_FACE_Y), ("Shaft 8 h6 x 84", *SHAFT_Y),
]

CSS = """
@page { size: A4 landscape; margin: 10mm; }
body { font-family: 'DejaVu Sans', Arial, sans-serif; font-size: 8.6pt; color: #111; }
.sheet { page-break-after: always; break-inside: avoid; box-sizing: border-box; border: 1.2px solid #222; padding: 5mm; height: 186mm; position: relative; overflow: hidden; }
h1 { font-size: 14pt; margin: 0 0 2mm; } h2 { font-size: 11pt; margin: 0 0 2mm; }
.views { display: grid; grid-template-columns: 1fr 1fr; gap: 2mm; width: 62%; float: left; }
.v { border: 0.6px solid #bbb; text-align: center; } .v svg { width: 100%; height: 66mm; }
.v .cap { font-size: 7.5pt; color: #555; }
.sheet:last-child { page-break-after: auto; }
.tbl { width: 36%; float: right; }
table { border-collapse: collapse; width: 100%; } th, td { border: 0.6px solid #999; padding: 1.2mm 1.6mm; vertical-align: top; text-align: left; }
th { background: #eee; }
.tb { position: absolute; bottom: 4mm; right: 6mm; width: 36%; font-size: 7.5pt; }
.tb td { padding: 0.8mm 1.4mm; }
.note { font-size: 7.6pt; color: #333; margin-top: 2mm; }
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    html = [f"<!doctype html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body>"]
    # cover / stack sheet
    rows = "".join(f"<tr><td>{n}</td><td>{a:.1f}</td><td>{b:.1f}</td><td>{b - a:.1f}</td></tr>" for n, a, b in STACK)
    html.append(f"""<div class='sheet'><h1>Delta cell CAD v2: drawing set</h1>
<p>All dimensions in mm. Arm parts are drawn in the arm-local frame: origin on the arm axis at radius R = {R:.0f},
x radial outward, y along the arm axis, z up. Rod ends are drawn in the rod frame (ball centre at the origin, rod along +z').
General tolerance for printed parts ISO 2768-m (+-0.2); fits and critical dimensions are in each sheet's table.
Hidden lines are grey.</p>
<h2>Axial stack of one arm module (along the arm axis, y)</h2>
<table style='width:60%'><tr><th>Component</th><th>y from</th><th>y to</th><th>length</th></tr>{rows}</table>
<p class='note'>Assembly: press both bearings into the cheeks from the inside; slide the shaft from +y through bearing b, spacer, hub,
spacer, bearing a; set the 3.0 spacer and the 120T pulley on +y, the 2.5 spacer and the collar on -y; preload lightly with
the collar, tighten the pulley set screws on the flat, then the hub clamp. Glue the magnet holder last, with the board removed.
Belt: 10 mm GT2, 300 mm closed (150 teeth), centre distance {C_BELT:.2f}, tension by sliding the motor in its slots.</p>
<table class='tb'><tr><td>Project</td><td>High-speed delta cell, IBISC</td></tr><tr><td>Sheet</td><td>0 / {len(SHEETS)}: stack</td></tr>
<tr><td>Source</td><td>cad/delta_cad (generated from config/robot.yaml)</td></tr></table></div>""")
    for k, (ref, title, fn, mat, dims) in enumerate(SHEETS, 1):
        shp = fn().val()
        views = [("View along y", (0, -1, 0)), ("View along z (from above)", (0, 0, 1)),
                 ("View along x", (1, 0, 0)), ("Isometric", (1, -1, 0.8))]
        vh = "".join(f"<div class='v'>{svg(shp, d)}<div class='cap'>{c}</div></div>" for c, d in views)
        bb = shp.BoundingBox()
        rows = "".join(f"<tr><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in dims)
        html.append(f"""<div class='sheet'><h1>{ref}  {title}</h1>
<div class='views'>{vh}</div>
<div class='tbl'><table><tr><th>Feature</th><th>Dimension</th><th>Tolerance / note</th></tr>{rows}
<tr><td>Envelope</td><td>{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}</td><td>x, y, z extents</td></tr></table>
<p class='note'>Material: {mat}. STEP and STL: cad/out/parts/{ref}_*.step.</p></div>
<table class='tb'><tr><td>Part</td><td>{ref} {title}</td></tr><tr><td>Sheet</td><td>{k} / {len(SHEETS)}</td></tr>
<tr><td>Units</td><td>mm, frame as in sheet 0</td></tr></table></div>""")
        print(ref, flush=True)
    html.append("</body></html>")
    path_html = os.path.join(OUT, "drawings.html")
    open(path_html, "w").write("\n".join(html))
    from playwright.sync_api import sync_playwright
    with sync_playwright() as p:
        b = p.chromium.launch()
        pg = b.new_page()
        pg.goto("file://" + path_html)
        pg.pdf(path=os.path.join(OUT, "Delta_Cell_CAD_v2_Drawings.pdf"), format="A4", landscape=True,
               print_background=True, margin=dict(top="10mm", bottom="10mm", left="10mm", right="10mm"), prefer_css_page_size=True)
        b.close()
    print("written", os.path.join(OUT, "Delta_Cell_CAD_v2_Drawings.pdf"))


if __name__ == "__main__":
    main()
