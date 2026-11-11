"""GT2 (2 mm pitch) pulleys and toothed belts with the real tooth profile.

Profile: the GT2 2 mm groove used by the open-source parametric pulley generators
(tooth depth 0.764 mm, tooth width 1.494 mm at the land), given as depth vs half-width.
Pitch line sits 0.254 mm outside the pulley land, so OD = PD - 0.508.
Belt: total thickness 1.38 mm, tooth height 0.75 mm, backing 0.63 mm.
"""
import math

import cadquery as cq
import numpy as np

PITCH = 2.0
PLD = 0.254                      # pitch line differential (land to pitch line)
BELT_BACK = 0.63                 # backing thickness above the land
BELT_TOOTH_H = 0.75
GROOVE = np.array([              # (half-width x, depth) from the land, x increasing
    (0.000000, 0.764470), (0.127259, 0.750044), (0.248400, 0.707276), (0.358229, 0.636924),
    (0.451556, 0.539750), (0.504649, 0.443762), (0.547158, 0.343077), (0.578556, 0.238423),
    (0.598311, 0.130528), (0.647876, 0.037218), (0.747183, 0.000000)])


def pd(teeth):
    return teeth * PITCH / math.pi


def od(teeth):
    return pd(teeth) - 2 * PLD


def groove_depth(x):
    """Depth of the pulley groove at tangential offset x from the groove centre (0 on the land)."""
    x = abs(x)
    if x >= GROOVE[-1, 0]:
        return 0.0
    return float(np.interp(x, GROOVE[:, 0], GROOVE[:, 1]))


def tooth_height(x):
    """Belt tooth: groove shape, 4 % narrower and scaled to 0.75 mm, so it sits with clearance."""
    return groove_depth(x / 0.96) * BELT_TOOTH_H / GROOVE[0, 1]


def pulley_profile(teeth, phase_deg=0.0):
    """Closed polyline (x, y) of the toothed rim, groove centres at phase + k*360/teeth."""
    ro = od(teeth) / 2
    xs = np.concatenate([-GROOVE[::-1, 0], GROOVE[1:, 0]])          # across one groove
    pts = []
    dth = 2 * math.pi / teeth
    for k in range(teeth):
        c = math.radians(phase_deg) + k * dth
        for x in xs:
            a = c + x / ro
            rr = ro - groove_depth(x)
            pts.append((rr * math.cos(a), rr * math.sin(a)))
        a = c + dth / 2                                              # middle of the land
        pts.append((ro * math.cos(a), ro * math.sin(a)))
    return pts


def pulley_rim(teeth, width, phase_deg=0.0):
    """Toothed solid of the given width along +z from z=0 (no bore).
    One B-spline per groove and one arc per land, so STEP files stay small and surfaces smooth."""
    pts = pulley_profile(teeth, phase_deg)
    per = len(pts) // teeth
    edges = []
    for k in range(teeth):
        g = [cq.Vector(x, y, 0) for x, y in pts[k * per:(k + 1) * per - 1]]
        land = cq.Vector(*pts[(k + 1) * per - 1], 0)
        nxt = cq.Vector(*pts[((k + 1) * per) % len(pts)], 0)
        edges.append(cq.Edge.makeSpline(g))
        edges.append(cq.Edge.makeThreePointArc(g[-1], land, nxt))
    face = cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))
    return cq.Workplane("XY").newObject([cq.Solid.extrudeLinear(face, cq.Vector(0, 0, width))])


# ------------------------------------------------------------------ open belt on two pulleys
class BeltPath:
    """Pitch line of an open belt on pulley 1 (centre c1, teeth n1) and pulley 2 (c2, n2), in a plane (u, v).
    Travel is counter-clockwise; s = 0 at the tangent point where the belt leaves pulley 2 toward pulley 1."""

    def __init__(self, c1, n1, c2, n2):
        self.c1, self.c2 = np.asarray(c1, float), np.asarray(c2, float)
        self.r1, self.r2 = pd(n1) / 2, pd(n2) / 2
        d = self.c2 - self.c1
        self.C = float(np.linalg.norm(d))
        self.e = d / self.C                                          # pulley 1 -> pulley 2
        self.f = np.array([-self.e[1], self.e[0]])                   # left of e
        b = math.asin((self.r1 - self.r2) / self.C)
        self.beta = b
        # outward normals at the tangent points, right side (-f) and left side (+f)
        nR = -self.f * math.cos(b) + self.e * math.sin(b)
        nL = self.f * math.cos(b) + self.e * math.sin(b)
        self.nR, self.nL = nR, nL
        self.span = self.C * math.cos(b)
        self.wrap1 = math.pi + 2 * b
        self.wrap2 = math.pi - 2 * b
        # segments in travel order: left straight (2->1), arc on 1, right straight (1->2), arc on 2
        self.seg = [("line", self.c2 + self.r2 * nL, self.c1 + self.r1 * nL, self.span),
                    ("arc", self.c1, self.r1, math.atan2(nL[1], nL[0]), self.wrap1),
                    ("line", self.c1 + self.r1 * nR, self.c2 + self.r2 * nR, self.span),
                    ("arc", self.c2, self.r2, math.atan2(nR[1], nR[0]), self.wrap2)]
        self.length = 2 * self.span + self.r1 * self.wrap1 + self.r2 * self.wrap2

    def at(self, s):
        """Point and inward normal (toward the inside of the loop) at arc length s."""
        s = s % self.length
        for sg in self.seg:
            L = sg[3] if sg[0] == "line" else sg[2] * sg[4]
            if s <= L:
                if sg[0] == "line":
                    p0, p1 = sg[1], sg[2]
                    t = (p1 - p0) / np.linalg.norm(p1 - p0)
                    return p0 + s * t, np.array([-t[1], t[0]])        # left of travel = inside (CCW)
                c, rr, a0 = sg[1], sg[2], sg[3]
                a = a0 + s / rr
                n = np.array([math.cos(a), math.sin(a)])
                return c + rr * n, -n
            s -= L
        raise RuntimeError

    def arc_start(self, which):
        """Arc length at which the belt starts wrapping pulley 1 or 2, and that start angle."""
        if which == 1:
            return self.span, self.seg[1][3]
        return 2 * self.span + self.r1 * self.wrap1, self.seg[3][3]


def solve_centre_distance(n1, n2, teeth_belt):
    """Centre distance for a closed belt of teeth_belt teeth (Newton on the exact open-belt length)."""
    L = teeth_belt * PITCH
    r1, r2 = pd(n1) / 2, pd(n2) / 2
    C = (L - math.pi * (r1 + r2)) / 2
    for _ in range(50):
        b = math.asin((r1 - r2) / C)
        f = 2 * C * math.cos(b) + r1 * (math.pi + 2 * b) + r2 * (math.pi - 2 * b) - L
        h = 1e-6
        b2 = math.asin((r1 - r2) / (C + h))
        f2 = 2 * (C + h) * math.cos(b2) + r1 * (math.pi + 2 * b2) + r2 * (math.pi - 2 * b2) - L
        C -= f / ((f2 - f) / h)
    return C


def belt_solid(path, width, groove_angle_1_deg, ds=0.08):
    """Toothed belt along path, extruded along +z (plane (u, v) = (x, y)) over [0, width].
    The tooth phase is set so a tooth sits in a groove of pulley 1 whose centre is at groove_angle_1_deg.
    Returns (solid, phase_of_pulley2_deg) where the second value is a groove angle that pulley 2 must have."""
    n = int(round(path.length / PITCH))
    s1, a1 = path.arc_start(1)
    # tooth centred at s where the arc angle equals the groove angle (mod pitch)
    da = (math.radians(groove_angle_1_deg) - a1) % (PITCH / path.r1)
    s0 = s1 + da * path.r1
    pitch = path.length / n                                           # exact pitch along this path
    s2, a2 = path.arc_start(2)
    k = math.ceil((s2 - s0) / pitch)
    s_tooth_on_2 = s0 + k * pitch
    phase2 = math.degrees(a2 + (s_tooth_on_2 - s2) / path.r2)

    # inner (tooth) side: one spline per pitch, from land middle to land middle
    edges = []
    per = 24
    for k in range(n):
        sa = s0 + (k - 0.5) * pitch
        pts = []
        for j in range(per + 1):
            s_ = sa + pitch * j / per
            p, nin = path.at(s_)
            x = s_ - (s0 + k * pitch)
            pts.append(cq.Vector(*(p + nin * (PLD - 0.02) + nin * tooth_height(x)), 0))
        if edges:
            pts[0] = edges[-1].endPoint()
        if k == n - 1:
            pts[-1] = edges[0].startPoint()
        edges.append(cq.Edge.makeSpline(pts))
    wi = cq.Workplane("XY").newObject([cq.Solid.extrudeLinear(cq.Face.makeFromWires(cq.Wire.assembleEdges(edges)),
                                                              cq.Vector(0, 0, width))])
    # outer (back) side: exact lines and arcs
    off = BELT_BACK - (PLD - 0.02)
    oe = []
    for sg in path.seg:
        if sg[0] == "line":
            p0, p1 = sg[1], sg[2]
            t = (p1 - p0) / np.linalg.norm(p1 - p0)
            nout = -np.array([-t[1], t[0]])
            oe.append(cq.Edge.makeLine(cq.Vector(*(p0 + off * nout), 0), cq.Vector(*(p1 + off * nout), 0)))
        else:
            c, rr, a0, wrap = sg[1], sg[2], sg[3], sg[4]
            pt = [c + (rr + off) * np.array([math.cos(a0 + wrap * u), math.sin(a0 + wrap * u)]) for u in (0, 0.5, 1)]
            oe.append(cq.Edge.makeThreePointArc(*[cq.Vector(*q, 0) for q in pt]))
    wo = cq.Workplane("XY").newObject([cq.Solid.extrudeLinear(cq.Face.makeFromWires(cq.Wire.assembleEdges(oe)),
                                                              cq.Vector(0, 0, width))])
    return wo.cut(wi), phase2
