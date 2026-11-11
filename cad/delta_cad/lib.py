"""Geometry helpers, transforms, materials and colours shared by every module. Units: mm, g."""
import math

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf

V = cq.Vector

# ------------------------------------------------------------------ materials (g/mm^3) and colours
DENSITY = {
    "petg_cf": 1.30e-3, "pla": 1.24e-3, "tpu": 1.21e-3, "alu": 2.70e-3, "steel": 7.85e-3,
    "stainless": 7.95e-3, "brass": 8.50e-3, "carbon": 1.55e-3, "pvc": 1.40e-3, "rubber": 1.20e-3,
    "pcb": 1.85e-3, "magnet": 7.50e-3, "pp": 0.91e-3, "zinc": 6.6e-3, "silicone": 1.15e-3,
    "neoprene": 1.25e-3, "abs": 1.05e-3, "polycarb": 1.20e-3,
}
# printed parts: effective density with 4 walls + 40 % gyroid, measured on similar parts
PRINT_FILL = 0.80

RGB = {
    "print":   (0.30, 0.32, 0.35),   # PETG-CF, graphite
    "print2":  (0.86, 0.42, 0.10),   # accent prints (cable clips, bin tray), orange
    "alu":     (0.80, 0.81, 0.83),
    "alu_dark": (0.55, 0.56, 0.58),
    "steel":   (0.62, 0.63, 0.66),
    "chrome":  (0.90, 0.90, 0.92),
    "black":   (0.07, 0.07, 0.08),
    "oxide":   (0.16, 0.16, 0.17),   # black-oxide screws
    "carbon":  (0.10, 0.10, 0.11),
    "brass":   (0.80, 0.62, 0.25),
    "rubber":  (0.10, 0.10, 0.10),
    "belt":    (0.12, 0.12, 0.13),
    "conv_belt": (0.13, 0.42, 0.25),  # green PVC conveyor belt
    "pcb_blue": (0.10, 0.25, 0.55),
    "pcb_green": (0.08, 0.40, 0.20),
    "pcb_purple": (0.35, 0.15, 0.45),
    "chip":    (0.05, 0.05, 0.05),
    "white":   (0.93, 0.93, 0.90),
    "cup":     (0.20, 0.45, 0.85),   # blue silicone
    "bin":     (0.95, 0.55, 0.12),
    "bin_red": (0.80, 0.15, 0.12),
    "bin_grey": (0.45, 0.47, 0.50),
    "led":     (1.00, 0.98, 0.85),
    "red":     (0.85, 0.10, 0.10),
    "yellow":  (0.95, 0.80, 0.10),
    "magnet":  (0.75, 0.75, 0.78),
    "spring":  (0.72, 0.73, 0.75),
    "glass":   (0.70, 0.85, 0.95),
}


def color(key):
    r, g, b = RGB[key]
    return cq.Color(r, g, b)


# ------------------------------------------------------------------ solids
def _wp(shape):
    return cq.Workplane("XY").newObject([shape])


def cyl(d, p0, p1):
    """Cylinder of diameter d between two points."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    v = p1 - p0
    h = float(np.linalg.norm(v))
    return _wp(cq.Solid.makeCylinder(d / 2, h, V(*p0), V(*(v / h))))


def cone(d0, d1, p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    v = p1 - p0
    h = float(np.linalg.norm(v))
    return _wp(cq.Solid.makeCone(d0 / 2, d1 / 2, h, V(*p0), V(*(v / h))))


def xcyl(d, x0, x1, y=0.0, z=0.0):
    return cyl(d, (x0, y, z), (x1, y, z))


def ycyl(d, y0, y1, x=0.0, z=0.0):
    return cyl(d, (x, y0, z), (x, y1, z))


def zcyl(d, z0, z1, x=0.0, y=0.0):
    return cyl(d, (x, y, z0), (x, y, z1))


def sphere(d, c=(0, 0, 0)):
    return _wp(cq.Solid.makeSphere(d / 2, V(*c), angleDegrees1=-90, angleDegrees2=90))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def rbox(x0, x1, y0, y1, z0, z1, r, sel=None):
    """Box with filleted edges (all edges, or a selector such as '|Z')."""
    b = box(x0, x1, y0, y1, z0, z1)
    try:
        return (b.edges(sel) if sel else b.edges()).fillet(r)
    except Exception:
        return b


def tube(od, idd, p0, p1):
    return cyl(od, p0, p1).cut(cyl(idd, p0, p1))


def hex_prism(af, p0, p1, rot=0.0):
    """Hexagonal prism, across-flats af, axis p0->p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    v = p1 - p0
    h = float(np.linalg.norm(v))
    pr = cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(h)
    pr = pr.rotate((0, 0, 0), (0, 0, 1), rot)
    return _wp(place(pr, frame_from_z(p0, v / h)))


def bore_with_leadin(d, p_mouth, p_bottom, chamfer=0.5):
    """Blind bore from p_mouth to p_bottom with a 45 deg lead-in chamfer at the mouth (cut tool)."""
    p_mouth, p_bottom = np.asarray(p_mouth, float), np.asarray(p_bottom, float)
    u = (p_bottom - p_mouth) / np.linalg.norm(p_bottom - p_mouth)
    tool = cyl(d, p_mouth - 0.5 * u, p_bottom)
    return tool.union(cone(d + 2 * chamfer + 1.0, d - 0.01, p_mouth - 0.5 * u, p_mouth + (chamfer + 0.5) * u))


def safe_fillet(wp, sel, r):
    try:
        out = wp.edges(sel).fillet(r)
        return out if out.val().isValid() else wp
    except Exception:
        return wp


def safe_chamfer(wp, sel, c):
    try:
        out = wp.edges(sel).chamfer(c)
        return out if out.val().isValid() else wp
    except Exception:
        return wp


def ring_pattern(tool_fn, n, phase=0.0, axis=(0, 0, 1)):
    out = None
    for k in range(n):
        t = tool_fn().rotate((0, 0, 0), axis, phase + 360.0 * k / n)
        out = t if out is None else out.union(t)
    return out


# ------------------------------------------------------------------ transforms (4x4, mm)
def rz(deg):
    a = math.radians(deg)
    M = np.eye(4)
    M[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return M


def ry(deg):
    a = math.radians(deg)
    M = np.eye(4)
    M[0, 0], M[0, 2], M[2, 0], M[2, 2] = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
    return M


def rx(deg):
    a = math.radians(deg)
    M = np.eye(4)
    M[1, 1], M[1, 2], M[2, 1], M[2, 2] = math.cos(a), -math.sin(a), math.sin(a), math.cos(a)
    return M


def tr(x, y, z):
    M = np.eye(4)
    M[:3, 3] = [x, y, z]
    return M


MIRROR_Y = np.diag([1.0, -1.0, 1.0, 1.0])


def frame_from_z(p, z, xhint=None):
    """Frame at p whose z axis is z."""
    z = np.asarray(z, float)
    z = z / np.linalg.norm(z)
    if xhint is None:
        xhint = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.asarray(xhint, float) - np.dot(xhint, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, p
    return M


def place(shape, M):
    """Apply a 4x4 homogeneous matrix to a Workplane or Shape; returns a Shape.
    A matrix with negative determinant (mirror) is applied with gp_Trsf as well."""
    s = shape.val() if isinstance(shape, cq.Workplane) else shape
    M = np.asarray(M, float)
    if np.linalg.det(M[:3, :3]) < 0:
        # mirror: OCC needs a mirror transformation, do it in two steps
        Mr = M @ MIRROR_Y                       # proper rotation
        s = s.mirror("XZ", (0, 0, 0))
        M = Mr
    t = gp_Trsf()
    t.SetValues(*M[0, :4], *M[1, :4], *M[2, :4])
    return s.moved(cq.Location(t))


# ------------------------------------------------------------------ mass properties
def mass_props(shape, density):
    """Mass (g), centre of mass (mm) and inertia about the COM (g mm^2) of a solid."""
    s = shape.val() if isinstance(shape, cq.Workplane) else shape
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(s.wrapped, props)
    vol = props.Mass()
    c = props.CentreOfMass()
    I = props.MatrixOfInertia()
    Imat = np.array([[I.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]) * density
    return vol * density, np.array([c.X(), c.Y(), c.Z()]), Imat


def combine_props(items):
    """items: (mass, com, I_com). Returns total mass, com, inertia about com."""
    m = sum(i[0] for i in items)
    c = sum(i[0] * i[1] for i in items) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in items:
        d = ci - c
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, c, I


# ------------------------------------------------------------------ tessellation / STL
def tessellate(shape, tol=0.03, ang=0.15):
    s = shape.val() if isinstance(shape, cq.Workplane) else shape
    verts, tris = s.tessellate(tol, ang)
    v = np.array([[p.x, p.y, p.z] for p in verts], float)
    f = np.array(tris, int).reshape(-1, 3)
    return v, f


def write_stl(path, meshes):
    """meshes: list of (verts, faces). Binary STL."""
    tri = []
    for v, f in meshes:
        if len(f):
            tri.append(v[f])
    tri = np.concatenate(tri) if tri else np.zeros((0, 3, 3))
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    data = np.zeros(len(tri), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    data["n"], data["v"] = n, tri
    with open(path, "wb") as fh:
        fh.write(b"delta_cad".ljust(80, b" "))
        fh.write(np.uint32(len(tri)).tobytes())
        fh.write(data.tobytes())
