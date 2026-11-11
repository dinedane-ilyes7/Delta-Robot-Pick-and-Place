"""Offscreen renders with PyVista (needs no display)."""
import numpy as np
import pyvista as pv

from .lib import RGB, tessellate

pv.OFF_SCREEN = True


def to_poly(shape, tol=0.05, ang=0.2):
    v, f = tessellate(shape, tol, ang)
    if len(f) == 0:
        return None
    faces = np.hstack([np.full((len(f), 1), 3), f]).ravel()
    return pv.PolyData(v, faces)


def render(items, path, view="iso", size=(1600, 1200), zoom=1.0, focus=None, tol=0.05, edges=False,
           bg="white", elev=None, azim=None, parallel=False, pscale=None):
    """items: list of (shape, colour_key). view: iso, front, top, side or a (position, focal, up) tuple."""
    pl = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
    pl.set_background(bg)
    groups = {}
    for shp, ck in items:
        groups.setdefault(ck, []).append(shp)
    for ck, shapes in groups.items():
        polys = [p for p in (to_poly(s, tol) for s in shapes) if p is not None]
        if not polys:
            continue
        mesh = polys[0].merge(polys[1:]) if len(polys) > 1 else polys[0]
        metal = ck in ("alu", "alu_dark", "steel", "chrome", "brass", "spring", "magnet")
        pl.add_mesh(mesh, color=RGB.get(ck, (0.6, 0.6, 0.6)), smooth_shading=True, split_sharp_edges=True,
                    feature_angle=35, specular=0.6 if metal else 0.25, specular_power=30 if metal else 12,
                    metallic=0.0, show_edges=False, opacity=0.35 if ck == "glass" else 1.0)
        if edges:
            fe = mesh.extract_feature_edges(feature_angle=35, boundary_edges=False, non_manifold_edges=False,
                                            manifold_edges=False)
            if fe.n_points:
                pl.add_mesh(fe, color=(0.05, 0.05, 0.05), line_width=0.6, opacity=0.35)
    if parallel:
        pl.enable_parallel_projection()
    if isinstance(view, tuple):
        pl.camera_position = view
    else:
        {"iso": pl.view_isometric, "front": pl.view_xz, "top": pl.view_xy, "side": pl.view_yz}[view]()
    if parallel and pscale:
        pl.camera.parallel_scale = pscale
    if focus is not None:
        pl.camera.focal_point = focus
    if azim:
        pl.camera.azimuth = azim
    if elev:
        pl.camera.elevation = elev
    pl.camera.zoom(zoom)
    pl.enable_anti_aliasing("ssaa")
    pl.screenshot(path)
    pl.close()
    return path
