"""Delta kinematics. Frame: z up, base joints at z=0, platform below.
theta_i > 0 rotates upper arm i downward. Elbow-out branch."""
import numpy as np
from params import PHI


def rz(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


ROT = [rz(p) for p in PHI]


class Delta:
    def __init__(self, R, r, L1, L2):
        self.R, self.r, self.L1, self.L2 = R, r, L1, L2

    # elbow position and its derivatives wrt theta (world frame), vectorised over theta array
    def elbow(self, i, th):
        th = np.asarray(th)
        loc = np.stack([self.R + self.L1*np.cos(th), 0*th, -self.L1*np.sin(th)], -1)
        return loc @ ROT[i].T

    def d_elbow(self, i, th):
        th = np.asarray(th)
        loc = np.stack([-self.L1*np.sin(th), 0*th, -self.L1*np.cos(th)], -1)
        return loc @ ROT[i].T

    def dd_elbow(self, i, th):
        th = np.asarray(th)
        loc = np.stack([-self.L1*np.cos(th), 0*th, self.L1*np.sin(th)], -1)
        return loc @ ROT[i].T

    def pjoint(self, i, x):
        return x + ROT[i] @ np.array([self.r, 0, 0])

    def ik(self, x):
        """x: (...,3) -> theta (...,3), valid mask (...)"""
        x = np.asarray(x, float)
        th = np.empty(x.shape[:-1] + (3,))
        ok = np.ones(x.shape[:-1], bool)
        for i in range(3):
            p = x @ ROT[i]            # world -> arm local (R^T x)
            u = self.R - self.r - p[..., 0]
            py, pz = p[..., 1], p[..., 2]
            A = 2*u*self.L1
            B = 2*pz*self.L1
            C = self.L2**2 - self.L1**2 - u**2 - py**2 - pz**2
            rho = np.hypot(A, B)
            c = C/np.where(rho > 0, rho, 1)
            ok &= np.abs(c) <= 1
            t = np.arctan2(B, A) + np.arccos(np.clip(c, -1, 1))
            th[..., i] = (t + np.pi) % (2*np.pi) - np.pi
        return th, ok

    def fk(self, th):
        th = np.asarray(th, float)
        cs = [self.elbow(i, th[..., i]) - ROT[i] @ np.array([self.r, 0, 0]) for i in range(3)]
        p1, p2, p3 = cs
        d = np.linalg.norm(p2 - p1, axis=-1)
        ex = (p2 - p1)/d[..., None]
        i_ = np.sum(ex*(p3 - p1), -1)
        ey = p3 - p1 - i_[..., None]*ex
        j = np.linalg.norm(ey, axis=-1)
        ey = ey/j[..., None]
        ez = np.cross(ex, ey)
        xx = d/2
        yy = (i_**2 + j**2)/(2*j) - i_*xx/j
        zz2 = self.L2**2 - xx**2 - yy**2
        zz = np.sqrt(np.clip(zz2, 0, None))
        sol_a = p1 + xx[..., None]*ex + yy[..., None]*ey + zz[..., None]*ez
        sol_b = p1 + xx[..., None]*ex + yy[..., None]*ey - zz[..., None]*ez
        return np.where((sol_a[..., 2] < sol_b[..., 2])[..., None], sol_a, sol_b)

    def AB(self, x, th):
        """Rows s_i = P_i - E_i; A = [s_i^T], B = diag(s_i . dE_i). xdot = A^-1 B thdot."""
        x = np.asarray(x, float)
        S = np.stack([self.pjoint(i, x) - self.elbow(i, th[..., i]) for i in range(3)], -2)
        b = np.stack([np.sum(S[..., i, :]*self.d_elbow(i, th[..., i]), -1) for i in range(3)], -1)
        return S, b

    def jacobian(self, x, th):
        S, b = self.AB(x, th)
        return np.linalg.solve(S, np.eye(3)*b[..., None, :]) if S.ndim == 2 else \
            np.linalg.solve(S, b[..., None, :]*np.eye(3))

    # ---- task -> joint rates (exact, from the differentiated closure constraint) ----
    def ik_rates(self, x, xd, xdd):
        th, ok = self.ik(x)
        thd = np.empty_like(th); thdd = np.empty_like(th)
        for i in range(3):
            E = self.elbow(i, th[..., i]); dE = self.d_elbow(i, th[..., i]); ddE = self.dd_elbow(i, th[..., i])
            s = self.pjoint(i, x) - E
            b = np.sum(s*dE, -1)
            thd[..., i] = np.sum(s*xd, -1)/b
            v = xd - dE*thd[..., i][..., None]
            thdd[..., i] = (np.sum(s*xdd, -1) + np.sum(v*v, -1) - np.sum(s*ddE, -1)*thd[..., i]**2)/b
        return th, thd, thdd, ok

    # ---- joint -> task (used for mass matrix and tests) ----
    def fk_rates(self, th, thd, thdd):
        x = self.fk(th)
        S, b = self.AB(x, th)
        xd = np.linalg.solve(S, b*thd)
        rhs = np.empty(3)
        for i in range(3):
            dE = self.d_elbow(i, th[i]); ddE = self.dd_elbow(i, th[i])
            Edd = dE*thdd[i] + ddE*thd[i]**2
            v = xd - dE*thd[i]
            rhs[i] = S[i] @ Edd - v @ v
        xdd = np.linalg.solve(S, rhs)
        return x, xd, xdd


def task_points(n_r=6, n_a=12, n_z=5, radius=None, stroke=None, z_top=0.0):
    from params import TASK_RADIUS, TASK_STROKE
    radius = TASK_RADIUS if radius is None else radius
    stroke = TASK_STROKE if stroke is None else stroke
    pts = []
    for z in np.linspace(z_top, z_top - stroke, n_z):
        pts.append([0, 0, z])
        for rr in np.linspace(radius/n_r, radius, n_r):
            for a in np.linspace(0, 2*np.pi, n_a, endpoint=False):
                pts.append([rr*np.cos(a), rr*np.sin(a), z])
    return np.array(pts)
