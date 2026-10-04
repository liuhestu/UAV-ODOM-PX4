"""SE(3) adapter, ROS Hamilton xyzw. T_AB maps B coordinates into A."""
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(R):
    # Eigenvector formulation remains stable near 180 degrees.
    m = np.asarray(R)
    K = np.array([[m[0,0]-m[1,1]-m[2,2], m[1,0]+m[0,1], m[2,0]+m[0,2], m[2,1]-m[1,2]],
                  [m[1,0]+m[0,1], m[1,1]-m[0,0]-m[2,2], m[2,1]+m[1,2], m[0,2]-m[2,0]],
                  [m[2,0]+m[0,2], m[2,1]+m[1,2], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
                  [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]]) / 3
    q = np.linalg.eigh(K)[1][:, -1]
    return q if q[3] >= 0 else -q


def vector(v):
    a = np.asarray(v, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError('invalid vector')
    return a


def covariance(c):
    c = np.asarray(c, dtype=float).reshape(6, 6)
    if not np.isfinite(c).all() or not np.allclose(c, c.T, atol=1e-6):
        raise ValueError('invalid covariance')
    c = (c+c.T)*0.5
    if np.linalg.eigvalsh(c).min() < -1e-7:
        raise ValueError('covariance is not positive semidefinite')
    return c


class Adapter:
    def __init__(self, cfg):
        self.cfg = cfg
        self.R_aw = rotation(cfg['world_alignment']['rotation_xyzw'])
        self.t_aw = vector(cfg['world_alignment']['translation_xyz'])
        # T_SB: base_link origin and axes expressed in sensor frame.
        self.R_sb = rotation(cfg['extrinsic']['rotation_xyzw'])
        self.t_sb = vector(cfg['extrinsic']['translation_xyz'])
        if cfg['input']['linear_velocity_frame'] not in ('body', 'world'):
            raise ValueError('linear_velocity_frame must be body or world')
        if cfg['input']['orientation_covariance_frame'] not in ('body', 'world'):
            raise ValueError('orientation_covariance_frame must be body or world')
        if cfg['input']['angular_velocity_frame'] != 'body':
            raise ValueError('angular velocity must be in sensor body frame')
        self.floors = {}
        for key in ('pose', 'twist'):
            f = np.asarray(cfg['adapter'][key+'_variance_floor'], dtype=float)
            if f.shape != (6,) or not np.isfinite(f).all() or (f < 0).any():
                raise ValueError('invalid variance floor')
            self.floors[key] = f

    def adapt(self, p, q, v, w, pc, tc):
        R_ws = rotation(q)
        p, v, w = vector(p), vector(v), vector(w)
        pc, tc = covariance(pc), covariance(tc)
        body_cov = self.cfg['input']['orientation_covariance_frame'] == 'body'
        R_ab = self.R_aw @ R_ws @ self.R_sb
        p_ab = self.R_aw @ (p+R_ws@self.t_sb)+self.t_aw
        V = np.eye(3) if self.cfg['input']['linear_velocity_frame']=='body' else R_ws.T
        R_bs = self.R_sb.T
        v_b = R_bs @ (V@v + np.cross(w, self.t_sb))
        w_b = R_bs @ w
        Jp = np.zeros((6, 6))
        Jp[:3,:3] = self.R_aw
        Jp[:3,3:] = -self.R_aw @ (R_ws@skew(self.t_sb) if body_cov else skew(R_ws@self.t_sb))
        Jp[3:,3:] = self.R_aw @ (R_ws if body_cov else np.eye(3))
        Jt = np.zeros((6, 6))
        Jt[:3,:3] = R_bs @ V
        Jt[:3,3:] = -R_bs @ skew(self.t_sb)
        Jt[3:,3:] = R_bs
        pc, tc = Jp@pc@Jp.T, Jt@tc@Jt.T
        for c, key in ((pc, 'pose'), (tc, 'twist')):
            c[np.diag_indices(6)] += np.maximum(0, self.floors[key]-np.diag(c))
        return p_ab, quaternion(R_ab), v_b, w_b, pc, tc
