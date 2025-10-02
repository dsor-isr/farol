import numpy as np
import rospy
from math import pi, sin, cos
from scipy.spatial.transform import Rotation as R
np.set_printoptions(precision=3, suppress=True)

def hat(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]], dtype=float)

class LTVPlaneKF:
    """
    Linear Time-Varying KF for plane parameters x = [n; d],
    with measurement rows built from beam ranges: 0 = H x + v.
    """
    def __init__(self, n0, d0, qn=1e-6, qd=1e-3, P0_diag=( (5*np.pi/180)**2, )*3 + (0.5**2,) ):
        n0 = np.asarray(n0, dtype=float)
        n0 = n0/np.linalg.norm(n0)
        self.x = np.hstack([n0, d0]).astype(float)  # [n(3); d]
        self.P = np.diag(P0_diag)
        self.qn, self.qd = float(qn), float(qd)

    @property
    def n(self): return self.x[:3]
    @property
    def d(self): return float(self.x[3])

    def predict(self, V_I, dt):
        # x+ = F x, with F = [[I, 0],[ -V_I^T dt, 1 ]]
        F = np.eye(4)
        F[3,:3] = -V_I.reshape(1,3) * float(dt)
        Q = np.diag([self.qn, self.qn, self.qn, self.qd])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q


    def update(self, H, R):
        if H is None:
            return

        # Ambient KF step
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        innov = - H @ self.x
        delta = K @ innov

        delta_n_amb = delta[:3]
        delta_d     = float(delta[3])

        # Current normal
        n_old = self.x[:3]

        # >>> FIX: map additive correction to rotation vector via cross product
        delta_theta = np.cross(n_old, delta_n_amb)

        # Multiplicative retraction (Rodrigues)
        theta = np.linalg.norm(delta_theta)
        if theta > 0:
            u = delta_theta / theta
            Kx = np.array([[0, -u[2],  u[1]],
                        [u[2],  0, -u[0]],
                        [-u[1], u[0],  0]])
            Rn = np.eye(3) + np.sin(theta)*Kx + (1-np.cos(theta))*(Kx @ Kx)
            n_new = Rn @ n_old
        else:
            n_new = n_old

        # Optional: continuity flip (homogeneous)
        if n_old @ n_new < 0.0:
            n_new = -n_new
            delta_d = -delta_d

        d_new = self.x[3] + delta_d
        self.x[:3] = n_new
        self.x[3]  = d_new

        # Joseph covariance, then project onto NEW tangent plane (keep PD)
        I4 = np.eye(4)
        P_post_amb = (I4 - K @ H) @ self.P @ (I4 - K @ H).T + K @ R @ K.T

        Pi_new = np.eye(3) - np.outer(n_new, n_new)
        J = np.eye(4); J[:3,:3] = Pi_new
        P_proj = J @ P_post_amb @ J.T
        P_proj[:3,:3] += 1e-12 * np.outer(n_new, n_new)  # tiny radial jitter

        self.P = P_proj


class BottomFollowing:
    """
    Reworked: estimator produces plane (n,d) via LTV-KF in beam space.
    Controller unchanged in spirit: align to n, regulate |d| to d_ref,
    and move tangentially along projected heading.
    """
    def __init__(self, h, attitude, alpha, kp, ki):
        self.alpha = float(alpha)
        self.kp = float(kp)
        self.ki = float(ki)
        self.int = 0.0

        # --- Initial plane from your old closed-form (SVD) ---
        D_I, n_I, d_I = self._compute_initial_D_n_d(h, attitude)
        # Seed KF (tiny Q on n, modest on d)
        self.kf = LTVPlaneKF(n0=n_I, d0=d_I, qn=1e-6, qd=1e-3)

        # Cached debug
        self.n = n_I.copy()
        self.D_B = np.array([None,None,None])

        # Beam noise std (m): [b1,b2,b3,b4,alt]; tune via params if you want
        self.sigmas = np.array([0.02,0.02,0.02,0.02,0.01])

    # ------- Public API (node calls this) -------
    def compute(self, h, body_velocity_B, attitude, Dt):
        """
        One filter tick:
        - Build H,R from beams in inertial frame (p0=0, lever-arm ignored).
        - Predict with V^I and dt.
        - KF update with z==0.
        """
        R_IB = self._rot(attitude[0],attitude[1],attitude[2])  # body->inertial
        V_I = R_IB @ np.asarray(body_velocity_B, dtype=float)

        # Build beam unit directions in BODY, then rotate to INERTIAL
        uB = self._beam_dirs_body()
        uI = (R_IB @ uB.T).T  # 5x3

        # Assemble H and R (skip grazing beams)
        H_rows, R_diag = [], []
        n_hat = self.kf.n
        eps = 1e-3
        # p0 = 0 in inertial (origin at vehicle); lever-arm can be added later
        for i in range(5):
            z = float(h[i])
            u = uI[i] / np.linalg.norm(uI[i])
            cosn = float(u @ n_hat)
            if abs(cosn) < eps:
                continue  # grazing, poor info
            q = z * u                                   # p0 (0) + z u
            H_rows.append(np.hstack([q, 1.0]))          # [(p0+zu)^T  1]
            R_diag.append((self.sigmas[i]**2) * (cosn**2))

        H = np.vstack(H_rows) if H_rows else None
        Rm = np.diag(R_diag) if R_diag else None

        # KF step
        self.kf.predict(V_I, Dt)
        if H is not None:
            # (Optional) conditioning guard
            try:
                Minfo = H.T @ np.linalg.inv(Rm) @ H
                if np.linalg.cond(Minfo) < 1e8:
                    self.kf.update(H, Rm)
            except np.linalg.LinAlgError:
                pass

        # Cache for external users
        self.n = self.kf.n.copy()

    def controller(self, d_ref, u_ref, heading_ref, u_max):
        """
        Uses estimated (n,d) for attitude + velocity refs.
        """
        n = self.kf.n.reshape((3,1))
        if n[2] < 0:
            n = -n
        d = float(self.kf.d)
        if d<0:
            d = -d
            
        # ----- Attitude: align z_B with terrain normal n -----
        z_B_des = n
        xc = np.array([cos(heading_ref), sin(heading_ref), 0]).reshape((3,1))
        # Project heading into plane to avoid degeneracy
        xc = xc - (n @ (n.T @ xc))
        if np.linalg.norm(xc) < 1e-6:
            xc = np.array([[1.0],[0.0],[0.0]])  # fallback
        xc = xc / np.linalg.norm(xc)

        y_B_des = np.cross(n.T, xc.T).T; y_B_des /= np.linalg.norm(y_B_des)
        x_B_des = np.cross(y_B_des.T, z_B_des.T).T
        R_des = np.hstack((x_B_des, y_B_des, z_B_des))
        attitude_ref = R.from_matrix(R_des).as_euler('xyz', degrees=False)

        # ----- Velocity: priority to normal correction, then tangential -----
        # Normal regulation (note the sign: dot(d)= - n^T V_I)
        e = d_ref - abs(d)  # use absolute distance
        V_D = - self.kp * e * n

        # Tangential along projected heading in-plane
        V_P = u_ref * x_B_des

        V_T_ref = V_D + V_P
        norm = np.linalg.norm(V_T_ref)
        if norm > u_max:
            # Prioritize normal correction: keep V_D, add as much tangential as fits
            V_T_ref = V_D + (u_max - min(u_max, np.linalg.norm(V_D))) * (V_P/np.linalg.norm(V_P) if np.linalg.norm(V_P)>1e-9 else 0)

        return V_T_ref, attitude_ref

    # ------- Helpers -------
    def _beam_dirs_body(self):
        """
        5 unit directions in BODY for your current geometry.
        Beams 1..4 tilted by alpha around x/y, altimeter straight down +Z_B.
        """
        a = self.alpha
        u1 = np.array([ sin(a),  0.0,  cos(a)])
        u2 = np.array([ 0.0,    -sin(a), cos(a)])
        u3 = np.array([-sin(a),  0.0,  cos(a)])
        u4 = np.array([ 0.0,     sin(a), cos(a)])
        u5 = np.array([ 0.0,     0.0,   1.0])   # altimeter
        U = np.vstack([u1,u2,u3,u4,u5])
        # normalize (just in case)
        U = U / np.linalg.norm(U, axis=1, keepdims=True)
        return U

    def _compute_initial_D_n_d(self, h, attitude):
        """
        Your previous SVD-based closed-form, used once to seed the KF.
        Returns D_I, n_I (unit), d_I (signed, with continuity).
        """
        # Build hit points in BODY
        H_b = np.zeros((3,5))
        H_b[:,0] = [sin(self.alpha)*h[0], 0,                cos(self.alpha)*h[0]]
        H_b[:,1] = [0,                   -sin(self.alpha)*h[1], cos(self.alpha)*h[1]]
        H_b[:,2] = [-sin(self.alpha)*h[2],0,                cos(self.alpha)*h[2]]
        H_b[:,3] = [0,                    sin(self.alpha)*h[3], cos(self.alpha)*h[3]]
        H_b[:,4] = [0,                    0,                    h[4]]
        h_c = (np.sum(H_b, axis=1)/5.0).reshape((3,1))
        H_0 = H_b - h_c
        U, S, Vt = np.linalg.svd(H_0)
        n_B = U[:, -1]
        # D in BODY (unsigned distance times normal)
        D_B = abs(n_B.T @ h_c) * n_B
        R_IB = self._rot(attitude[0],attitude[1],attitude[2])
        D_I = R_IB @ D_B
        n_I = D_I / np.linalg.norm(D_I)
        d_I = float(np.linalg.norm(D_I))
        return D_I, n_I, d_I

    @staticmethod
    def _rot(roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]
        ])
