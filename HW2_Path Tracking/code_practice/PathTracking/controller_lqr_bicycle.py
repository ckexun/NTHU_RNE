import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, Q=np.eye(4), R=np.eye(1)):
        self.path = None
        self.Q = Q
        self.Q[0,0] = 1
        self.Q[1,1] = 1
        self.Q[2,2] = 1
        self.Q[3,3] = 1
        self.R = R*5000
        self.pe = 0
        self.pth_e = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            # @ 是矩陣乘法運算子（matrix multiplication operator），專門用於NumPy中的矩陣相乘
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    # State: [x, y, yaw, delta, v, l, dt]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v, l, dt = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"], info["dt"]
        yaw = utils.angle_norm(yaw)
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
        # TODO: LQR Control for Bicycle Kinematic Model
        next_delta = 0
        theta_p = np.deg2rad(target[2])
        theta = np.deg2rad(yaw)  - theta_p
        theta = (theta + np.pi) % (2 * np.pi) - np.pi 
        theta_dot = (theta - self.pth_e)/dt

        dx = x - target[0]
        dy = y - target[1]
        yaw_vec = np.array([np.cos(theta_p), np.sin(theta_p)])
        norm_vec = np.array([-yaw_vec[1], yaw_vec[0]])  
        e = np.dot(np.array([dx, dy]), norm_vec)
        e_dot = (e - self.pe)/dt

        self.pe = e
        self.pth_e = theta
                
        A = np.array([
            [1, dt, 0,  0],
            [0,  0, v,  0],
            [0,  0, 1, dt],
            [0,  0, 0,  0]])

        B = np.array([
            [0],
            [0],
            [0],
            [v/l]])
        
        X = np.array([
            [e], 
            [e_dot], 
            [theta], 
            [theta_dot]])

        P = self._solve_DARE(A, B, self.Q, self.R)

        K = np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A
        next_delta = -K @ X
        next_delta = float(next_delta[0])
        next_delta = np.rad2deg(next_delta)
        return next_delta, target
