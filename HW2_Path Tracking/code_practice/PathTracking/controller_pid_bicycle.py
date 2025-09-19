import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPIDBicycle(Controller):
    def __init__(self, kp=0.4, ki=0.0001, kd=0.5):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
    
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State
        x, y, dt = info["x"], info["y"], info["dt"]

        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        
        # TODO: PID Control for Bicycle Kinematic Model
        next_delta = 0
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        lookahead_idx = min(min_idx + 1, len(self.path) - 1)
        next_target = self.path[lookahead_idx]

        v_x = next_target[0] - target[0]
        v_y = next_target[1] - target[1]

        d_x = x - target[0]
        d_y = y - target[1]

        cross_product = d_x * v_y - d_y * v_x 

        ep = np.sign(cross_product) * min_dist

        self.acc_ep += dt * ep
        diff_ep = (ep - self.last_ep) / dt
        next_delta = self.kp * ep + self.ki * self.acc_ep + self.kd * diff_ep
        self.last_ep = ep

        return next_delta, target
