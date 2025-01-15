import numpy as np

class RigidBody:
    def __init__(self, dt = 1/100):
        self.a = [0,0,0]
        self.v = [0,0,0]
        self.r = [0,0,0]
        self.Body2Inertial = np.array([
            [1,0],
            [0,1]
        ])
        self.dt = dt
    def integrate(self, fx):
        pass