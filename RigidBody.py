import numpy as np

class RigidBody:
    def __init__(self, dt = 1/100):
        self.AinI = [0,0,0]
        self.VinI = [0,0,0]
        self.RinI = [0,0,0]
        self.Body2Inertial = np.array([
            [1,0],
            [0,1]
        ])
        self.dt = dt
    def integrate(self, fx):
        pass