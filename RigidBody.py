import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
class RigidBody:
    def __init__(self, dt = 1/100):
        self.rInI = array([0,0,0])
        self.vInI = array([0,0,0])
        self.aInI = array([0,0,0])
        self.rInIhist = []
        self.vInIhist = []
        self.aInIhist = []
        self.IB = np.array([ #body 2 inertial dcm
            [1,0],
            [0,1]
        ])

        self.dt = dt
    def integrate(self, fx):
        pass
    def storeStates(self, fx):
        self.aInIhist.append(self.aInI)
        self.vInIhist.append(self.vInI)
        self.rInIhist.append(self.rInI)
    def toInertial(self, aInB, vInB):
        self.aInI = matmul(self.IB, aInB)
        self.vInI = matmul(self.IB, vInB)