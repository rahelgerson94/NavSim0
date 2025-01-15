from RigidBody import RigidBody 
import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad
from numpy import zeros, dot, matmul,cross
from numpy.linalg import norm

class Target(RigidBody):
    def __init__(self, accel0, beta0, initState, dt= 1/100):
        super().__init(dt)
        self.beta = beta0
        self.RwrtIwrtI = initState[0]
        self.VwrtIwrtI = initState[1]
        self.accel = accel0
        self.dt = dt
        self.accelCmdinB = 0

    def update(self):
        betaDot = self.accel[0]/self.VwrtI[0]
        self.beta = self.beta + self.betaDot*self.dt
    def toInertial(self):
        self.accel = self.accel

class Pursuer(RigidBody):
    def __init__(self, accel0, HE0, gamma0, initState, dt = 1/100):
        super().__init(dt)
        self.gamma = gamma0
        self.HE = HE0
        self.RwrtI = initState[0]
        self.VwrtI = initState[1]
        self.aWrtI = (self.VwrtI - np.zeros(2))/self.dt
    def integrate(self, accelCmd, ):
        pass
    '''
        requires, lmabda, L, accel from guidance
        -accel: [ap_x, ap_z]
    '''
    def update(self, lamda, L, accelCmdInP):
        self.L = L 
        self.lamda = lamda 
        self.accelCmdInP = accelCmd
        anglePursuerInertialRad = deg2rad(self.HE + self.L + self.lamda)
        self.pursuer2inertialDcm = np.array([[cos(anglePursuerInertialRad), -sin(anglePursuerInertialRad)],
                                  [          sin(anglePursuerInertialRad), cos(anglePursuerInertialRad)]])
        ## guidance will pass the accel command in the pursuer frame!!

        accelCmdZ = array([0, accelCmd[1]])
        self.aWrtI = dot(accelCmdZ, self.pursuer2inertialDcm )
        aWrtI = matmul(self.pursuer2inertialDcm , accelCmdZ)

        self.VwrtIwrtI = self.VwrtI + (self.aWrtI * dt)
        self.RwrtIwrtI = self.RwrtI + (self.VwrtIWrtI * dt)

        ####check that these are equalivant
        az = accelCmd[1]
        VwrtI = np.array([[-az*sin(anglePursuerInertialRad)],
                          [az*cos(anglePursuerInertialRad)]])
        np.assert_allclose(self.VwrtIwrtI , VwrtI)



