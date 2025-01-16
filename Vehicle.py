from RigidBody import RigidBody 
import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad
from numpy import zeros, dot, matmul, cross, array
from numpy.linalg import norm

class Target(RigidBody):
    def __init__(self, 
                 accel0InI,
                 targetAngleFromHorizontalDeg, 
                 initState, 
                 dt= 1/100):
        self.betaRad = np.pi - deg2rad(targetAngleFromHorizontalDeg)
        self.RinI = array(initState[0]) # target pos in the inertial frame 
        self.VinI = array(initState[1]) # target vel in the inertial frame 
        self.AinI = np.array(accel0InI) # target accel in the inertial frame 
        self.dt = dt
        
        

    def update(self):
        betaDot = self.AinI[0]/self.VinI[0]
        self.betaRad = self.betaRad + betaDot*self.dt
        accelZ = self.AinI[1]
        self.AinI = array([accelZ*sin(self.betaRad), 
                               accelZ*cos(self.betaRad)])
        self.VinI = self.VinI + self.AinI*self.dt
        self.RinI = self.RinI + self.VinI*self.dt

class Pursuer(RigidBody):
    def __init__(self, 
                 accel0InI,
                 pursuerAngleFromHorizontalDeg, 
                 HEdeg0,
                 initState, 
                 dt = 1/100):
        self.angleFromHorizontalRad = deg2rad(pursuerAngleFromHorizontalDeg)
        self.HErad = deg2rad(HEdeg0)
        self.RinI = array(initState[0]) # pursuer pos in the inertial frame 
        self.VinI = array(initState[1]) # pursuer vel in the inertial frame 
        self.AinI = accel0InI           # pursuer accel in the inertial frame 
        self.dt = dt
    def integrateRk4(self, accelCmd ):
        pass
    '''
        requires, lmabda,S L, accel from guidance
        -accel: [ap_x, ap_z]
    '''
    def update(self, accelCmdInP, lamda, L):
        self.accelCmdInP = accelCmdInP
        self.angleFromHorizontal = (self.HErad + L + lamda)
        self.pursuer2inertialDcm = np.array([[cos(self.angleFromHorizontal), -sin(self.angleFromHorizontal)],
                                  [          sin(self.angleFromHorizontal), cos(self.angleFromHorizontal)]])
        ## guidance will pass the accel command in the pursuer frame!!

        accelCmdZ = array([0, accelCmdInP[1]])
        self.AinI = dot(accelCmdZ, self.pursuer2inertialDcm )
        AinI = matmul(self.pursuer2inertialDcm , accelCmdZ)

        self.VinI = self.VinI + (self.AinI * self.dt)
        self.RinI = self.RinI + (self.VinI * self.dt)
        
        ####check that these are equalivant
        #assert(np.allclose(self.AinI , AinI))



