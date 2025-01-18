from RigidBody import RigidBody 
import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, dot, matmul, cross, array
from numpy.linalg import norm

class Target(RigidBody):
    def __init__(self, 
                 accel0InI,
                 targetAngleFromHorizontalDeg, 
                 initState, 
                 dt= 1/100):
        self.betaRad =  deg2rad(targetAngleFromHorizontalDeg)
        self.RinI = array(initState[0]) # target pos in the inertial frame 
        self.VinI = array(initState[1]) # target vel in the inertial frame 
        self.AinI = np.array(accel0InI) # target accel in the inertial frame 
        self.dt = dt
        
        

    def update(self):
        I2T = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                        [-sin(self.betaRad),     cos(self.betaRad)]])
        aInT = array([0, norm(self.AinI)])
        vInT = array([norm(self.VinI), 0])
        betaDot = norm(aInT) / norm(vInT)
        self.betaRad = self.betaRad + betaDot*self.dt
        accelZ = self.AinI[1]
        self.AinI = array([-accelZ*cos(self.betaRad), 
                               accelZ*sin(self.betaRad)])
        self.VinI = self.VinI + self.AinI*self.dt
        self.RinI = self.RinI + self.VinI*self.dt
    def getBetaDeg(self):
        return rad2deg(self.betaRad)
class Pursuer(RigidBody):
    def __init__(self, 
                 accel0InI,
                 pursuerAngleFromHorizontalDeg, 
                 initState, 
                 dt = 1/100):
        self.angleFromHorizontalRad = deg2rad(pursuerAngleFromHorizontalDeg)
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
    def update(self, accelCmdInI, lamda, L):
        self.AinI = accelCmdInI
        self.angleFromHorizontal = ( L + lamda)
        self.pursuer2inertialDcm = np.array([[cos(self.angleFromHorizontal), -sin(self.angleFromHorizontal)],
                                  [          sin(self.angleFromHorizontal), cos(self.angleFromHorizontal)]])
        ## guidance will pass the accel command in the pursuer frame!!

        self.VinI = self.VinI + (self.AinI * self.dt)
        self.RinI = self.RinI + (self.VinI * self.dt)
        
        ####check that these are equalivant
        #assert(np.allclose(self.AinI , AinI))



