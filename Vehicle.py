from RigidBody import RigidBody 
import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, arccos, deg2rad, rad2deg
from numpy import zeros, dot, matmul, cross, array
from numpy.linalg import norm


class Target(RigidBody):
    def __init__(self, 
                 accel0InT,
                 targetAngleFromHorizontalDeg, 
                 initState, 
                 dt= 1/100,
                 w = 1,
                 aInIfunc = None, 
                 vInIfunc = None,
                 rInIfunc = None):
        super().__init__(dt)
        self.aInB = accel0InT
        self.betaRad =  deg2rad(targetAngleFromHorizontalDeg)
        self.IB = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                             [sin(self.betaRad),  cos(self.betaRad)]])
        self.rInI = array(initState[0]) # target pos in the inertial frame 
        if aInIfunc is None:
            self.toInertial(accel0InT, initState[1])
        
        self.dt = dt
        self.w = w
        self.t = 0
        self.aInIHist = []
        self.vInIHist = []
        self.rInIHist = []
        self.aInIfunc = aInIfunc
        self.vInIFunc = vInIfunc
        self.rInIFunc = rInIfunc
    def setVelocityFunc(self, velocityFunc, w):
        self.w = w
        self.vInIFunc = velocityFunc

    def update(self):
        self.t += self.dt
        betaDot = norm(self.aInI) / norm(self.vInI)
        self.betaRad = self.betaRad + betaDot*self.dt
        
        self.IB = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                            [sin(self.betaRad),   cos(self.betaRad)]])
        #update target state vars. note for const velcoty, this is
        #unecessary
        
        self.aInI = matmul(self.IB, self.aInB)
        self.vInI = self.vInI + self.aInI*self.dt
        self.rInI = self.rInI + self.vInI*self.dt
        self.storeStates()
        
    def getBetaDeg(self):
        return rad2deg(self.betaRad)
    def printStates(self, rbodyName=""):
        print(f"------{rbodyName} states--------")
        print(f"β (deg): {self.getBetaDeg()}")
        super().printStates()
        
class Pursuer(RigidBody):
    def __init__(self, 
                 accel0InP,
                 pursuerAngleFromHorizontalDeg, 
                 HEdeg,
                 initState, 
                 dt = 1/100):
        super().__init__(dt)
        self.angleFromHorizontalRad = deg2rad(pursuerAngleFromHorizontalDeg) + deg2rad(HEdeg)
        print(f"In c-tor: self.angleFromHorizontal: {rad2deg(self.angleFromHorizontalRad)}")
        self.IB = np.array([[cos(self.angleFromHorizontalRad), -sin(self.angleFromHorizontalRad)],
                            [sin(self.angleFromHorizontalRad), cos(self.angleFromHorizontalRad)]])
        self.rInI = array(initState[0]) # pursuer pos in the inertial frame 
        self.toInertial(accel0InP, initState[1])
        self.dt = dt
        self.HErad = deg2rad(HEdeg)
    def integrateRk4(self, accelCmd ):
        pass
    '''
        requires, lmabda,S L, accel from guidance
        -accel: [ap_x, ap_z]
    '''
    def update(self, accelCmdInI, lamda, Lrad):
        self.aInI = accelCmdInI
        self.angleFromHorizontalRad = ( Lrad + lamda + self.HErad)
        #print(f"In update: self.angleFromHorizontal: {rad2deg(self.angleFromHorizontalRad)}")
        
        ## guidance will pass the accel command in the pursuer frame!!

        self.vInI = self.vInI + (self.aInI * self.dt)
        self.rInI = self.rInI + (self.vInI * self.dt)
        
        ####check that these are equalivant
    def printStates(self, rbodyName=""):
        print(f"------{rbodyName} states--------")
        print(f"θp (deg): {rad2deg(self.angleFromHorizontalRad)}")
        super().printStates()
    
