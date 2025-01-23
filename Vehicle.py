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
                 velocityFunc = None):
        super().__init__(dt)
        self.betaRad =  deg2rad(targetAngleFromHorizontalDeg)
        self.IB = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                             [sin(self.betaRad),  cos(self.betaRad)]])
        
        self.rInT = matmul(self.IB.T, initState[0])
        self.vInT = array(initState[1]) # target vel in the BODY (T) frame 
        self.aInT = np.array(accel0InT) # target accel in the BODY (T) frame 
        
        self.rInI = array(initState[0]) # target pos in the inertial frame 
        self.vInI = matmul(self.IB, self.vInT)
        self.aInI = matmul(self.IB, self.aInT)
        self.dt = dt
        self.w = w
        self.t = 0
        self.velocityFunc = velocityFunc
        self.aInIHist = []
        self.vInIHist = []
        self.rInIHist = []
    def setVelocityFunc(self, velocityFunc, w):
        self.w = w
        self.velocityFunc = velocityFunc

    def update(self):
        self.t += self.dt
        betaDot = norm(self.aInT) / norm(self.vInT)
        self.betaRad = self.betaRad + betaDot*self.dt
        
        self.IB = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                            [sin(self.betaRad),   cos(self.betaRad)]])
        #update target state vars. note for const velcoty, this is
        #unecessary
        self.aInT = array([0, norm(self.aInT)])
        self.vInT = self.vInT + self.aInT*self.dt
        self.rInT = self.rInT + self.vInT*self.dt
        
        ### inertial frames via dcms
        self.aInI = matmul(self.IB, self.aInT)
        self.vInI = matmul(self.IB, self.vInT)
        self.rInI = matmul(self.IB, self.rInT)
        
        

        #compute states via incrementation / provided formulas
        accelZ = self.aInT[1]
        aInI = array([accelZ*sin(self.betaRad), 
                               accelZ*cos(self.betaRad)])
        
        vInI = self.vInI + self.aInI*self.dt
        rInI = self.rInI + self.vInI*self.dt
        ### append the dcm computed states to 
        #see if they are euqal to the states computed via incrmemnt 
        self.aInIHist.append(aInI)
        self.vInIHist.append(vInI)
        self.rInIHist.append(rInI)
        
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
        print(f"In update: self.angleFromHorizontal: {rad2deg(self.angleFromHorizontalRad)}")
        
        ## guidance will pass the accel command in the pursuer frame!!

        self.vInI = self.vInI + (self.aInI * self.dt)
        self.rInI = self.rInI + (self.vInI * self.dt)
        
        ####check that these are equalivant
    def printStates(self, rbodyName=""):
        print(f"------{rbodyName} states--------")
        print(f"θp (deg): {rad2deg(self.angleFromHorizontalRad)}")
        super().printStates()
    
