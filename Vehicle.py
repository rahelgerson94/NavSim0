from RigidBody import RigidBody 
import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
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
        self.betaRad =  deg2rad(targetAngleFromHorizontalDeg)
        self.T2I = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                             [sin(self.betaRad),  cos(self.betaRad)]])
        
        self.rInT = matmul(self.T2I.T, initState[0])
        self.vInT = array(initState[1]) # target vel in the BODY (T) frame 
        self.aInT = np.array(accel0InT) # target accel in the BODY (T) frame 
        
        self.rInI = array(initState[0]) # target pos in the inertial frame 
        self.vInI = matmul(self.T2I, self.vInT)
        self.aInI = matmul(self.T2I, self.aInT)
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
        
        self.T2I = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                            [sin(self.betaRad),   cos(self.betaRad)]])
        #update target state vars. note for const velcoty, this is
        #unecessary
        self.aInT = array([0, norm(self.aInT)])
        self.vInT = self.vInT + self.aInT*self.dt
        self.rInT = self.rInT + self.vInT*self.dt
        
        ### inertial frames via dcms
        self.aInI = matmul(self.T2I, self.aInT)
        self.vInI = matmul(self.T2I, self.vInT)
        self.rInI = matmul(self.T2I, self.rInT)
        
        

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
class Pursuer(RigidBody):
    def __init__(self, 
                 accel0InI,
                 pursuerAngleFromHorizontalDeg, 
                 initState, 
                 dt = 1/100):
        self.angleFromHorizontalRad = deg2rad(pursuerAngleFromHorizontalDeg)
        self.rInI = array(initState[0]) # pursuer pos in the inertial frame 
        self.vInI = array(initState[1]) # pursuer vel in the inertial frame 
        self.aInI = accel0InI           # pursuer accel in the inertial frame 
        self.dt = dt
    def integrateRk4(self, accelCmd ):
        pass
    '''
        requires, lmabda,S L, accel from guidance
        -accel: [ap_x, ap_z]
    '''
    def update(self, accelCmdInI, lamda, L):
        self.aInI = accelCmdInI
        self.angleFromHorizontal = ( L + lamda)
        self.pursuer2inertialDcm = np.array([[cos(self.angleFromHorizontal), -sin(self.angleFromHorizontal)],
                                  [          sin(self.angleFromHorizontal), cos(self.angleFromHorizontal)]])
        ## guidance will pass the accel command in the pursuer frame!!

        self.vInI = self.vInI + (self.aInI * self.dt)
        self.rInI = self.rInI + (self.vInI * self.dt)
        
        ####check that these are equalivant



