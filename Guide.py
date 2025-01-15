import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad
from numpy import zeros, matmul, inv
from numpy.linalg import norm
from Vehicle import Pursuer
from Vehicle import Target

class Guide:
    def __init__(self, N=3, dt = 1/100):
        self.L = 0 #look angle
        self.lamda = 0 # line of sight angle 
        self.lamdaDot = 0
        self.Vc = 0
        self.dt = dt 
        self.N = N
        self.aPureExLos = 0
        self.aPureinP = 0
    def updateDcms(self):
         ### update the DCMs ###
        anglePurserLos = np.deg2rad(self.L + pursuerObj.HE)
        self.los2purserDcm = np.array([[cos(anglePurserLos), sin(anglePurserLos)],
                                  [-sin(anglePurserLos), cos(anglePurserLos)]])
        
        lamdaRad = np.deg2rad(self.lamda)
        self.los2inertialDcm = np.array([[cos(lamdaRad), sin(lamdaRad)],
                                  [      -sin(lamdaRad), cos(lamdaRad)]])
        
        anglePursuerInertialRad = deg2rad(pursuerObj.HE + self.L + self.lamda)
        self.pursuer2inertialDcm = np.array([[cos(anglePursuerInertialRad), sin(anglePursuerInertialRad)],
                                  [          -sin(anglePursuerInertialRad), cos(anglePursuerInertialRad)]])
        
    def updateStates(self, pursuerObj, targetObj):

        #the states are output by purser, target in the intertial frames
        Vt = targetObj.V
        Vp = pursuerObj.V
        Rt = targetObj.R
        Rp = pursuerObj.R 


        Rrel = Rt - Rp  
        Vrel = Vt - Vp  

        self.lamda = arctan2(Rrel[1]/Rrel[0])
        arg1 = sin(deg2rad(targetObj.beta + pursuerObj.lamda))*Vt/Vp
        self.L = arcsin(arg1) - pursuerObj.HE

        RrelNorm = norm(Rrel)
        
        self.Vc = -(Rrel[0] * Vrel[0] - (Rrel[1] * Vrel[1]) )  / RrelNorm
        self.lambdaDot = (Rrel[0] * Vrel[1] - (Rrel[0] * Vrel[1])) /  RrelNorm**2
        self.aPureInLos = self.N*self.lamdaDot*self.Vc
        
    def update(self):
        self.updateStates()
        self.updateDcms()
    
    '''
    compute ProNav accel in the los frame
    '''
    def getApureInLos(self):
        return self.N*self.lamdaDot*self.Vc
    '''
    provide the ProNav accel in the pursuer frame 
    '''
    def getApureInP(self):
        return matmul(self.los2purserDcm, self.aPureInLos)
    def getLosAngle(self):
        return self.los
    def getLookAngle(self):
        return self.L