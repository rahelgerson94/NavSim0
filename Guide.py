import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad
from numpy import zeros, matmul, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target

class Guide:
    def __init__(self, N=3, dt = 1/100):
        self.Lrad = 0 #look angle
        self.lamdaRad = 0 # line of sight angle 
        self.lamdaFromDeRad = 0
        self.VcInI = 0
        self.dt = dt 
        self.N = N
        self.aPureInLos = 0
        self.aPureinP = 0
    def updateDcms(self,pursuerObj, targetObj ):
         ### update the DCMs ##
        angleBtwPursuerLosRad = self.Lrad + pursuerObj.HErad
        self.los2pursuerDcm = np.array([[cos(angleBtwPursuerLosRad), sin(angleBtwPursuerLosRad)],
                                  [     -sin(angleBtwPursuerLosRad),     cos(angleBtwPursuerLosRad)]])
        #print(f"los2pDcm.shape: {self.los2pursuerDcm.shape}")
        self.los2inertialDcm = np.array([[cos(self.lamdaRad), sin(self.lamdaRad)],
                                  [      -sin(self.lamdaRad), cos(self.lamdaRad)]])
        #print(f"los2inertialDcm.shape: {self.los2inertialDcm.shape}")
        angleBtwPursuerInertialRad = pursuerObj.HErad + self.Lrad + self.lamdaFromDeRad
        self.pursuer2inertialDcm = np.array([[cos(angleBtwPursuerInertialRad), sin(angleBtwPursuerInertialRad)],
                                  [          -sin(angleBtwPursuerInertialRad), cos(angleBtwPursuerInertialRad)]])
        #print(f"pursuer2inertialDcm.shape: {self.pursuer2inertialDcm.shape}")
    def updateStates(self, pursuerObj, targetObj):

        #the states are output by pursuer, target in the intertial frames
        Vt = targetObj.VinI
        Vp = pursuerObj.VinI
        Rt = targetObj.RinI
        Rp = pursuerObj.RinI 


        Rrel = Rt - Rp  
        Vrel = Vt - Vp  
        RrelNorm = norm(Rrel)
        
        self.lamdaRad = np.arctan2(Rrel[1],Rrel[0])
        lamdaDot = (Rrel[0]*Vrel[1] - Rrel[1]*Vrel[0]) / RrelNorm**2
        self.lamdaFromDeRad = self.lamdaFromDeRad + lamdaDot*self.dt
        arg1 = sin(targetObj.betaRad + self.lamdaFromDeRad)*norm(Vt)/norm(Vp)
        self.L = arcsin(arg1) - pursuerObj.HErad

        
        
        self.VcInI = -(Rrel[0] * Vrel[0] - (Rrel[1] * Vrel[1]) )  / RrelNorm
        
        self.aPureInLos = array([ 0,  self.N*lamdaDot*norm(self.VcInI)])
        
    def update(self,  pursuerObj, targetObj):
        self.updateStates( pursuerObj, targetObj)
        self.updateDcms( pursuerObj, targetObj)
    
    '''
    compute ProNav accel in the los frame
    '''
    def getApureInLos(self):
        return self.aPureInLos
    '''
    provide the ProNav accel in the pursuer frame 
    '''
    def getApureInP(self):
        return matmul(self.los2pursuerDcm, self.aPureInLos)
    def getLosAngle(self):
        return self.lamdaFromDeRad
    def getLookAngle(self):
        return self.L
    