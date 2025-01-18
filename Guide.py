import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target

class Guide:
    def __init__(self, N=3, dt = 1/100):
        self.Lrad = 0 #look angle
        self.lamdaRad = 0 # line of sight angle 
        self.lamdaDot = 0
        self.VcInI = 100000
        self.dt = dt 
        self.N = N
        self.aPureInLos = 0
        self.aPureinP = 0
        self.lamdaDotHist = []
        self.lamdaHist = []
        self.VcInIhist = []
        self.lookAngleHist = []
        
        self.RrelHist = []

    def updateDcms(self,pursuerObj, targetObj ):
         ### update the DCMs ##
        angleBtwPursuerLosRad = self.Lrad 
        self.los2pursuerDcm = np.array([[cos(angleBtwPursuerLosRad), sin(angleBtwPursuerLosRad)],
                                  [     -sin(angleBtwPursuerLosRad),     cos(angleBtwPursuerLosRad)]])
        #print(f"los2pDcm.shape: {self.los2pursuerDcm.shape}")
        self.los2inertialDcm = np.array([[cos(self.lamdaRad), sin(self.lamdaRad)],
                                  [      -sin(self.lamdaRad), cos(self.lamdaRad)]])
        #print(f"los2inertialDcm.shape: {self.los2inertialDcm.shape}")
        angleBtwPursuerInertialRad = self.Lrad + self.lamdaRad
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
        self.lamdaDot = (Rrel[0]*Vrel[1] - Rrel[1]*Vrel[0]) / RrelNorm**2
        arg1 = sin(targetObj.betaRad + self.lamdaRad)*norm(Vt)/norm(Vp)
        self.Lrad = arcsin(arg1) 

        
        
        self.VcInI = -(Rrel[0] * Vrel[0] - (Rrel[1] * Vrel[1]) )  / RrelNorm
        
        
        self.aPureInLos = array([ 0,  self.N*self.lamdaDot*norm(self.VcInI)])
        self.appendVars(Rrel)
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
    def getApureInI(self):
        #return matmul(self.los2pursuerDcm, self.aPureInLos)
        accelZ = self.aPureInLos[1]
        return array([-accelZ*sin(self.lamdaRad),
                      accelZ*cos(self.lamdaRad)])
    def getLosAngleRad(self):
        return self.lamdaRad
    def getLosAngleDeg(self):
        return rad2deg(self.lamdaRad)
    def getLookAngleRad(self):
        return self.Lrad
    def getLookAngleDeg(self):
        return rad2deg(self.L)
    def getVcInI(self):
        return self.VcInI
    def appendVars(self, Rrel):
        self.lamdaDotHist.append(rad2deg(self.lamdaDot))
        self.lamdaHist.append(rad2deg(self.lamdaRad))
        self.VcInIhist.append(self.VcInI)
        self.lookAngleHist.append(rad2deg(self.Lrad))
        
        self.RrelHist.append(Rrel)