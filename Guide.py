import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target
X = 0
Z = 1
R = 0
V = 1
class Guide:
    def __init__(self, HEdeg = -20, N=3, dt = 1/100, mode = 'pure'):
        self.Lrad = 0 #look angle
        self.lamdaRad = 0 # line of sight angle 
        self.lamdaDot = 0
        self.Vc = 100000
        self.dt = dt 
        self.N = N
        self.aTrueInLos = 0
        self.aPureMag = 0
        self.aTrueMag = 0
        self.lamdaDotHist = []
        self.lamdaHist = []
        self.VcHist = []
        self.lookAngleHist = []
        self.RrelHist = []
        self.HErad = deg2rad(HEdeg)
        self.collisionLocationInI = np.inf
        self.Rrel = 0
    def updateDcms(self,pursuerObj, targetObj ):
         ### update the DCMs ##
        angleBtwPursuerLosRad = self.Lrad + self.HErad
        self.pursuer2los = np.array([[cos(angleBtwPursuerLosRad), -sin(angleBtwPursuerLosRad)],
                                  [ sin(angleBtwPursuerLosRad),     cos(angleBtwPursuerLosRad)]])
        #print(f"los2pDcm.shape: {self.los2pursuerDcm.shape}")
        self.los2inertialDcm = np.array([[cos(self.lamdaRad), sin(self.lamdaRad)],
                                  [      sin(self.lamdaRad), cos(self.lamdaRad)]])
        #print(f"los2inertialDcm.shape: {self.los2inertialDcm.shape}")
        angleBtwPursuerInertialRad = self.Lrad + self.lamdaRad
        self.pursuer2inertialDcm = np.array([[cos(angleBtwPursuerInertialRad), -sin(angleBtwPursuerInertialRad)],
                                  [          sin(angleBtwPursuerInertialRad), cos(angleBtwPursuerInertialRad)]])
        #print(f"pursuer2inertialDcm.shape: {self.pursuer2inertialDcm.shape}")
    def updateStates(self, pursuerObj, targetObj):

        #the states are output by pursuer, target in the intertial frames
        Vt = targetObj.vInI
        Vp = pursuerObj.vInI
        Rt = targetObj.rInI
        Rp = pursuerObj.rInI 

        self.Rrel = Rt - Rp
        
        Vrel = Vt - Vp  
        RrelNorm = norm(self.Rrel)
        
        self.lamdaRad = np.arctan2(self.Rrel[Z],self.Rrel[X])
        self.lamdaDot = (self.Rrel[X]*Vrel[Z] - self.Rrel[Z]*Vrel[X]) / RrelNorm**2
        arg1 = sin(targetObj.betaRad + self.lamdaRad)*norm(Vt)/norm(Vp)
        self.Lrad = arcsin(arg1) 

        
        
        self.Vc = -(self.Rrel[X] * Vrel[X] - (self.Rrel[Z] * Vrel[Z]) )  / RrelNorm
        
        
        self.aTrueMag =  self.N*self.lamdaDot*norm(self.Vc) 
        self.aPureMag = self.N*self.lamdaDot*Vp
        self.appendVars()
    def update(self,  pursuerObj, targetObj):
        self.updateStates( pursuerObj, targetObj)
        self.updateDcms( pursuerObj, targetObj)
    
    '''
    compute ProNav accel in the los frame
    '''
    def getAtrueInLos(self):
        return array([0, self.aTrueMag])
    '''
    provide the ProNav accel in the pursuer frame 
    '''
    def getAtrueInI(self):
        #return matmul(self.los2pursuerDcm, self.aTrueInLos)
        
        aTrueInP = matmul(self.pursuer2los.T, array([0, self.aTrueMag]))
        angleLI = self.lamdaRad  #angle between los and inertial
        return aTrueInP[Z]*array([-sin(angleLI),
                                cos(angleLI)])
    def getApureMag(self):
        #return matmul(self.los2pursuerDcm, self.aPureInLos)
        return self.aPureMag
    def getApureInI(self):
        angleBtwPursuerInertialRad = self.Lrad + self.lamdaRad + self.HErad
        
        return self.aPureMag *\
            array([-sin(angleBtwPursuerInertialRad),\
                   cos(angleBtwPursuerInertialRad)])
    def getLosAngleRad(self):
        return self.lamdaRad
    def getLosAngleDeg(self):
        return rad2deg(self.lamdaRad)
    def getLookAngleRad(self):
        return self.Lrad
    def getLookAngleDeg(self):
        return rad2deg(self.Lrad)
    def getVc(self):
        return self.Vc
    def appendVars(self):
        self.lamdaDotHist.append(rad2deg(self.lamdaDot))
        self.lamdaHist.append(rad2deg(self.lamdaRad))
        self.VcHist.append(self.Vc)
        self.lookAngleHist.append(rad2deg(self.Lrad))
        
        self.RrelHist.append(self.Rrel)
    def setCollisionPoint(self, coord):
        self.collisionLocationInI = coord
    def getRtp(self):
        return self.Rrel