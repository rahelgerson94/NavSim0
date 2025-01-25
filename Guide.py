import numpy as np
import debug as db

from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target
from math import asin
from scipy.optimize import fsolve #to compute tgo using implicit formual

X = 0
Z = 1
R = 0
V = 1
class Guide:
    def __init__(self, pursuerObj, 
                 targetObj, 
                 HEdeg = -20, 
                 N=3, 
                 dt = 1/100, 
                 mode = 'pure'):
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
        self.Rrel = targetObj.rInI - pursuerObj.rInI
        self.target = targetObj
        self.pursuer = pursuerObj
        
        self.update()
        # self.target.printStates("target")
        # self.pursuer.printStates("pursuer")
        Vrel = targetObj.vInI - pursuerObj.vInI
        self.tgo = self.Rrel[Z]/ Vrel[Z]
    def updateDcms(self ):
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
    def computeRelativeStates(self):
        aRelInI = self.target.aInI - self.pursuer.aInI
        vRelInI = self.target.vInI - self.pursuer.vInI
        rRelInI = self.target.rInI - self.pursuer.rInI
        return rRelInI, vRelInI, aRelInI
    def computeTgo(self):
        rRelInI, vRelInI, aRelInI = self.computeRelativeStates()
        num = norm(rRelInI)
         
        denom = norm()
    def updateStates(self):
        
        #the states are output by pursuer, target in the intertial frames
        Vt = self.target.vInI
        Vp = self.pursuer.vInI
        Rt = self.target.rInI
        Rp = self.pursuer.rInI 

        self.Rrel = Rt - Rp
        Rrel, _, _ = self.computeRelativeStates()
        Vrel = Vt - Vp  
        RrelNorm = norm(Rrel)
        
        self.lamdaRad = np.arctan2(Rrel[Z],Rrel[X])
        self.lamdaDot = (Rrel[X]*Vrel[Z] - Rrel[Z]*Vrel[X]) / RrelNorm**2
        betaLamda = (self.target.betaRad + self.lamdaRad)
        VT = norm(Vt)
        VP = norm(Vp)
        try:
            self.Lrad = asin(VT/VP*sin(betaLamda))  
        except ValueError as e:
            print(f"Error: {e}")
            self.printState()
        self.Vc = (-(self.Rrel[X] * Vrel[X]) - (self.Rrel[Z] * Vrel[Z]) )  / RrelNorm
    
        self.aTrueMag =  self.N*self.lamdaDot * self.Vc
        self.aPureMag =  self.N*self.lamdaDot  * VP
        self.storeStates()
        # self.target.printStates("target")
        # self.pursuer.printStates("pursuer")
    def update(self):
        db.enter()
        self.updateStates( )
        self.updateDcms()
    
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
    def getAtrueZemInI(self):
        self.tgo
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
    def storeStates(self):
        self.lamdaDotHist.append(rad2deg(self.lamdaDot))
        self.lamdaHist.append(rad2deg(self.lamdaRad))
        self.VcHist.append(self.Vc)
        self.lookAngleHist.append(rad2deg(self.Lrad))
        
        self.RrelHist.append(self.Rrel)
    def setCollisionPoint(self, coord):
        self.collisionLocationInI = coord
    def getRtp(self):
        return self.Rrel
    def printState(self):
            
            print(f"Guide.L (deg): {self.getLookAngleDeg()}")
            print(f"Guide.λ (deg): {self.getLosAngleDeg()}")
            print(f"target.β (deg): {self.target.getBetaDeg()}")
            print(f"|Vt|: {norm(self.target.vInI)}")
            print(f"|Vp|: {norm(self.pursuer.vInI)}")
            print(f"sin(λ+β)*VT/VP: {VT/VP*sin(betaLamda)}")