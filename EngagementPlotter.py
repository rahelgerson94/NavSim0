import numpy as np
import debug as db

from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target
from math import asin
X = 0
Y = 1
Z = 2
R = 0
V = 1
class Guide:
    def __init__(self, pursuerObj, 
                 targetObj, 
                 HEdeg = -20, 
                 N=3, 
                 dt = 1/100,
                 aMax  = [100, 0,  100],
                 aMin = [-100, 0, -100]):
        self.aCmdInI = 0
        self.dt = dt 
        self.N = N
        self.zemInI = array([1, 1, 1])

        self.HErad = deg2rad(HEdeg)
        self.collisionLocationInI = np.inf
        self.target = targetObj
        self.pursuer = pursuerObj
        self.zemHist = []
        self.aCmdInIHist = []
        self.VcHist = []
        self.rRelInIhist = []
        self.aMax = array(aMax)
        self.aMin = array(aMin)
        ''' for this specific sim, the los angle is rotation about the y axis
        and represents the angle between the relative pos vector and the horizontal'''
        self.losAngleRad = 0
        
    
    def updateStates(self):
        
        rRelInI, vRelInI, _ = self.getRelativeStates()
        RTP = norm(rRelInI)
        self.tgo = RTP/norm(vRelInI)
        #the states are output by pursuer, target in the intertial frames
        self.zemInI  = rRelInI - vRelInI*self.tgo
        
        zemPerp = self.computeZemPlos(rRelInI)
        self.aCmdInI  = array((self.N/ (self.tgo)**2) * (zemPerp))
        print(f"[guide]: self.aCmdInI: {self.aCmdInI}")
        
        
        self.losAngleRad = arctan2(rRelInI[Z], rRelInI[X]) #TODO: compute angles about the other axes

    def computeZemPlos(self,rRelInI):
        '''
        compute the component of zem (m) thats perp. to line of sight. 
        this is equal to the zem vector in the inertial frame (self.zemInI),
        minus the  (unit length) component of self.zemInI along/parallel to the los.
        '''
        #zemPar the component of the zem vector parallel to the los
        #zemPar  = np.dot(self.zemInI, rRelInI)   / (norm(rRelInI)*norm(self.zemInI)) *rRelInI  
        zemDotRtp =  np.dot(self.zemInI, rRelInI)/ norm(rRelInI)
        zemPar = zemDotRtp* rRelInI/norm(rRelInI)
        zemPerp = self.zemInI - zemPar
        return zemPerp
    def getRelativeStates(self):
        rRelInI = self.target.rInI - self.pursuer.rInI
        vRelInI = self.target.vInI - self.pursuer.vInI
        aRelInI = self.target.aInI - self.pursuer.aInI
        return rRelInI, vRelInI, aRelInI
    def update(self, pursuer, target):
        db.enter()
        self.target = target
        self.pursuer = pursuer
        self.updateStates( )
        #self.updateDcms()

    '''
    getAcmd: return the commanded accel for this 
    timestep
    '''
    def getAcmdInI(self):
        self.clipAccel()
        return self.aCmdInI
    def clipAccel(self):
       
        if self.aCmdInI[X] > self.aMax[X]:
             self.aCmdInI[X] = self.aMax[X]
        if self.aCmdInI[Y] > self.aMax[Y]:
             self.aCmdInI[Y] = self.aMax[Y]
        if self.aCmdInI[Z] > self.aMax[Z]:
             self.aCmdInI[Z] = self.aMax[Z]
             
             
        if self.aCmdInI[X] < self.aMin[X]:
             self.aCmdInI[X] = self.aMin[X]
        if self.aCmdInI[Y] < self.aMin[Y]:
             self.aCmdInI[Y] = self.aMin[Y]
        if self.aCmdInI[Z] < self.aMin[Z]:
             self.aCmdInI[Z] = self.aMin[Z]        
    def storeStates(self):
        self.zemHist.append(self.zemInI)
        self.aCmdInIHist.append(self.aCmdInI)
        self.rRelInIhist.append(self.target.rInI - self.pursuer.rInI)
    
    def getZemInI(self):
        return self.zemInI
        
    def setCollisionPoint(self, coord):
        self.collisionLocationInI = coord
    def getLosAngleRad(self):
        return self.losAngleRad
    def printState(self):  
        
        print(f"zemInI: [", end='')
        print("  ".join(f"{elem:.2f}" for elem in self.zemInI), end='')
        print("]")
