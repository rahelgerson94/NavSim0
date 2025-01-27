import numpy as np
import debug as db

from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target
from math import asin
X = 0
Z = 1
R = 0
V = 1
class Guide:
    def __init__(self, pursuerObj, 
                 targetObj, 
                 HEdeg = -20, 
                 N=3, 
                 dt = 1/100):
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
        self.RrelHist = []
        
 
    
    def updateStates(self):
        
        rRelInI, vRelInI, _ = self.getRelativeStates()
        RTP = norm(rRelInI)
        self.tgo = RTP/norm(vRelInI)
        #the states are output by pursuer, target in the intertial frames
        self.zemInI  = rRelInI - vRelInI*self.tgo
        
        zemPerp = self.computeZemPlos(rRelInI)
        self.aCmdInI  = array((self.N/ (self.tgo)**2) * (zemPerp))
        print(f"[guide]: self.aCmdInI: {self.aCmdInI}")
        

    def computeZemPlos(self,rRelInI):
        '''
        compute the component of zem (m) thats perp. to line of sight. 
        this is equal to the zem vector in the inertial frame (self.zemInI),
        minus the  (unit length) component of self.zemInI along/parallel to the los.
        '''
        #zemPar the component of the zem vector parallel to the los
        zemPar  = (np.dot(self.zemInI, rRelInI)/ norm(rRelInI)) *rRelInI  
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
        return self.aCmdInI
    def storeStates(self):
        self.zemHist.append(self.zemInI)
        self.aCmdInIHist.append(self.aCmdInI)
    
    def getZemInI(self):
        return self.zemInI
        
    def setCollisionPoint(self, coord):
        self.collisionLocationInI = coord

    def printState(self):  
        print(f"zemInI: [", end='')
        print("  ".join(f"{elem:.2f}" for elem in self.zemInI), end='')
        print("]")
