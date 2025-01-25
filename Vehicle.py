from RigidBody import RigidBody 
import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, arccos, deg2rad, rad2deg
from numpy import zeros, dot, matmul, cross, array
from numpy.linalg import norm
import matplotlib.pyplot as plt
from EngagementPlotter import EngagementPlotter
import debug as db
class Target(RigidBody):
    def __init__(self, 
                 accel0InT,
                 targetAngleFromHorizontalDeg, 
                 initState, 
                 dt= 1/100,
                 f = 1,
                 aInIfunc = None):
        super().__init__(dt)
        self.aInB = accel0InT
        self.betaRad =  deg2rad(targetAngleFromHorizontalDeg)
        self.IB = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                             [sin(self.betaRad),  cos(self.betaRad)]])
        self.rInI = array(initState[0]) # target pos in the inertial frame 
        self.t = 0
        self.aInIfunc = aInIfunc
        if self.aInIfunc is None:
            self.toInertial(accel0InT, initState[1])
        else:
            
            self.aInI = self.aInIfunc(0)
            self.vInI = matmul(self.IB.T, initState[1])
            #self.updateStatesFromFunction()
        self.dt = dt
        self.f = f
        self.aInIHist = []
        self.vInIHist = []
        self.rInIHist = []
        self.betaDot = 0
    def update(self):
        self.t += self.dt
        betaDot = norm(self.aInI) / norm(self.vInI)
        self.betaRad = self.betaRad + betaDot*self.dt
        
        self.IB = np.array([[-cos(self.betaRad), sin(self.betaRad)],
                            [sin(self.betaRad),   cos(self.betaRad)]])
        #update target state vars. note for const velcoty, this is
        #unecessary
        if self.aInIfunc is None:
            self.aInI = matmul(self.IB, self.aInB)
            self.vInI = self.vInI + self.aInI*self.dt
            self.rInI = self.rInI + self.vInI*self.dt
        else:
            self.updateStatesFromFunction()
        self.storeStates()
    def updateStatesFromFunction(self):
        self.aInI = self.aInIfunc(self.t)
         
        # v = lambda t: array([(w)*cos(w * t), 0])/40
        # self.vInI = v(self.t)
        # r = lambda t: array([-sin(w * t), 0])/40
        # self.rInI = r(self.t)
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
        db.enter()
        self.aInI = accelCmdInI
        self.angleFromHorizontalRad = ( Lrad + lamda + self.HErad)
        #print(f"In update: self.angleFromHorizontal: {rad2deg(self.angleFromHorizontalRad)}")
        
        ## guidance will pass the accel command in the pursuer frame!!

        self.vInI = self.vInI + (self.aInI * self.dt)
        self.rInI = self.rInI + (self.vInI * self.dt)
        self.storeStates()
        ####check that these are equalivant
    def printStates(self, rbodyName=""):
        print(f"------{rbodyName} states--------")
        print(f"θp (deg): {rad2deg(self.angleFromHorizontalRad)}")
        super().printStates()
    
if __name__ ==  "__main__":
    dt = 2/1000

    betaDeg0 = 30
    betaRad0 = deg2rad(betaDeg0)
    VtMag = 2 #m/s
    at0mag = .5
    AtInT0 = at0mag*array([0,at0mag]) #in the BODY frame
    VtInT0 = VtMag*array([1,0]) #in the BODY frame
    RtInI0 = array([60, 16]) #in the INERTIAL frame
    f = 1
    w = 2*pi*f
    accelFunc = lambda t: array([-(w**2)*sin(w * t), 0])
    target = Target(AtInT0, 
                    betaDeg0, 
                    [RtInI0, VtInT0],
                    dt = dt,
                    aInIfunc= accelFunc)
    tvec = np.linspace(0, 8, int(8/dt))
    for t in tvec:
        target.update()
    ep = EngagementPlotter()
    
    ep.plotVehicleStates(tvec, target)