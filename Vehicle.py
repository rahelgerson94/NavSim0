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
                 rotationSequenceDeg, 
                 initState, 
                 dt= 1/100):
        
        '''
        super().__init__ stores the sampling period and computes 
        the body2inertial dcm from rotationSequence
        '''
        super().__init__(initState[0],
                         initState[1],
                         accel0InT,
                         rotationSequence=rotationSequenceDeg,
                         dt=dt )
        
        self.t = 0
        
        
    def update(self, rotationSeqeunce, aInB):
        self.t += self.dt
        super().update(rotationSeqeunce, aInB)
        
        

        
    def printStates(self, rbodyName="target"):
        print(f"------{rbodyName} states--------")
       
        super().printStates()
        
class Pursuer(RigidBody):
    def __init__(self, 
                 accel0InP,
                 rotationSequenceDeg, 
                 HEdeg,
                 initState, 
                 dt = 1/100):
        rotationSequenceDeg = rotationSequenceDeg +  array([HEdeg, 0, 0])  
        super().__init__(initState[0],
                         initState[1],
                         accel0InP,
                         rotationSequence=rotationSequenceDeg,
                         dt=dt )
        self.HErad = deg2rad(HEdeg)
    def integrateRk4(self, accelCmd ):
        pass
    '''
        requires, lmabda,S L, accel from guidance
        -accel: [ap_x, ap_z]
    '''
    def update(self, rotationSeq, accelCmdInI):
        self.updateIB(rotationSeq)
                 
        self.aInI = accelCmdInI
        self.vInI = self.vInI + self.aInI*self.dt
        self.rInI = self.rInI + self.vInI*self.dt
        
        
        ####check that these are equalivant
    def printStates(self, rbodyName="pursuer"):
        print(f"------{rbodyName} states--------")
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
                    dt = dt)
    tvec = np.linspace(0, 8, int(8/dt))
    for t in tvec:
        target.update()
    ep = EngagementPlotter()
    
    ep.plotVehicleStates(tvec, target)
