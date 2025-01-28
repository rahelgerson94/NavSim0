import numpy as np
import matplotlib.pyplot as plt
from RigidBody import RigidBody 
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, array
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target
from Guide import Guide
from EngagementPlotter import EngagementPlotter
import debug as db
X = 0
Y = 1
Z = 2
R = 0
V = 1

dt = 2/1000
tpos = []
tvel = []
taccel = []
ppos = []
pvel = []
paccel = []
losRateHist = []
def computeAngleBtwVecsDeg(v1, v2):
    v1 = array(v1)
    v2 = array(v2)
    return np.arccos((np.dot(v1,v2))/(norm(v2)*norm(v1)))*(180/pi)
def printVeloRatio(tObj,pObj):
    Vt = tObj.vInI
    Vp = pObj.vInI
    print(f"Vt/Vp: {norm(Vt)/norm(Vp)}")

if __name__ == "__main__":
    
    engagementPlotter = EngagementPlotter()
 
    ######### define target init orientation ########

    VtMag = 2 #m/s
    at0mag = 0
    AtInT0 = at0mag*array([0,0,5]) #in the BODY frame
    VtInT0 = VtMag*array([1,0,0]) #in the BODY frame
    RtInI0 = array([60, 0, 16]) #in the INERTIAL frame
    tRotSeqDeg = [0,180+40,0]
    target = Target(AtInT0, 
                    tRotSeqDeg, 
                    [RtInI0, VtInT0],
                    dt = dt)
    target.printStates()
    #define target constant accel, to be passed to its update method 
    # and used to compute vel, pos
    aTinT = AtInT0
    ######## define pursuer init orientation ##########
    RpInI0 = array([20, 0,0]) 
    VpInP0 = array([4,0,0])
    ApInP0 = array([0, 0,0])
    
    '''
    define the purser rotation seuence, to go in 
    to computing its dcm. 
    pRotSeqDeg[0] is the rotation about the z (3rd) axis
    pRotSeqDeg[i] is the rotation about the (3-i)th axis
    '''
    
    HEdeg0 = 0
    
    rRel0 = RtInI0 - RpInI0 
    
    loasAngleRad0 = arctan2(rRel0[Z], rRel0[X])
    pRotSeqDeg = [0,30,0]
    pursuer = Pursuer(ApInP0, 
                      pRotSeqDeg, 
                      HEdeg0 , 
                      [RpInI0, VpInP0],
                      dt = dt
                    )
    pursuer.printStates()
    vRelInI0 = target.vInI - pursuer.vInI
    print(f"vt/p(0) = {vRelInI0}")
    guide = Guide(pursuer, target, N=4, dt = dt, HEdeg = HEdeg0)
    guide.printState()


    tstart = 0
    tend = 10 
    tvec = []
    
    n = 0
    NMAX = int( (tend - tstart)/dt)
    while guide.tgo > 0.1 : #and n < NMAX:
        guide.update(pursuer, target)
        guide.storeStates()
        
        pursuer.update([0, 0, 0], guide.getAcmdInI())  
        pursuer.storeStates()
        
        target.update([0,0,0], aTinT)
        target.storeStates()
        
        if np.abs(pursuer.rInI[X] - target.rInI[X]) <=  0.1 and\
            np.abs(pursuer.rInI[Y] - target.rInI[Z]) <= 0.1 and\
            np.abs(pursuer.rInI[Z] - target.rInI[Z]):
                print("Rt/p getting small")

        n+=1
        tvec.append(n*dt)
    print(f"guide.tgo: {guide.tgo}")
    ep = EngagementPlotter()
    #ep.plotVehiclesSubplots(tvec[0:n], pursuer, target)
    # ep.plotVehicleStatesSubplots(tvec[0:n], pursuer, "pursuer", figNum = 0)
    # ep.plotVehicleStatesSubplots(tvec[0:n], target, "target", figNum = 1)
    print(f"zem[{n}]: {guide.getZemInI()} ")
    guide.setCollisionPoint(array([pursuer.rInI[X],\
                                    pursuer.rInI[Y],\
                                    pursuer.rInI[Z]] )
                            )
    tvec = tvec[0:n]
    print(f"collision at: ({pursuer.rInI[0]}, {pursuer.rInI[1]})")
    
    engagementPlotter.plotCollision3d( target, pursuer, tvec = tvec)
    
    engagementPlotter.plotStateProjections(tvec, target, pursuer,  state='r', overlayed=True)
    
    
