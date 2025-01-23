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
X = 0
Z = 1
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
def appendVars(p, t, g):

    ppos.append(p.rInI)
    pvel.append(p.vInI)
    paccel.append(p.aInI)
    tpos.append(t.rInI)
    tvel.append(t.vInI)
    taccel.append(t.aInI)
    losRateHist.append(g.lamdaDot)
if __name__ == "__main__":
    
    engagementPlotter = EngagementPlotter()
 
    ######### define target init orientation ########
    betaDeg0 = 30
    betaRad0 = deg2rad(betaDeg0)
    VtMag = 2 #m/s
    at0mag = .5
    AtInT0 = at0mag*array([0,at0mag]) #in the BODY frame
    VtInT0 = VtMag*array([1,0]) #in the BODY frame
    RtInI0 = array([60, 16]) #in the INERTIAL frame
    target = Target(AtInT0, 
                    betaDeg0, 
                    [RtInI0, VtInT0],
                    dt = dt)
    
    ######## define pursuer init orientation ##########
    RpInI0 = array([20, 0]) 
    VpInP0 = array([4,0])
    ApInP0 = array([0, 4])
    anglePursuerInertialDeg = 45
    
    HEdeg0 = -20
    Rrel0 = RtInI0 - RpInI0 
    pursuer = Pursuer(ApInP0, 
                      anglePursuerInertialDeg, 
                      HEdeg0 , 
                      [RpInI0, VpInP0],
                      dt = dt
                    )
    #printVeloRatio(target, pursuer)
    guide = Guide(pursuer, target, N=4, dt = dt, HEdeg = HEdeg0)
    print(f"Vt/p: {target.vInI - pursuer.vInI}")
    lamda = computeAngleBtwVecsDeg((target.rInI - pursuer.rInI), [1, 0]) 
    print(f"manual λ (deg):{lamda}")
    print(f"Guide.L (deg): {guide.getLookAngleDeg()}")
    print(f"Guide.λ (deg): {guide.getLosAngleDeg()}")
    tstart = 0
    tend = 6

    tvec = np.linspace(tstart, tend, int(tend/dt))
    tvec2 = []
    n = 0
    while (guide.getVc()) > 0:

    #for t in tvec:
        guide.update()
            
        pursuer.update(guide.getAtrueInI(), #proNav aceel in pursuer frame
                       guide.getLosAngleRad(), # line of sight angle
                       guide.getLookAngleRad()) 

        target.update()
        appendVars(pursuer, target, guide)
        if np.abs(pursuer.rInI[X] - target.rInI[X]) <=  0.1 and\
            np.abs(pursuer.rInI[Z] - target.rInI[Z]) <= 0.1:
                print("Rt/p getting small")
                
        tvec2.append(n*dt)
        n+=1
    print(f"Vc({n}): {guide.getVc()}")
    guide.setCollisionPoint(array(pursuer.rInI[X],pursuer.rInI[Z] ))
    print(f"collision at: ({pursuer.rInI[0]}, {pursuer.rInI[1]})")
    engagementPlotter.plotCollision(tpos, ppos)
    engagementPlotter.plotGuideVars(tvec[0:len(guide.lamdaDotHist)], guide)
    
    # plt.figure(3)
    # plt.plot(tvec, tpos,            label='incremented pos', linewidth=2, markersize=2)
    # plt.plot(tvec, target.rInIHist, label='xformed pos', linewidth=2, markersize=2)
    # plt.title("target pos")
    # plt.legend()
    # plt.show()
    
    
    # plt.figure(1)
    # plt.plot(tvec, tvel,            label='incremented velo', linewidth=2, markersize=2)
    # plt.plot(tvec, target.vInIHist, label='xformed velo', linewidth=2, markersize=2)
    # plt.title("target velocoty")
    # plt.legend()
    # plt.show()
    
    # plt.figure(2)
    # plt.plot(tvec, taccel,           label='incremented accel', linewidth=2, markersize=2)
    # plt.plot(tvec, target.aInIHist , label='xformed accel', linewidth=2, markersize=2)
    # plt.title("target accel")
    # plt.legend()
    # plt.show()
    
