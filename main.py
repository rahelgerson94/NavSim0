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
    at0mag = 0
    AtInT0 = at0mag*array([0,at0mag]) #in the BODY frame
    VtInT0 = VtMag*array([VtMag,0]) #in the BODY frame
    RtInI0 = array([60, 16]) #in the INERTIAL frame
    target = Target(AtInT0, 
                    betaDeg0, 
                    [RtInI0, VtInT0],
                    dt = dt)
    
    ######## define pursuer init orientation ##########
    Rp0 = array([20, 0]) 
    VpMag = 5 #m/s
    Rrel0 = RtInI0 - Rp0
    HEdeg0 = -40
    
    lamdaRad0 = arctan2(Rrel0[1],Rrel0[0])
    arg1 = sin(betaRad0 + lamdaRad0) * VtMag/VpMag
    L0 = arcsin(arg1) 
    pursuerAngleFromHorizontalRad = L0 + lamdaRad0 
    P2Idcm = array([-sin(lamdaRad0), cos(lamdaRad0)]) 
    ap0mag = 1
    ap0 = ap0mag*P2Idcm
    Vp0 = VpMag* array([cos(pursuerAngleFromHorizontalRad + deg2rad(HEdeg0)), 
                        sin(pursuerAngleFromHorizontalRad + deg2rad(HEdeg0))])
    
    pursuer = Pursuer(ap0, 
                      rad2deg(pursuerAngleFromHorizontalRad), 
                      [Rp0, Vp0],
                      dt = dt
                    )
    guide = Guide(N=4, dt = dt)

    tstart = 0
    tend = 8

    tvec = np.linspace(tstart, tend, int(tend/dt))
    tvec = []
    n = 0
    while guide.getVc() >= 0:
        guide.update(pursuer,target)
        
        pursuer.update(guide.getApureInI(), #proNav aceel in pursuer frame
                       guide.getLosAngleRad(), # line of sight angle
                       guide.getLookAngleRad()) 

        target.update()
        appendVars(pursuer, target, guide)
        if np.abs(pursuer.rInI[X] - target.rInI[X]) <=  0.1 and\
            np.abs(pursuer.rInI[Z] - target.rInI[Z]) <= 0.1:
                print("Rt/p getting small")
                
        tvec.append(n*dt)
        n+=1
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
    