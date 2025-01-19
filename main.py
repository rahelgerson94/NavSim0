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

    ppos.append(p.RinI)
    pvel.append(p.VinI)
    paccel.append(p.AinI)
    tpos.append(t.RinI)
    tvel.append(t.VinI)
    taccel.append(t.AinI)
    losRateHist.append(g.lamdaDot)
if __name__ == "__main__":
    
    engagementPlotter = EngagementPlotter()
 
    ######### define target init orientation ########
    betaDeg0 = 30
    betaRad0 = deg2rad(betaDeg0)
    VtMag = 2 #m/s
    at0mag = 1/2
    T2Idcm = array([-cos(betaRad0),  #TODO: implement different, less ideal angles from pursuer
                        sin(betaRad0)])
    at0 = at0mag*T2Idcm
    Vt0 = VtMag* T2Idcm
    Rt0 = array([60, 16])
    target = Target(at0, 
                    betaDeg0, 
                    [Rt0, Vt0],
                    dt = dt)
    ######## define pursuer init orientation ##########
    Rp0 = array([20, 0]) 
    VpMag = 5 #m/s
    Rrel0 = Rt0 - Rp0
    HEdeg0 = -20
    
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
    guide = Guide(N=3, dt = dt)

    tstart = 0
    tend = 8
    dt  = 2/1000

    tvec = np.linspace(tstart, tend, int(tend/dt))
    tvec = []
    n = 0
    while guide.getVcInI() >= 0:
        guide.update(pursuer,target)
        
        pursuer.update(guide.getApureInI(), #proNav aceel in pursuer frame
                       guide.getLosAngleRad(), # line of sight angle
                       guide.getLookAngleRad()) 

        target.update()
        appendVars(pursuer, target, guide)
        if np.abs(pursuer.RinI[X] - target.RinI[X]) <=  0.1 and\
            np.abs(pursuer.RinI[Z] - target.RinI[Z]) <= 0.1:
                print("Rt/p getting small")
                
        tvec.append(n*dt)
        n+=1
    guide.setCollisionPoint(array(pursuer.RinI[X],pursuer.RinI[Z] ))
    print(f"collision at: ({pursuer.RinI[0]}, {pursuer.RinI[1]})")
    engagementPlotter.plotCollision(tpos, ppos)
    engagementPlotter.plotGuideVars(tvec[0:len(guide.lamdaDotHist)], guide)
    

    
    
    
        
