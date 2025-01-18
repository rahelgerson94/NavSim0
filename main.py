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
dt = 1/100
tpos = []
tvel = []
ppos = []
pvel = []
paccel = []
lamdaThist = []
betaThist = []
RrelThist = []
vRelHist = []
vrelHist = []
losRateHist = []
def appendVars(p, t, g):
    betaThist.append(t.betaRad)
    lamdaThist.append(guide.lamdaRad)
    ppos.append(p.RinI)
    pvel.append(p.VinI)
    paccel.append(p.AinI)
    tpos.append(t.RinI)
    tvel.append(t.VinI)
    RrelThist.append(t.RinI - p.RinI)
    vrelHist.append(g.VcInI)
    losRateHist.append(g.lamdaDot)
if __name__ == "__main__":
    
    engagementPlotter = EngagementPlotter()
 
    ######### define target init orientation ########
    betaDeg0 = 110
    betaRad0 = deg2rad(betaDeg0)
    VtMag = 5 #m/s
    
    at0 = array([0, 0])
    Vt0 = VtMag* array([cos(betaRad0),  #TODO: implement different, less ideal angles from pursuer
                        sin(betaRad0)])
    Rt0 = array([60, 16])
    target = Target(at0, 
                    betaDeg0, 
                    [Rt0, Vt0])
    ######## define pursuer init orientation ##########
    Rp0 = array([20, 0]) 
    VpMag = 7 #m/s
    Rrel0 = Rt0 - Rp0
    HEdeg0 = -10
    
    lamdaRad0 = arctan2(Rrel0[1],Rrel0[0])
    arg1 = sin(betaRad0 + lamdaRad0) * VtMag/VpMag
    L0 = arcsin(arg1) 
    pursuerAngleFromHorizontalRad = L0 + lamdaRad0 

    
    ap0 = array([0, 0]) 
    Vp0 = VpMag* array([cos(pursuerAngleFromHorizontalRad + deg2rad(HEdeg0)), 
                        sin(pursuerAngleFromHorizontalRad + deg2rad(HEdeg0))])
    
    pursuer = Pursuer(ap0, 
                      rad2deg(pursuerAngleFromHorizontalRad), 
                      [Rp0, Vp0]
                    )
    guide = Guide()

    tstart = 0
    tend = 100
    dt = .001

    tvec = np.linspace(tstart, tend, int(tend/dt))
    # tvec = []
    # idx = 0
    for i in tvec:
        guide.update(pursuer,target)
        
        pursuer.update(guide.getApureInI(), #proNav aceel in pursuer frame
                       guide.getLosAngleRad(), # line of sight angle
                       guide.getLookAngleRad()) 
        target.update()
        appendVars(pursuer, target, guide)
    engagementPlotter.plotCollision(ppos, tpos)
    missIdx =  np.argmin(np.abs(RrelThist))
    h = dt
    dtIdx = int(.1/h)
    plt.figure()
    plt.plot(tvec, paccel)
    plt.show()
    

    # engagementPlotter.plotEngagementFullScale( tvec, 
    #                                dtIdx, 
    #                                missIdx,
    #                                betaThist,
    #                                [ppos, pvel],
    #                                [tpos, tvel],
    #                                'True')
        
    #print(f"collision at: ({engagementPlotter.collisionPoint[0]}, {engagementPlotter.collisionPoint[1]})")

  
    
    
        
