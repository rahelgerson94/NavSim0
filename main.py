import numpy as np
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
if __name__ == "__main__":
    
    engagementPlotter = EngagementPlotter()
    ######## define pursuer init orientation ##########
    pursuerAngleFromHorizontalDeg = 60
    pursuerAngleFromHorizontalRad = deg2rad(pursuerAngleFromHorizontalDeg)
    HEdeg0 = (10)
    lamda0 = deg2rad(20) #los angle from horizonatal
    VpMag = 5 #m/s
    ap0 = array([1, 0]) 
    Vp0 = VpMag* array([cos(pursuerAngleFromHorizontalRad), 
                        sin(pursuerAngleFromHorizontalRad)])
    Rp0 = array([20, 0]) 
    pursuer = Pursuer(ap0, 
                      pursuerAngleFromHorizontalDeg,
                      HEdeg0, 
                      [Rp0, Vp0]
                    )
    
    ######### define target init orientation ########
    targetAngleFromHorizontal = deg2rad(110)
    VtMag = 4 #m/s
    
    B0 = deg2rad(70) #Target heading
    at0 = array([.5, 0])
    Vt0 = VtMag* array([cos(targetAngleFromHorizontal),  #TODO: implement different, less ideal angles from pursuer
                        -sin(targetAngleFromHorizontal)])
    Rt0 = array([35, 40])
    target = Target(at0, 
                    targetAngleFromHorizontal, 
                    [Rt0, Vt0])
    
    guide = Guide()

    tstart = 0
    tend = 100
    dt = .001

    tvec = np.linspace(tstart, tend, int(tend/dt))
    for i in tvec:
        guide.update(pursuer,target)
        
        pursuer.update(guide.getApureInP(), #proNav aceel in pursuer frame
                       guide.getLosAngle(), # line of sight angle
                       guide.getLookAngle()) 
        target.update()
        ppos.append(pursuer.RinI)
        pvel.append(pursuer.VinI)
        tpos.append(target.RinI)
        tvel.append(target.VinI)
    #engagementPlotter.plotCollision(ppos, tpos)
    
    
        
