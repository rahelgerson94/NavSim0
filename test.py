import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, array, abs
from numpy.linalg import norm, inv
from Vehicle import Pursuer
from Vehicle import Target
from Guide import Guide
from EngagementPlotter import EngagementPlotter

X = 0
Z = 1
R = 0
V = 1

class collisionTester:
    def __init__(self, p, t, g, tstart, tend, dt):
        self.pursuer = p
        self.target = t
        self.guide = g
        self.pursuerPosHist = []
        self.pursuerVelHist = []
        self.pursuerAccelHist = []
        self.targetPosHist = []
        self.targetVelHist = []
        self.losRateHist = []
        self.tvec  = []
        self.plotter = EngagementPlotter()
    def appendVars(self):
        self.pursuerPosHist.append(self.pursuer.RinI)
        self.pursuerVelHist.append(self.pursuer.VinI)
        self.pursuerAccelHist.append(self.pursuer.AinI)
        self.targetPosHist.append(self.target.RinI)
        self.targetVelHist.append(self.target.VinI)
        self.losRateHist.append(self.guide.lamdaDot)
    
def simulate(N, HE, pursuer, target, guide, tstart, tend, dt, title, savefig = False):
    tvec = np.linspace(tstart, tend, int(tend/dt))
    n = 0
    while (guide.getVc()) > 0:

    #for t in tvec:
        guide.update()
            
        pursuer.update(guide.getAtrueInI(), #proNav aceel in pursuer frame
                    guide.getLosAngleRad(), # line of sight angle
                    guide.getLookAngleRad()) 

        target.update()
        
        if np.abs(pursuer.rInI[X] - target.rInI[X]) <=  0.1 and\
            np.abs(pursuer.rInI[Z] - target.rInI[Z]) <= 0.1:
                print("Rt/p getting small")
                
        
        n+=1
    ep = EngagementPlotter()

    print(f"Vc({n}): {guide.getVc()}")
    guide.setCollisionPoint(array(pursuer.rInI[X],pursuer.rInI[Z] ))
    print(f"collision at: ({pursuer.rInI[0]}, {pursuer.rInI[1]})")
    ep.plotCollision(target.rInIhist, pursuer.rInIhist, title = title, savefig=savefig)
    
        
    
if __name__ == "__main__":
    N = 5
    dt = 2/1000

     
    ######### define target init orientation ########
    betaDeg0 = 30
    betaRad0 = deg2rad(betaDeg0)
    VtMag = 2 #m/s
    at0mag = 1/2
    AtInT0 = at0mag*array([0,at0mag]) #in the BODY frame
    VtInT0 = VtMag*array([1,0]) #in the BODY frame
    RtInI0 = array([60, 16]) #in the INERTIAL frame
    target = Target(AtInT0, 
                    betaDeg0, 
                    [RtInI0, VtInT0],
                    dt = dt)
    
    ######## define pursuer init orientation ##########
    RpInI0 = array([20, 0]) 
    VpInP0 = (1.1)*VtMag*array([1,0])
    ApInP0 = at0mag*array([0, 1])
    anglePursuerInertialDeg = 45
    
    HEdeg0 = -20
    pursuer = Pursuer(ApInP0, 
                      anglePursuerInertialDeg, 
                      HEdeg0 , 
                      [RpInI0, VpInP0],
                      dt = dt
                    )
    #printVeloRatio(target, pursuer)
    guide = Guide(pursuer, target, N=N, dt = dt, HEdeg = HEdeg0)
    print(f"Vt/p: {target.vInI - pursuer.vInI}")
    print(f"guide.Vc: {guide.getVc()}")
    print(f"Guide.L (deg): {guide.getLookAngleDeg()}")
    print(f"Guide.λ (deg): {guide.getLosAngleDeg()}")
    tstart = 0
    tend = 6
    
    HEs = [-20]
    
    for HEdeg0 in HEs:
        simulate(N, HEdeg0, pursuer, target, guide, tstart, tend, dt, title = f"HE (deg)= {HEdeg0}, N = {N}, ap = at, vp = 1.1vt", savefig=True)