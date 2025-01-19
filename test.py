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
    
    def run(self, plotTitle = ""):
        n = 0
        while self.guide.getVcInI() >= 0:           
            guide.update(pursuer,target) 
            self.pursuer.update(self.guide.getApureInI(), #proNav aceel in self.pursuer frame
                        self.guide.getLosAngleRad(), # line of sight angle
                        self.guide.getLookAngleRad()) 

            self.target.update()
            self.appendVars()
            if np.abs(self.pursuer.RinI[X] - self.target.RinI[X]) <=  0.1 and\
                np.abs(self.pursuer.RinI[Z] - self.target.RinI[Z]) <= 0.1:
                    print("Rt/p getting small")
                    
            self.tvec.append(n*dt)
            n+=1
        self.guide.setCollisionPoint(array(self.pursuer.RinI[X],self.pursuer.RinI[Z] ))
        print(f"collision at: ({self.pursuer.RinI[0]}, {self.pursuer.RinI[1]})")
        self.plotter.plotCollision(tester.targetPosHist, tester.pursuerPosHist, plotTitle)
        self.plotter.plotGuideVars(tester.tvec[0:len(tester.pursuerPosHist)], guide, plotTitle)
    
if __name__ == "__main__":
    dt  = 2/1000
    betaDeg0 = 0
    betaRad0 = deg2rad(betaDeg0)
    VtMag = 2 #m/s
    at0mag = 1/2
    T2Idcm = array([-cos(betaRad0),  #TODO: implement different, less ideal angles from pursuer
                        sin(betaRad0)])
    at0 = at0mag*T2Idcm
    Vt0 = VtMag* T2Idcm
    Rt0 = array([60, 16])
   
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
    
    #instantiate the objects
    target = Target(at0, 
                    betaDeg0, 
                    [Rt0, Vt0],
                    dt = dt)
    pursuer = Pursuer(ap0, 
                      rad2deg(pursuerAngleFromHorizontalRad), 
                      [Rp0, Vp0],
                      dt = dt
                    )
    guide = Guide(N=3, dt = dt)
    tester = collisionTester(pursuer, target, guide,
                             0,
                             8,
                             dt)
    #tester.run()
    
    ########### test2: target accel > pursuer accel ###########
    target = Target(5*ap0, 
                    betaDeg0, 
                    [Rt0, Vt0],
                    dt = dt)
    tester = collisionTester(pursuer, target, guide,
                             0, #start time
                             12, #end time
                             dt)
    #tester.run("target accel = 5x pursuer accel")
    
    target = Target(2*ap0, 
                    betaDeg0, 
                    [Rt0, Vt0],
                    dt = dt)
    tester = collisionTester(pursuer, target, guide,
                             0, #start time
                             8, #end time
                             dt)
    tester.run("target accel = 2x pursuer accel")