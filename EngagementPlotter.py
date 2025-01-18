import matplotlib.pyplot as plt
import numpy as np
from numpy import array, allclose, deg2rad, cos, sin, arctan2
from numpy.linalg import norm

X = 0
Z = 1
R = 0
V = 1
class EngagementPlotter:
    def __init__(self):
        self.collisionPoint = array([np.inf,np.inf])
        
    def plotCollision(self, target_coords, pursuer_coords):
        """
        Plots the trajectories of the target and pursuer and stops at the collision point.

        Parameters:
        - target_coords: List of tuples representing the target's (x, y) coordinates over time.
        - pursuer_coords: List of tuples representing the pursuer's (x, y) coordinates over time.
        """
        # Truncate at collision point
        truncated_target = []
        truncated_pursuer = []

        for t, p in zip(target_coords, pursuer_coords):
            truncated_target.append(t)
            truncated_pursuer.append(p)
            if allclose(t,p):  # Collision point found
                self.collisionPoint = array([t[0], t[1]])
                print(f"collision at: ({self.collisionPoint[0]}, {self.collisionPoint[1]})")
                break
            
        print("no collision found")
        print(f"Rt(tf) = ({t[0]}, {t[1]})")
        print(f"Rp(tf) = ({p[0]}, {p[1]})")       
        # Unpack truncated coordinates
        target_x, target_y = zip(*truncated_target)
        pursuer_x, pursuer_y = zip(*truncated_pursuer)

        # Plot the truncated trajectories
        plt.figure()
        plt.plot(target_x, target_y, 'r-o', label='Target', linewidth=2, markersize=2)
        plt.plot(pursuer_x, pursuer_y, 'g-o', label='Pursuer', linewidth=2, markersize=2)
        plt.scatter(target_x[-1], target_y[-1], c='b', s=100, label='Collision Point', zorder=5)

        # Add labels, legend, and grid
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Collision Plot: Target vs Pursuer (Stopped at Collision)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # Ensures equal scaling for x and y axes
        plt.show()
    def plotPursuerVars(self, tvec, p):
        names = {"A"}
        hists = {p.AinI}
        for i,n in enumerate(names):
            plt.figure()
            plt.plot(tvec, hists[i])
            plt.title(n)
            plt.legend()
            plt.grid(True)
            plt.show()
        
    def plotGuideVars(self, tvec, g):
        
        names = {"d/dt(λ) (deg/s)",
                 "λ (deg)",
                 "Vc (m/s)",
                 "L (deg)",
                 "Rt/p (m)",
                 }
        hists = [g.lamdaDotHist,
                 g.lamdaHist,
                 g.VcInIhist,
                 g.lookAngleHist,
                 g.RrelHist]
        for i, name in enumerate(names):
            plt.figure()
            plt.plot(tvec, hists[i])
            plt.title(name)
            plt.legend()
            plt.grid(True)
            plt.show()
                
    
        
            
        # Define functions for plotting engagement
        
    def plotEngagementFullScale(self, 
                                   tvec, 
                                   dt_index, 
                                   miss_index,
                                   betaTimeHist,
                                   pursuerStateTimeHist,
                                   targetStateTimeHist,
                                   pnType, 
                                   HEdeg = 20,
                                   Np = 3,  
                                   vid_file="off"):
        
        HE = deg2rad(HEdeg)
        Rp = pursuerStateTimeHist[R]
        Vp = pursuerStateTimeHist[V]
        Rt = targetStateTimeHist[R]
        Vt = targetStateTimeHist[V]
        k = 1
        for ii in range(0, len(tvec), dt_index):
            if ii == 0:
                #plt.figure(6)
                plt.plot(Rp[0,X], Rp[0,Z], 'ob', label="Pursuer", markersize=5)
                plt.plot(Rt[0,X], Rt[0,Z], 'or', label="Target", markersize=5)
                plt.title(f"{pnType} ProNav, -{HEdeg} Deg, N = {Np}", fontsize=16)
                plt.xlabel("Downrange [ft]", fontsize=16)
                plt.ylabel("Altitude [ft]", fontsize=16)
                plt.grid()
                plt.axis("equal")
                plt.xlim([0, 40000])
                plt.ylim([6000, 12000])
            
            if ii > 0:
                plt.plot(Rp[ii, X], Rp[ii-dt_index, Z], 'b-', linewidth=2, label="Pursuer")
                plt.plot(Rt[ii, X], Rt[ii-dt_index, Z], 'r-', linewidth=2, label="Target")
                plt.pause(0.1)

            if vid_file == "on":
                plt.savefig(f"frame_{k}.png")
                k += 1

        if vid_file == "on":
            print("Save video using an external tool.")


    def plotEngagementZoomed(self,
                               pursuerStateTimeHist,
                               pursuerAccelTimeHist,
                               targetStateTimeHist,
                               tvec, 
                               pnType, 
                               lambdaTimeHist, 
                               dt_index, 
                               vid_file="off"):
        Rp = pursuerStateTimeHist[R]
        Vp = pursuerStateTimeHist[V]
        Rt = targetStateTimeHist[R]
        Vt = targetStateTimeHist[V]
        k = 1
        for ii in range(0, len(tvec), dt_index):

            plt.figure(7)
            ph1 = plt.quiver(Rp[ii,X], Rp[ii-dt_index,Z], 
                       Vp[ii,X], Vp[ii,Z], 
                       color='b', scale=1, scale_units='xy')
            plt.grid()
            if ii > 0:
                plt.plot(Rt[ii,X], Rt[ii-dt_index,X], 'r-', linewidth=2)
                plt.plot(Rt[ii,Z], Rt[ii-dt_index,Z], 'r-', linewidth=2)

                plt.plot(Rp[ii,X], Rp[ii-dt_index,X], 'b-', linewidth=2)
                plt.plot(Rp[ii,Z], Rp[ii-dt_index,Z], 'b-', linewidth=2)
            #  Determine pursuer acceleration vector
            if pnType == 'Pure':
                Heading_pursuer = np.atan2(Vp[ii,X], Vp[ii,Z]);
                apx = -pursuerAccelTimeHist[ii] * np.sin(Heading_pursuer) * 8
                apz = pursuerAccelTimeHist[ii] * np.cos(Heading_pursuer) * 8
            elif pnType == 'True':
                ph2 = plt.plot(Rp[ii,X], Rp[ii-dt_index,X], 'b-', linewidth=2)
                ph2a = plt.plot(Rp[ii,Z], Rp[ii-dt_index,Z], 'b-', linewidth=2)
                apx = -pursuerAccelTimeHist[ii]*sin(lambdaTimeHist[ii])*8;
                apz =  pursuerAccelTimeHist[ii]*cos(lambdaTimeHist[ii])*8;
            #Plot pursuer acceleration vector
            ph3 = plt.quiver(Rp[ii,X], Rp[ii-dt_index,Z], 
                       apx, apz, 
                       color='b', scale=1, scale_units='xy')
            # Plot pursuer point
            ph4 = plt.plot(Rp[ii, X], Rp[ii, Z], 'b-', linewidth=2)
       
            plt.xlim([Rp[ii, X] - 4000, Rp[ii, X] + 4000])
            plt.xlim([Rp[ii, Z] - 4000, Rp[ii, Z] + 4000])

            plt.title(f"Engagement Visualization - {pnType} ProNav", fontsize=14)
            plt.grid()
            plt.pause(0.1)

            if vid_file == "on":
                plt.savefig(f"zoomed_frame_{k}.png")
                k += 1
        #Delete plots for next frame
        if pnType == 'True':
            ph2.remove()
            ph2a.remove()
        ph1.remove()
        ph3.remove()
        ph4.remove()
       
    

    def plot_late_engagement(self, yout, tvec, pnType, dt_index, miss_index, vid_file="off"):
        k = 1
        plt.figure(8)
        for ii in range(907, miss_index + dt_index, dt_index):
            plt.plot([yout[ii, 1], yout[ii - dt_index * 4, 1]],
                    [yout[ii, 2], yout[ii - dt_index * 4, 2]], 'r.-', linewidth=1, markersize=14)
            plt.plot([yout[ii, 3], yout[ii - dt_index * 4, 3]],
                    [yout[ii, 4], yout[ii - dt_index * 4, 4]], 'b.-', linewidth=1, markersize=14)
            plt.plot([yout[ii, 3], yout[ii, 1]],
                    [yout[ii, 4], yout[ii, 2]], 'k--', linewidth=1)

            if k == 1:
                plt.xlabel("Downrange [ft]", fontsize=16)
                plt.ylabel("Altitude [ft]", fontsize=16)
                plt.title(f"Collision Triangle Visualization - {pnType} ProNav", fontsize=16)
                plt.grid()

            plt.pause(0.1)

            if vid_file == "on":
                plt.savefig(f"late_frame_{k}.png")
                k += 1

        if vid_file == "on":
            print("Save video using an external tool.")
if __name__ == "__main__":
    # Example of how to use plotting functions
    ep = EngagementPlotter()
    dt_index = int(0.1 / 0.01)  # Assuming h = 0.01
    yout = np.array(yout)  # Ensure yout is a NumPy array
    ep.plotEngagementFullScale(yout, t, pnType, Np, dt_index, len(yout) - 1)
    ep.plotEngagementZoomed(yout, t, pnType, aM, RTMx, RTMz, lambda_, dt_index)
    #ep.plot_late_engagement(yout, t, pnType, dt_index, len(yout) - 1)

