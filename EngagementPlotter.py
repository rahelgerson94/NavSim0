import matplotlib.pyplot as plt
import numpy as np
from numpy import array, allclose, deg2rad, cos, sin, arctan2
from numpy.linalg import norm
from mpl_toolkits.mplot3d import Axes3D

X = 0
Y = 1
Z = 2
R = 0
V = 1
class EngagementPlotter:
    def __init__(self):
        self.collisionPoint = array([np.inf,np.inf])
        
    def plotCollision3d(self, target, pursuer, title=""):
        """
        Plots the trajectories of the target and pursuer and stops at the collision point.

        Parameters:
        - target_coords: List of tuples representing the target's (x, y) coordinates over time.
        - pursuer_coords: List of tuples representing the pursuer's (x, y) coordinates over time.
        """
        target_x, target_y, target_z = zip(*target.rInIhist)
        pursuer_x, pursuer_y, pursuer_z = zip(*pursuer.rInIhist)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        if tvec is not None:
            sc1 = ax.scatter(target_x, target_y, target_z, c=tvec, cmap='viridis', label='target',s = 2)
            sc2 = ax.scatter(pursuer_x, pursuer_y, pursuer_z, c=tvec, cmap='plasma', label='pursuer',s = 2)
        
            # Add a color bar for each trajectory to show time info
            cbar1 = fig.colorbar(sc1, ax=ax, pad=0.1, shrink=0.5, location='left')
            cbar1.set_label('time (target)')
            cbar2 = fig.colorbar(sc2, ax=ax, pad=0.1, shrink=0.5)
            cbar2.set_label('time (pursuer)')
        else:
            
            ax.scatter(target_x, target_y, target_z, color='red', label='target',s = 2)
             
            ax.plot(pursuer_x, pursuer_y, pursuer_z, color='blue', label='pursuer')
            ax.scatter(pursuer_x, pursuer_y, pursuer_z, color='blue', label='pursuer',s = 2)
        
        # Labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Trajectory')
        
        # set axis limits
        xmax = np.max([np.max(pursuer_x),np.max(target_x)])
        ymax = np.max([np.max(pursuer_y),np.max(target_y)])  
        zmax = np.max([np.max(pursuer_z),np.max(target_z)])  
        
        xmax += 0.1*xmax
        ymax += 0.1*ymax
        zmax += 0.1*zmax
        
        xmin = np.min([np.min(pursuer_x),np.min(target_x)])  
        ymin = np.min([np.min(pursuer_y),np.min(target_y)])  
        zmin = np.min([np.min(pursuer_z),np.min(target_z)])  
        
        xmin -= 0.1*xmin
        ymin -= 0.1*ymin
        zmin -= 0.1*zmin 
        
        ax.set_xlim([xmin, xmax])  # X-axis range
        ax.set_ylim([ymin, ymax])  # Y-axis range
        ax.set_zlim([zmin, zmax])  # Z-axis range (beyond the trajectory for a larger grid)


        # Add legend
        ax.legend()

        # Show the plot
        plt.show()
        
    '''
    @name:  plotVehicleStatesSubplots
    @brief: plots a single vehicles accel, vel, pos 
    '''
    def plotVehicleStatesSubplots(self, tvec, v, title="", figNum = 0):
        names = ["aInI", "vInI", "rInI"]
        hists = [v.aInIhist, v.vInIhist, v.rInIhist]
        states = ["a(t) m/s^2", "v(t) m/s", "r(t) m"]
        fig = plt.figure()
        
        for i,n in enumerate(names):
            ax = fig.add_subplot(3,1,i+1, projection='3d')
            ax.scatter(x2, y2, z2, color='red', s=10) 
            if len(title) >0:
                plt.title(f"{title}: {states[i]}")
            else:
                plt.title(f"Vehicle: {states[i]}")
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        plt.show()
    def plotStates3d(self, vehicle):

        """
        Plot position, velocity, and acceleration in 3D.
        
        Parameters:
            pos: numpy array of shape (n, 3) representing position (x, y, z).
            vel: numpy array of shape (n, 3) representing velocity (x, y, z).
            accel: numpy array of shape (n, 3) representing acceleration (x, y, z).
        """
        # Create a figure with 3 subplots
        fig = plt.figure(figsize=(15, 5))
        titles = ['Position', 'Velocity', 'Acceleration']
        data = [vehicle.rInIhist, vehicle.vInIhist, vehicle.aInIhist]
        data = [np.array(d) for d in data]
        for i, (array, title) in enumerate(zip(data, titles)):
            ax = fig.add_subplot(1, 3, i + 1, projection='3d')
            
            # Extract x, y, z columns
            x, y, z = array[:, 0], array[:, 1], array[:, 2]
            
            # Scatter plot with color by time
            sc = ax.scatter(x, y, z, s=10)
            ax.plot(x, y, z, color='gray', alpha=0.7)  # Optional: Connect points
            
            # Labels and title
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(title)
                    
        # Adjust layout
        plt.tight_layout()
        plt.show()
    def plotStateProjections(self, tvec, target, pursuer, state='r', overlayed=False):
        '''
        @name:  plotprojections
        @brief: plots pursuer (state) in x,y,z 
                      target  (state) in x,y,z 
        '''
        if state == 'r':
            stateName = "r(t)"
            pdata = pursuer.rInIhist
            tdata = target.rInIhist
        elif state == 'v':
            stateName = "v(t)"
            pdata = pursuer.vInIhist
            tdata = target.vInIhist
        elif state == 'a':
            stateName = "a(t)"
            pdata = pursuer.aInIhist
            tdata = target.aInIhist
        pdata = array(pdata)
        tdata = array(tdata)


        '''
        if overlayed == True, this will plot 3 plots, each with 
        target, pursuer states overlayed on top of one other'''
        if overlayed:
            fig, axez = plt.subplots(1, 3, figsize=(10, 12))  # Adjust figsize as needed
        
            
            
            axesNames = ["x", "y", "z"]
            
            for j in range(len(axesNames)):  # Columns
                axez[j].plot(tvec, pdata[:, j], color = 'blue', label = 'pursuer')  
                axez[j].plot(tvec, tdata[:, j], color = 'red', label = 'target')    
                axez[ j].set_title(f"{stateName} {axesNames[j]}")  # Title for each subplot
                axez[j].set_ylim(-100, 100)
                axez[j].grid(True)  # Add grid
            plt.legend()
            plt.tight_layout()
            plt.show()
        
        else:
            '''
            if overlayed == False, this will plot a grid of  
            (2 rows, 3 cols) 
            target pos,  target vel,  target accel
            pursuer pos, pursuer vel, pursuer accel'''
            fig, axez = plt.subplots(2, 3, figsize=(10, 12))  # Adjust figsize as needed          
            playerNames = ["pursuer", "target"]
            playerData = [pdata, tdata]
            axesNames = ["x", "y", "z"]
            for i in range(len(playerNames)):  # Rows
                for j in range(len(axesNames)):  # Columns
                    axez[i, j].plot(tvec, playerData[i][:, j])   
                    axez[i, j].set_title(f" {playerNames[i]} {stateName} {axesNames[j]}")  # Title for each subplot
                    axez[i,j].set_ylim(-100, 100)
                    axez[i, j].grid(True)  # Add grid
            plt.tight_layout()
            plt.show()
            
        
            

    def plotVehiclesSubplots(self, tvec, p, t ):
        '''
        @name:  plotVehiclesSubplots
        @brief: plots pursuer accel, vel, pos in left col, and
              target  accel, vel, pos in right col
        '''
        names = ["aInI", "vInI", "rInI"]
        states = ["a(t) m/s^2", "v(t) m/s", "r(t) m"]
        pursuer_states = [p.aInIhist, p.vInIhist, p.rInIhist]
        target_states = [t.aInIhist, t.vInIhist, t.rInIhist ] 
         
        # Create a 3x2 grid of subplots
        fig, axes = plt.subplots(3, 2, figsize=(10, 12))  # Adjust figsize as needed
        
        
        # Plot the data
        for i, (pursuer, target) in enumerate(zip(pursuer_states, target_states)):
            # Plot pursuer data in the first column
            axes[i, 0].plot(tvec, pursuer, label="Pursuer")
            #axes[i, 0].set_title(f"Pursuer {states[i]}")
            axes[i, 0].set_xlabel("t (s)")
            axes[i, 0].set_ylabel(states[i])
            axes[i, 0].grid(True)
            # Plot target data in the second column
            axes[i, 1].plot(tvec, target, label="Target", color="orange")
            #axes[i, 1].set_title(f"Target {states[i]}")
            axes[i, 1].set_xlabel("t (s)")
            axes[i, 1].set_ylabel(states[i])
            axes[i, 1].grid(True)
        # Add space between plots
        plt.tight_layout()
        plt.show()
    def plotGuideVars(self, tvec, g, title=""):


        names = {"d/dt(λ) (deg/s)",
                 "λ (deg)",
                 "Vc (m/s)",
                 "L (deg)",
                 "Rt/p (m)",
                 }
        hists = [g.lamdaDotHist,
                 g.lamdaHist,
                 g.VcHist,
                 g.lookAngleHist,
                 g.RrelHist]
        for i, name in enumerate(names):
            plt.figure(i)
            
            plt.plot(tvec, hists[i])
            
            if len(name) >0:
                plt.title(f"{title}: {name}")
            else:
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

