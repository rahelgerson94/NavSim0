import matplotlib.pyplot as plt
import numpy as np
from numpy import array, allclose
from numpy.linalg import norm

sel_beta = 0;
sel_RT1  = 1;
sel_RT2  = 2;
sel_RM1  = 3;
sel_RM2  = 4;
sel_VT1  = 5;
sel_VT2  = 6;
sel_VM1  = 7;
sel_VM2  = 8;


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
            else:
                print("no collisio found")
                print(f"Rt(tf) = ({t[0]}, {t[1]})")
                print(f"Rp(tf) = ({p[0]}, {p[1]})")
        # Unpack truncated coordinates
        target_x, target_y = zip(*truncated_target)
        pursuer_x, pursuer_y = zip(*truncated_pursuer)

        # Plot the truncated trajectories
        plt.figure()
        plt.plot(target_x, target_y, 'r-o', label='Target', linewidth=2, markersize=6)
        plt.plot(pursuer_x, pursuer_y, 'g-o', label='Pursuer', linewidth=2, markersize=6)
        plt.scatter(target_x[-1], target_y[-1], c='b', s=100, label='Collision Point', zorder=5)

        # Add labels, legend, and grid
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Collision Plot: Target vs Pursuer (Stopped at Collision)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # Ensures equal scaling for x and y axes
        plt.show()

        # Define functions for plotting engagement

    def plot_full_scale_engagement(self, yout, tvec, PN_type, Np, dt_index, miss_index, vid_file="off"):
        k = 1
        for ii in range(0, len(tvec), dt_index):
            if ii == 0:
                plt.figure(6)
                plt.plot(yout[0, 3], yout[0, 4], 'ob', label="Missile", markersize=5)
                plt.plot(yout[0, 1], yout[0, 2], 'or', label="Target", markersize=5)
                plt.title(f"{PN_type} ProNav, -20 Deg HE, N = {Np}", fontsize=16)
                plt.xlabel("Downrange [ft]", fontsize=16)
                plt.ylabel("Altitude [ft]", fontsize=16)
                plt.grid()
                plt.axis("equal")
                plt.xlim([0, 40000])
                plt.ylim([6000, 12000])
            
            if ii > 0:
                plt.plot(yout[ii-dt_index:ii+1, 1], yout[ii-dt_index:ii+1, 2], 'r-', linewidth=2, label="Target")
                plt.plot(yout[ii-dt_index:ii+1, 3], yout[ii-dt_index:ii+1, 4], 'b-', linewidth=2, label="Missile")
                plt.pause(0.1)

            if vid_file == "on":
                plt.savefig(f"frame_{k}.png")
                k += 1

        if vid_file == "on":
            print("Save video using an external tool.")


    def plot_zoomed_engagement(self, yout, tvec, PN_type, aM, RTMx, RTMz, lambda_, dt_index, vid_file="off"):
        k = 1
        for ii in range(0, len(tvec), dt_index):
            VMx = yout[ii, 7]
            VMz = yout[ii, 8]

            plt.figure(7)
            plt.quiver(yout[ii, 3], yout[ii, 4], VMx, VMz, color='b', scale=1, scale_units='xy')
            if ii > 0:
                plt.plot(yout[ii-dt_index:ii+1, 1], yout[ii-dt_index:ii+1, 2], 'r-', linewidth=2)
                plt.plot(yout[ii-dt_index:ii+1, 3], yout[ii-dt_index:ii+1, 4], 'b-', linewidth=2)

            aMx = -aM[ii] * np.sin(lambda_[ii]) * 8
            aMz = aM[ii] * np.cos(lambda_[ii]) * 8
            plt.quiver(yout[ii, 3], yout[ii, 4], aMx, aMz, color='k', scale=1, scale_units='xy')

            plt.xlim([yout[ii, 3] - 4000, yout[ii, 3] + 4000])
            plt.ylim([yout[ii, 4] - 4000, yout[ii, 4] + 4000])
            plt.title(f"Engagement Visualization - {PN_type} ProNav", fontsize=14)
            plt.grid()
            plt.pause(0.1)

            if vid_file == "on":
                plt.savefig(f"zoomed_frame_{k}.png")
                k += 1


    def plot_late_engagement(self, yout, tvec, PN_type, dt_index, miss_index, vid_file="off"):
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
                plt.title(f"Collision Triangle Visualization - {PN_type} ProNav", fontsize=16)
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
    ep.plot_full_scale_engagement(yout, t, PN_type, Np, dt_index, len(yout) - 1)
    ep.plot_zoomed_engagement(yout, t, PN_type, aM, RTMx, RTMz, lambda_, dt_index)
    ep.plot_late_engagement(yout, t, PN_type, dt_index, len(yout) - 1)

