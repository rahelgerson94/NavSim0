import matplotlib.pyplot as plt
from numpy import allclose
class EngagementPlotter:
    def __init__(self):
        pass
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
                break

        # Unpack truncated coordinates
        target_x, target_y = zip(*truncated_target)
        pursuer_x, pursuer_y = zip(*truncated_pursuer)

        # Plot the truncated trajectories
        plt.figure()
        plt.plot(target_x, target_y, 'b-o', label='Target', linewidth=2, markersize=6)
        plt.plot(pursuer_x, pursuer_y, 'r-o', label='Pursuer', linewidth=2, markersize=6)
        plt.scatter(target_x[-1], target_y[-1], c='g', s=100, label='Collision Point', zorder=5)

        # Add labels, legend, and grid
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Collision Plot: Target vs Pursuer (Stopped at Collision)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # Ensures equal scaling for x and y axes
        plt.show()

