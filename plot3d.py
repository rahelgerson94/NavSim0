import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Example data for trajectory 1
time1 = np.linspace(0, 10, 100)
x1 = np.sin(time1)
y1 = np.cos(time1)
z1 = time1

# Example data for trajectory 2
time2 = np.linspace(0, 10, 100)
x2 = np.sin(time2) + 0.5
y2 = np.cos(time2) - 0.5
z2 = time2

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory 1 with colormap
sc1 = ax.scatter(x1, y1, z1, c=time1, cmap='viridis', label='Trajectory 1', s=15)

# Plot trajectory 2 with colormap
sc2 = ax.scatter(x2, y2, z2, c=time2, cmap='plasma', label='Trajectory 2', s=15)

# Add a color bar for each trajectory
cbar1 = fig.colorbar(sc1, ax=ax, pad=0.1, shrink=0.5, location='left')
cbar1.set_label('Time (Trajectory 1)', labelpad=10)
cbar2 = fig.colorbar(sc2, ax=ax, pad=0.1, shrink=0.5)
cbar2.set_label('Time (Trajectory 2)', labelpad=10)

# Labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Overlaid Trajectories with Colormaps')

# Add a legend
ax.legend()

# Show the plot
plt.show()



