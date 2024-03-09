import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rotation(rotx, roty, rotz):
    rotx, roty, rotz = np.radians(rotx), np.radians(roty), np.radians(rotz)
    rotx_mat = np.array([[1, 0, 0], [0, np.cos(rotx), -np.sin(rotx)], [0, np.sin(rotx), np.cos(rotx)]])
    roty_mat = np.array([[np.cos(roty), 0, np.sin(roty)], [0, 1, 0], [-np.sin(roty), 0, np.cos(roty)]])
    rotz_mat = np.array([[np.cos(rotz), -np.sin(rotz), 0], [np.sin(rotz), np.cos(rotz), 0], [0, 0, 1]])
    rot = rotx_mat @ roty_mat @ rotz_mat
    return rot  


def translation(x, y, z):
    return np.array([[x],[y],[z]])

# def compound(x, y, z, rotx, roty, rotz):
#     total =  np.vstack([np.hstack([rotation(rotx, roty, rotz), translation(x, y, z)]), np.array([0, 0, 0, 1])])
#     return total


def compound(rotmatrix, transvector):
    total = np.vstack([np.hstack([rotmatrix, transvector]), np.array([0, 0, 0, 1])])
    return total


# Define parameters
l0 = 4.1  # cm
l1 = 2.5  # cm
l2 = 9.525   # cm (3.75”)
l3 = 10.795  # cm (4.25”)
l4 = 8.5725  # cm (3.375”)


# Create RGB axis data
length = 5
axis_system = np.array([[length,0,0], [0,length,0],[0,0,length]])
origin = np.array([0,0,0,0,0,0])
joint0 = np.array([0,0,l0,0,0,0])
joint1 = np.array([0,0,l1,0,0,45])
joint2 = np.array([0,0,l2,-45,0,0])
joint3 = np.array([0,0,l3,90,0,0])
EE = np.array([0,0,l4,35,0,0])
names = ['origin', 'joint0', 'joint1', 'joint2', 'joint3', 'EE']


# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors = ['r', 'g', 'b']
latest_pos = np.array([[0],[0],[0]])
latest_rot = np.array([[1,0,0],[0,1,0],[0,0,1]])
namecount = 0

# Plot each axis system at its respective position
for pos in origin, joint0, joint1, joint2, joint3, EE:
    old_pos = latest_pos
    latest_rot = latest_rot @ rotation(pos[3],pos[4],pos[5])
    latest_pos = latest_rot @ np.array([[pos[0]],[pos[1]],[pos[2]]]) + latest_pos
    compound_matrix = compound(latest_rot, latest_pos)
    new_pos = (compound_matrix @ np.vstack([np.array([[0],[0],[0]]),[1]]))[:-1]
    ax.plot([old_pos[0][0], new_pos[0][0]], [old_pos[1][0], new_pos[1][0]], [old_pos[2][0], new_pos[2][0]], color='black')
    ax.text(new_pos[0][0], new_pos[1][0], new_pos[2][0], names[namecount])
    namecount += 1
    for axis in axis_system:
        new_axis = (compound_matrix @ np.vstack([*axis, np.array([1])]))[:-1]
        ax.quiver(*new_pos,*(new_axis-new_pos), color=colors[axis_system.tolist().index(axis.tolist())])
    latest_pos = new_pos
    

# Set axis limits
box_size = 15
ax.set_xlim(-box_size, box_size)
ax.set_ylim(-box_size, box_size)
ax.set_zlim(0, 2*box_size)
        
# ax.set_box_aspect([1,1,1])

# Set plot title
plt.title('RGB Axis System')

# Show plot
plt.show()


