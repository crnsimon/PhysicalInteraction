import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rotation(rotx, roty, rotz):
    rotx_mat = np.array([[1, 0, 0], [0, np.cos(rotx), -np.sin(rotx)], [0, np.sin(rotx), np.cos(rotx)]])
    roty_mat = np.array([[np.cos(roty), 0, np.sin(roty)], [0, 1, 0], [-np.sin(roty), 0, np.cos(roty)]])
    rotz_mat = np.array([[np.cos(rotz), -np.sin(rotz), 0], [np.sin(rotz), np.cos(rotz), 0], [0, 0, 1]])
    rot = rotx_mat @ roty_mat @ rotz_mat
    return rot  


def translation(x, y, z):
    return np.array([[x],[y],[z]])

def compound(rotmatrix, transvector):
    total = np.vstack([np.hstack([rotmatrix, transvector]), np.array([0, 0, 0, 1])])
    return total


# Define parameters
lground = 0.041  # m
l0z = 0.025  # m
l0x = 0.01  # m
r0 = np.radians(0)
l1 = 0.09525   # m (3.75”)
q1 = np.radians(-45)
l2 = 0.10795  # c (4.25”)
q2 = np.radians(90)
l3 = 0.085725  # m (3.375”)
q3 = np.radians(35)


# Create RGB axis data
length = 0.05
axis_system = np.array([[length,0,0], [0,length,0],[0,0,length]])
origin = np.array([0,0,0,0,0,0])
joint0 = np.array([0,0,lground,0,0,0])
joint1 = np.array([l0x,0,l0z,0,0,r0])
joint2 = np.array([0,0,l1,0,q1,0])
joint3 = np.array([0,0,l2,0,q2,0])
EE = np.array([0,0,l3,0,q3,0])
names = ['ground', 'joint0', 'joint1', 'joint2', 'joint3', 'EE']


# # Create figure and 3D axis
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# colors = ['r', 'g', 'b']
# latest_pos = np.array([[0],[0],[0]])
# latest_rot = np.array([[1,0,0],[0,1,0],[0,0,1]])
# namecount = 0

# # Plot each axis system at its respective position
# for pos in origin, joint0, joint1, joint2, joint3, EE:
#     old_pos = latest_pos
#     latest_rot = latest_rot @ rotation(pos[3],pos[4],pos[5])
#     latest_pos = latest_rot @ np.array([[pos[0]],[pos[1]],[pos[2]]]) + latest_pos
#     compound_matrix = compound(latest_rot, latest_pos)
#     new_pos = (compound_matrix @ np.vstack([np.array([[0],[0],[0]]),[1]]))[:-1]
#     ax.plot([old_pos[0][0], new_pos[0][0]], [old_pos[1][0], new_pos[1][0]], [old_pos[2][0], new_pos[2][0]], color='black')
#     ax.text(new_pos[0][0], new_pos[1][0], new_pos[2][0], names[namecount])
#     namecount += 1
#     for axis in axis_system:
#         new_axis = (compound_matrix @ np.vstack([*axis, np.array([1])]))[:-1]
#         ax.quiver(*new_pos,*(new_axis-new_pos), color=colors[axis_system.tolist().index(axis.tolist())])
#     latest_pos = new_pos
    

# # Set axis limits
# box_size = 0.15
# ax.set_xlim(-box_size, box_size)
# ax.set_ylim(-box_size, box_size)
# ax.set_zlim(0, 2*box_size)
        
# ax.set_box_aspect([1,1,1])

# # Set plot title
# plt.title('4DOF System')

# Show plot
# plt.savefig('3d.eps', format='eps', dpi=1000)
# plt.show()


# Forward kinematics

z = lground + l0z + l1*np.cos(q1) + l2*np.cos(q1 + q2) + l3*np.cos(q1 + q2 + q3)
y = (l1*np.sin(q1) + l2*np.sin(q1 + q2) + l3*np.sin(q1 + q2 + q3))*np.sin(r0)
x = (l0x + l1*np.sin(q1) + l2*np.sin(q1 + q2) + l3*np.sin(q1 + q2 + q3))*np.cos(r0)

# # reachable subspace, no limits
fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot

r0 = np.radians(0)
reachable_x = []
reachable_y = []
reachable_z = []

ax.add_patch(plt.Circle((l0x, l0z+lground), l1+l2+l3, color='b'))
ax.add_patch(plt.Circle((-l0x, l0z+lground), l1+l2+l3, color='b'))

# plt.scatter(l0x,(l0z+lground), color='r')

plt.axis('equal')
plt.xlabel('x [cm]')
plt.ylabel('z [cm]')
plt.title('Reachable subspace x-z')
plt.savefig('reachable_xz_nolimits.png', format='png', dpi=1000)
plt.show()  


# reachable subspace, limits

reachable_x = []
reachable_y = []
reachable_z = []

# for i in np.linspace(-90, 90, 100):

r0 = np.radians(0)
for j in np.linspace(-60, 60, 1000):
    q1 = np.radians(j)
    print(j)
    for k in np.linspace(90, 160, 100):
        q2 = np.radians(k)
        x = l0x + l1*np.sin(q1) + l2*np.sin(q1 + q2)
        y = (l1*np.sin(q1) + l2*np.sin(q1 + q2))*np.sin(r0)
        z = lground + l0z + l1*np.cos(q1) + l2*np.cos(q1 + q2)
        for l in np.linspace(-90, 90, 100):
            q3 = np.radians(l)
            z = lground + l0z + l1*np.cos(q1) + l2*np.cos(q1 + q2) + l3*np.cos(q1 + q2 + q3)
            y = (l1*np.sin(q1) + l2*np.sin(q1 + q2) + l3*np.sin(q1 + q2 + q3))*np.sin(r0)
            x = (l1*np.sin(q1) + l2*np.sin(q1 + q2) + l3*np.sin(q1 + q2 + q3))*np.cos(r0)
            reachable_x.append(x)
            reachable_y.append(y)
            reachable_z.append(z)

# plt.figure()
# plt.plot(reachable_x, reachable_y, 'b.')
# plt.axis('equal')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.title('Reachable subspace x-y')
# plt.savefig('reachable_xy.png', format='png', dpi=1000)
# plt.show()
plt.figure()
plt.plot(reachable_x, reachable_z, 'b.')
plt.axis('equal')
plt.xlabel('x [cm]')
plt.ylabel('z [cm]')
plt.title('Reachable subspace x-z')
plt.savefig('reachable_xz.png', format='png', dpi=1000)
plt.show()
# plt.figure()
# plt.plot(reachable_y, reachable_z, 'b.')
# plt.axis('equal')
# plt.xlabel('y')
# plt.ylabel('z')
# plt.title('Reachable subspace y-z')
# plt.savefig('reachable_yz.png', format='png', dpi=1000)
# plt.show()





