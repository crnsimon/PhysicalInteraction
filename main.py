import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

def calculate_expression(l1, l2, l3, q1, q2, q3, r0):
    x = (l1 * math.sin(q1) + l2 * math.sin(q1 + q2) + l3 * math.sin(q1 + q2 + q3)) * math.cos(r0)
    y = (l1 * math.sin(q1) + l2 * math.sin(q1 + q2) + l3 * math.sin(q1 + q2 + q3)) * math.sin(r0)
    z = l1 * math.cos(q1) + l2 * math.cos(q1 + q2) + l3 * math.cos(q1 + q2 + q3)
    return x, y, z

def inverse_kinematics(l1, l2, l3, x, y, z):
    r0 = math.atan2(y, x)
    y = y / math.sin(r0)
    x = x / math.cos(r0)
    
    return math.degrees(q1), math.degrees(q2), math.degrees(q3), math.degrees(r0)


l1 = 9.525  # cm (3.75”)
l2 = 10.795  # cm (4.25”)
l3 = 8.5725  # cm (3.375”)

# listy = []
# listz = []

# r0 = math.radians(90)  # radians
# for i in range(0, 360, 10):
#     for j in range(0, 360, 10):
#         for k in range(0, 360, 10):
#             q1 = math.radians(i)
#             q2 = math.radians(j)
#             q3 = math.radians(k)
#             x, y, z = calculate_expression(l1, l2, l3, q1, q2, q3, r0)
#             listy.append(y)
#             listz.append(z)
# plt.plot(listy, listz, linewidth=0.1)
# plt.xlabel('y - [cm]')
# plt.ylabel('z - [cm]')
# plt.axis('square')
# plt.savefig('subspace_no_limits.eps', format='eps', dpi=1000)
# plt.show()

# listx = []
# listy = []
# listz = []

# for r0 in range(0, 360, 5):
#     for i in range(0, 360, 30):
#         for j in range(0, 360, 30):
#             for k in range(0, 360, 30):
#                 q1 = math.radians(i)
#                 q2 = math.radians(j)
#                 q3 = math.radians(k)
#                 x, y, z = calculate_expression(l1, l2, l3, q1, q2, q3, r0)
#                 listx.append(x)
#                 listy.append(y)
#                 listz.append(z)
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(listx, listy, listz)
# plt.show()

# for i in [[0.1,0.1,0.1],[0.2,0.1,0.3],[0.0,0.0,0.3],[0.0,0.0,0.07]]:
#     x, y, z = i
#     q1, q2, q3, r0 = inverse_kinematics(l1, l2, l3, x, y, z)
#     print(f'x: {x}, y: {y}, z: {z}, q1: {q1}, q2: {q2}, q3: {q3}, r0: {r0}')
#     x, y = calculate_expression(l1, l2, l3, math.radians(q1), math.radians(q2), math.radians(q3), math.radians(r0))
#     print(f'x: {x}, y: {y}, z: {z}, q1: {q1}, q2: {q2}, q3: {q3}, r0: {r0}')

