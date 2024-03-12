import numpy as np
from scipy.optimize import least_squares

# Define parameters
l0 = 0.041     # m
l1 = 0.025     # m
l1x = 0.01     # m
l2 = 0.09525   # m (3.75”)
l3 = 0.10795   # m (4.25”)
l4 = 0.085725  # m (3.375”)

x, y, z = (0.08, 0.08, 0.07)

orientation = np.pi

def equations(qs):
    q0, q1, q2, q3 = qs
    eq1 = np.cos(q0)*(l1x+l2*np.sin(q1) + l3*np.sin(q1+q2) + l4*np.sin(q1+q2+q3)) - x
    eq2 = np.sin(q0)*(l1x+l2*np.sin(q1) + l3*np.sin(q1+q2) + l4*np.sin(q1+q2+q3)) - y
    eq3 = l0 + l1 + l2*np.cos(q1) + l3*np.cos(q1+q2) + l4*np.cos(q1+q2+q3) - z
    return eq1, eq2, eq3


def equations_orientation(qs):
    q0, q1, q2, q3 = qs
    eq1 = np.cos(q0)*(l1x+l2*np.sin(q1) + l3*np.sin(q1+q2) + l4*np.sin(q1+q2+q3)) - x
    eq2 = np.sin(q0)*(l1x+l2*np.sin(q1) + l3*np.sin(q1+q2) + l4*np.sin(q1+q2+q3)) - y
    eq3 = l0 + l1 + l2*np.cos(q1) + l3*np.cos(q1+q2) + l4*np.cos(q1+q2+q3) - z
    eq4 = q1 + q2 + q3 - orientation
    return eq1, eq2, eq3, eq4


def FK(qs):
    q0, q1, q2, q3 = qs
    xf = np.cos(q0)*(l1x+l2*np.sin(q1) + l3*np.sin(q1+q2) + l4*np.sin(q1+q2+q3))
    yf = np.sin(q0)*(l1x+l2*np.sin(q1) + l3*np.sin(q1+q2) + l4*np.sin(q1+q2+q3))
    zf = l0 + l1 + l2*np.cos(q1) + l3*np.cos(q1+q2) + l4*np.cos(q1+q2+q3)
    return xf, yf, zf


res = least_squares(equations_orientation, (0, 0, 120*np.pi/180, 0), bounds = ((-np.pi/2, -np.pi/3, np.pi/2, -np.pi/2), (np.pi/2, np.pi/3, 160*np.pi/180, np.pi/2)))
print(res.cost)
print(FK(res.x))
print((res.x * 180/np.pi))

# 1a 44.99999540158393 -35.39631401076471 131.14051001614993 16.780832836585795
# 1b 45.00000000568893 -35.989212300338195 145.06289823013645 -13.85783524584236
# 1c 44.99999998019838 -32.17168302076569 124.18255894921438 27.96954621515868
# 1d 45.0000003105869 -36.73469010945322 135.43001894767596 8.970677559754671
# 1e 44.999999783643396 -37.247777842899175 138.79152190824337 2.0984447312954426
# 1f 45.00000001211157 -37.15681085316987 141.53425994687646 -4.226757242076513
# 1g 45.000000479526804 -36.69527782835328 143.48688505037603 -9.292542699460709
# 2 Unfeasible
# 3 Unfeasible
# 4 Unfeasible