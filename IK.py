import numpy as np
from scipy.optimize import least_squares

# Define parameters
l0 = 0.041     # m
l1 = 0.025     # m
l1x = 0.01     # m
l2 = 0.09525   # m (3.75”)
l3 = 0.10795   # m (4.25”)
l4 = 0.085725  # m (3.375”)

x, y, z = (0.0, 0.0, 0.07)
def equations(qs):
    q1, q2, q3, q4 = qs
    eq1 = np.cos(q1)*(l1x-l2*np.sin(q2) + l3*np.cos(q2+q3) + l4*np.cos(q2+q3+q4)) - x
    eq2 = np.sin(q1)*(l1x-l2*np.sin(q2) + l3*np.cos(q2+q3) + l4*np.cos(q2+q3+q4)) - y
    eq3 = l0 + l1 + l2*np.cos(q2) + l3*np.cos(q2-q3) + l4*np.cos(q2-q3-q4) - z
    return (eq1,eq2,eq3)

def FK(qs):
    q1, q2, q3, q4 = qs
    x = np.cos(q1)*(l1x-l2*np.sin(q2) + l3*np.cos(q2+q3) + l4*np.cos(q2+q3+q4))
    y = np.sin(q1)*(l1x-l2*np.sin(q2) + l3*np.cos(q2+q3) + l4*np.cos(q2+q3+q4))
    z = l0 + l1 + l2*np.cos(q2) + l3*np.cos(q2-q3) + l4*np.cos(q2-q3-q4)
    return x, y, z

res = least_squares(equations, (0, 0, 20*np.pi/180, 0), bounds = ((-np.pi/2, -np.pi/3, 0, -np.pi/2), (np.pi/2, np.pi/3, 70*np.pi/180, np.pi/2)))
q1, q2, q3, q4 = res.x * 180/np.pi
print(FK(res.x))
print(q1, q2, q3, q4)

# 1 45.00000039433621 -24.979112060238233 58.291236860903645 56.02913713165773
# 2 26.5650542060046 -15.071944527609885 25.76090878960464 4.419201679989688
# 3 0 20.70996636189051 63.244699362791515 -2.2143191395548176
# 4 Unfeasible