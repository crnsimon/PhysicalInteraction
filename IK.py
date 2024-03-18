import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

sim = False

if sim:
    # Define parameters
    l0 = 0.05  # m
    l1 = 0.02  # m
    l1x = 0  # m
    l2 = 0.0825  # m (3.75”)
    l3 = 0.086  # m (4.25”)
    l4 = 0.075  # m (3.375”)
else:
    # Define parameters
    l0 = 0.045  # m
    l1 = 0.019  # m
    l1x = 0.013  # m
    l2 = 0.09525  # m (3.75”)
    l3 = 0.10795  # m (4.25”)
    l4 = 0.085725  # m (3.375”)


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

def orientation_graph(angles):
    ans = np.zeros((5, np.size(angles)))
    for i, orientation in enumerate(angles):
        res = least_squares(equations_orientation, (0, 0, 100*np.pi/180, 0), bounds = ((-np.pi/2, -np.pi/3, np.pi/2, -np.pi/2), (np.pi/2, np.pi/3, 160*np.pi/180, np.pi/2)))
        ans[:4, i] = res.x * 180/np.pi
        ans[4, i] = sum((np.array([x, y, z]) - np.array(FK(res.x)))**2)*100000
    for i in range(4):
        plt.plot(angles*180/np.pi, ans[i], label=f"q{i} [deg]")
    plt.plot(angles*180/np.pi, ans[4], label='Error', linestyle='--')
    plt.legend()
    plt.xlabel('Orientation [deg]')
    plt.grid()
    plt.show()

def angles_to_sim(qs):
    q0, q1, q2, q3 = qs
    q1 = -q1
    q2 = q2-np.pi/2
    q3 = -q3
    return q0, q1, q2, q3



if __name__ == '__main__':
    x, y, z = (0.15, 0.05, 0.1)
    orientation = np.pi/2
    res = least_squares(equations_orientation, (0, 0, 90 * np.pi / 180, 0), bounds=(
    (-np.pi / 2, -np.pi / 3, 70 * np.pi / 180, -np.pi / 2), (np.pi / 2, np.pi / 3, 150 * np.pi / 180, np.pi / 2)))
    print(FK(res.x), sum([res.x[1], res.x[2], res.x[3]]) * 180 / np.pi)
    qs = angles_to_sim(res.x)
    print(', '.join(str(x) for x in qs))

# 1 0.3217505543965733, 0.37462937171086075, 0.8840461193694984, 0.509416747643335
# 2 0.3217505543770969, 0.5340738075978947, 0.49687699432751753, -0.037196813271319316
# 3 -0.32175055437718225, 0.5340738075979996, 0.496876994327192, -0.03719681327174962
# 4 -0.3217505543967424, 0.37462937171281363, 0.884046119370625, 0.509416747642509
