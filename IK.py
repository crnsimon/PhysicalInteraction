import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

sim = True

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
    l0 = 0.041  # m
    l1 = 0.025  # m
    l1x = 0.01  # m
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
    global orientation
    for i, orient in enumerate(angles):
        orientation = orient
        res = least_squares(equations_orientation, (0, 0, 90*np.pi/180, 0), bounds=((-np.pi/2, -np.pi/3, 70*np.pi/180, -np.pi/2), (np.pi/2, np.pi/3, 150*np.pi/180, np.pi/2)))
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

def inverse_kinematics(x0, orient=None, printing=False):
    global x, y, z, orientation
    x, y, z = x0
    orientation = orient
    if orient is not None:
        res = least_squares(equations_orientation, (0, 0, 90 * np.pi / 180, 0), bounds=(
            (-np.pi / 2, -np.pi / 3, 70 * np.pi / 180, -np.pi / 2),
            (np.pi / 2, np.pi / 3, 150 * np.pi / 180, np.pi / 2)))
    else:
        res = least_squares(equations, (0, 0, 90 * np.pi / 180, 0), bounds=(
            (-np.pi / 2, -np.pi / 3, 70 * np.pi / 180, -np.pi / 2),
            (np.pi / 2, np.pi / 3, 150 * np.pi / 180, np.pi / 2)))
    x = FK(res.x)
    if printing:
        print(FK(res.x), sum([res.x[1], res.x[2], res.x[3]]) * 180 / np.pi)
        print(', '.join(str(x*180/np.pi) for x in res.x))
    return res.x, x


def elbow_down(x0, orient=None, printing=False):
    global x, y, z, orientation
    x, y, z = x0
    orientation = orient
    if orient is not None:
        res = least_squares(equations_orientation, (0, 100 * np.pi / 180, -50 * np.pi / 180, -50 * np.pi / 180))
    else:
        res = least_squares(equations, (0, 0, -90 * np.pi / 180, 0))
    x = FK(res.x)
    if printing:
        print(res.cost)
        print(FK(res.x), sum([res.x[1], res.x[2], res.x[3]]) * 180 / np.pi)
        print(', '.join(str(x*180/np.pi) for x in res.x))
    return res.x, x


if __name__ == '__main__':
    x0 = (0.1, 0.1, 0.1)
    elbow_down(x0, orient=90/180*np.pi, printing=True)