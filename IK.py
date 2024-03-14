import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

# Define parameters
l0 = 0.041     # m
l1 = 0.025     # m
l1x = 0.01     # m
l2 = 0.09525   # m (3.75”)
l3 = 0.10795   # m (4.25”)
l4 = 0.085725  # m (3.375”)

x, y, z = (0, 0, 0.07)

orientation = np.pi/2
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


if __name__ == '__main__':
    res = least_squares(equations, (0, 0, 90 * np.pi / 180, 0), bounds=(
    (-np.pi / 2, -np.pi / 3, np.pi / 2, -np.pi / 2), (np.pi / 2, np.pi / 3, 160 * np.pi / 180, np.pi / 2)))
    print(res.x*180/np.pi)
    print(FK(res.x))

