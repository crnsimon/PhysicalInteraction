import matplotlib.pyplot as plt
import numpy as np
from IK import inverse_kinematics

sim = False

if sim:
    # Define parameters
    l0 = 0.05  # m
    l1 = 0.02  # m
    l1x = 0  # m
    l2 = 0.0825  # m
    l3 = 0.086  # m
    l4 = 0.075  # m
else:
    # Define parameters
    l0 = 0.045  # m
    l1 = 0.019  # m
    l1x = 0.013  # m
    l2 = 0.09525  # m (3.75”)
    l3 = 0.10795  # m (4.25”)
    l4 = 0.085725  # m (3.375”)


def jacobian(qs):
    q0, q1, q2, q3 = qs
    xq0 = -np.sin(q0)*(l4*np.sin(q1+q2+q3)+l3*np.sin(q1+q2)+l2*np.sin(q1)+l1x)
    xq1 = np.cos(q0)*(l4*np.cos(q1+q2+q3)+l3*np.cos(q1+q2)+l2*np.sin(q1))
    xq2 = np.cos(q0)*(l4*np.cos(q1+q2+q3)+l3*np.cos(q1+q2))
    xq3 = np.cos(q0)*(l4*np.cos(q1+q2+q3))
    yq0 = np.cos(q0)*(l4*np.sin(q1+q2+q3)+l3*np.sin(q1+q2)+l2*np.sin(q1)+l1x)
    yq1 = np.sin(q0)*(l4*np.cos(q1+q2+q3)+l3*np.cos(q1+q2)+l2*np.sin(q1))
    yq2 = np.sin(q0)*(l4*np.cos(q1+q2+q3)+l3*np.cos(q1+q2))
    yq3 = np.sin(q0)*(l4*np.cos(q1+q2+q3))
    zq0 = 0
    zq1 = -(l4*np.sin(q1+q2+q3)+l3*np.sin(q1+q2)+l2*np.sin(q1))
    zq2 = -(l4*np.sin(q1+q2+q3)+l3*np.sin(q1+q2))
    zq3 = (l4 * np.sin(q1 + q2 + q3))
    oq0 = 0
    oq1 = 1
    oq2 = 1
    oq3 = 1
    J = np.array([[xq0, xq1, xq2, xq3], [yq0, yq1, yq2, yq3], [zq0, zq1, zq2, zq3], [oq0, oq1, oq2, oq3]])
    return J

def const_v(v, x0, orientation, T, dt):
    qs = []
    err = []
    i = 0
    for t in np.arange(0, T, dt):
        x = x0 + np.array(v)*t
        q, x_res = inverse_kinematics(x, orient=orientation[i], printing=False)
        qs.append(q)
        err.append(sum((x_res-x)**2)*10000)
        print(x_res, x)
        i += 1
    return qs, err

def plot_results(qs, err, T, dt):
    qs = np.array(qs) * 180 / np.pi
    t_arr = np.arange(0, T, dt)
    for i in range(4):
        plt.plot(t_arr, qs[:, i], label=f"q{i}")
    plt.plot(t_arr, err, label=f"Error", linestyle='--')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    T = 7
    dt = 0.01
    orientation = [np.pi/2 for x in np.arange(0, T, dt)]
    qs, err = const_v((-0.02, 0, 0.02),(0.15, 0.15, 0.08), orientation, T, dt)
    plot_results(qs, err, T, dt)

