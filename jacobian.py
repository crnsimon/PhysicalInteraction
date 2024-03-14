import numpy as np

# Define parameters
l0 = 0.041     # m
l1 = 0.025     # m
l1x = 0.01     # m
l2 = 0.09525   # m (3.75”)
l3 = 0.10795   # m (4.25”)
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
