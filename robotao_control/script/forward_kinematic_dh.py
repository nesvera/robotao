import numpy as np

" Homogeneous transformation matrices"
theta_1 = -np.pi/2
theta_2 = 0
theta_3 = np.pi/2
theta_4 = np.pi/2
theta_5 = 0
theta_6 = 0.0

#                     theta,                alpha,      d,       r
dh_param = np.array([[0,                    -np.pi/2,   -0.255,  0],
                     [(np.pi/2)+theta_1,    np.pi/2,    0.14,    0],
                     [(np.pi/2)+theta_2,    np.pi/2,    0,       0],
                     [(np.pi/2)+theta_3,    np.pi/2,    +0.56,   0.05],
                     [(np.pi/2)+theta_4,    0,          0.,      0.63],
                     [theta_5,              np.pi/2,    0,       0],
                     [(np.pi/2)+theta_6,    -np.pi/2,   0,       0]])

"""
                     [-np.pi/2,             0,          -0.08,   0]]) 
"""

t_body2end = None

for i in range(dh_param.shape[0]):
    
    dh_mat = np.array([[np.cos(dh_param[i,0]), -np.sin(dh_param[i,0])*np.cos(dh_param[i,1]),   np.sin(dh_param[i,0])*np.sin(dh_param[i,1]),    dh_param[i,3]*np.cos(dh_param[i,0])],
                       [np.sin(dh_param[i,0]), np.cos(dh_param[i,0])*np.cos(dh_param[i,1]),    -np.cos(dh_param[i,0])*np.sin(dh_param[i,1]),   dh_param[i,3]*np.sin(dh_param[i,0])],
                       [0,                     np.sin(dh_param[i,1]),                          np.cos(dh_param[i,1]),                          dh_param[i,2]],
                       [0,                     0,                                              0,                                              1]])

    if t_body2end is None:
        t_body2end = dh_mat

    else:
        t_body2end = np.matmul(t_body2end, dh_mat)

p_e = np.array([0, 0, 0, 1])
eff_pos = np.matmul(t_body2end, np.transpose(p_e))

p2_e = np.array([0, 0, 1, 1])
eff2_pos = np.matmul(t_body2end, np.transpose(p2_e))

v_eff = eff2_pos-eff_pos

with np.printoptions(precision=3, suppress=True):
    print(eff_pos)
    print(v_eff)

if __name__ == "__main__":
    pass