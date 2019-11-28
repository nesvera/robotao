import numpy as np

" Homogeneous transformation matrices"
theta_1 = 0.90
theta_2 = 0.78
theta_3 = 0.13
theta_4 = -0.55
theta_5 = 0
theta_6 = 0.0

H_T1 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0.14],
                 [0, 0, 1, -0.255],
                 [0, 0, 0, 1]])

H_R1 = np.array([[np.cos(theta_1),  -np.sin(theta_1),   0,  0],
                 [np.sin(theta_1),  np.cos(theta_1),    0,  0],
                 [0,                0,                  1,  0],
                 [0,                0,                  0,  1]])

H_R2 = np.array([[1, 0,                 0,                  0],
                 [0, np.cos(theta_2),   -np.sin(theta_2),   0],
                 [0, np.sin(theta_2),   np.cos(theta_2),    0],
                 [0, 0,                 0,                  1]])

H_R3 = np.array([[np.cos(theta_3),  0, np.sin(theta_3), 0],
                 [0,                1, 0,               0],
                 [-np.sin(theta_3), 0, np.cos(theta_3), 0],
                 [0,                0, 0,               1]])

H_T2 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0.05],
                 [0, 0, 1, -0.56],
                 [0, 0, 0, 1]])

H_R4 = np.array([[np.cos(theta_4),  0, np.sin(theta_4), 0],
                 [0,                1, 0,               0],
                 [-np.sin(theta_4), 0, np.cos(theta_4), 0],
                 [0,                0, 0,               1]])

H_T3 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, -0.63],
                 [0, 0, 0, 1]])

H_R5 = np.array([[np.cos(theta_5),  0, np.sin(theta_5), 0],
                 [0,                1, 0,               0],
                 [-np.sin(theta_5), 0, np.cos(theta_5), 0],
                 [0,                0, 0,               1]])

H_R6 = np.array([[1, 0,                 0,                  0],
                 [0, np.cos(theta_6),   -np.sin(theta_6),   0],
                 [0, np.sin(theta_6),   np.cos(theta_6),    0],
                 [0, 0,                 0,                  1]])

H_T4 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, -0.08],
                 [0, 0, 0, 1]])

if __name__ == "__main__":

    v_e = np.array([0, 0, 0, 1])

    t_body2end = np.matmul(H_T1, H_R1)
    t_body2end = np.matmul(t_body2end, H_R2)
    t_body2end = np.matmul(t_body2end, H_R3)
    t_body2end = np.matmul(t_body2end, H_T2)
    t_body2end = np.matmul(t_body2end, H_R4)
    t_body2end = np.matmul(t_body2end, H_T3)
    t_body2end = np.matmul(t_body2end, H_R5)
    t_body2end = np.matmul(t_body2end, H_R6)
    #t_body2end = np.matmul(t_body2end, H_T4)

    p_e = np.array([0, 0, 0, 1])
    eff_pos = np.matmul(t_body2end, np.transpose(p_e))

    p2_e = np.array([0, 0, 1, 1])
    eff2_pos = np.matmul(t_body2end, np.transpose(p2_e))

    v_eff = eff2_pos-eff_pos

    with np.printoptions(precision=3, suppress=True):
        print(eff_pos)
        print(v_eff)

    pass

"""
First vesion, not modified 

H_T1 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0.14],
                 [0, 0, 1, -0.255],
                 [0, 0, 0, 1]])

H_R1 = np.array([[np.cos(theta_1),  0, np.sin(theta_1), 0],
                 [0,                1, 0,               0],
                 [-np.sin(theta_1), 0, np.cos(theta_1), 0],
                 [0,                0, 0,               1]])

H_R2 = np.array([[1, 0,                 0,                  0],
                 [0, np.cos(theta_2),   -np.sin(theta_2),   0],
                 [0, np.sin(theta_2),   np.cos(theta_2),    0],
                 [0, 0,                 0,                  1]])

H_R3 = np.array([[np.cos(theta_3),  -np.sin(theta_3),   0,  0],
                 [np.sin(theta_3),  np.cos(theta_3),    0,  0],
                 [0,                0,                  1,  0],
                 [0,                0,                  0,  1]])

H_T2 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0.05],
                 [0, 0, 1, -0.56],
                 [0, 0, 0, 1]])

H_R4 = np.array([[np.cos(theta_4),  0, np.sin(theta_4), 0],
                 [0,                1, 0,               0],
                 [-np.sin(theta_4), 0, np.cos(theta_4), 0],
                 [0,                0, 0,               1]])

H_T3 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, -0.63],
                 [0, 0, 0, 1]])

H_R5 = np.array([[np.cos(theta_5),  0, np.sin(theta_5), 0],
                 [0,                1, 0,               0],
                 [-np.sin(theta_5), 0, np.cos(theta_5), 0],
                 [0,                0, 0,               1]])

H_R6 = np.array([[1, 0,                 0,                  0],
                 [0, np.cos(theta_6),   -np.sin(theta_6),   0],
                 [0, np.sin(theta_6),   np.cos(theta_6),    0],
                 [0, 0,                 0,                  1]])

H_T4 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, -0.08],
                 [0, 0, 0, 1]])

"""