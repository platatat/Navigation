#!/usr/bin/env python
import numpy as np

def kalman_no_loop(state, C, s_current, P_current):
    """Kalman filter from MATLAB. Input is matrix of gps data, output is the Kalman matrix"""
    
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    state = np.matrix(state)

    x_pos = state[:,0]
    y_pos = state[:,1]
    yaw = state[:,2]
    v = state[:,3]
    t_step = state[:,4]
        
    else:
        x_pos = state[:,0]
        y_pos = state[:,1]
        phi = state[:,2]
        yaw = state[:,3]
        delta = state[:,4]
        phi_dot = state[:,5]
        v = state[:,6]
    
    v_0 = v.item(0)
    yaw_0 = yaw.item(0)
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    
    A = np.matrix([[1, 0 , t_step/1000., 0], [0, 1, 0, t_step/1000.], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    R = 4*eye4
 
    s_new = A*s_current
    P_new = A*P_current*A.T + eye4


    x_actual = state.item(0)
    y_actual = state.item(1)
    psi_actual = state.item(2)
    v_actual = state.item(3)
    x_dot_actual = v_actual*np.cos(psi_actual)
    y_dot_actual = v_actual*np.sin(psi_actual)

    z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
        
    G = P_new*C.T*((C*P_new*C.T + R).I)
    s_new = s_new + G*(z - C*s_new)
    P_new = (eye4 - G*C)*P_new
        
    return (s_new, P_new)

# if __name__ == '__main__':
#     a = [[1,2,3,4]]
#     a = np.matrix(a)
#     P_current = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#     x_pos = a[:,0]
#     y_pos = a[:,1]
#     yaw = a[:,2]
#     v = a[:,3]
#     v_0 = v.item(0)
#     v_0 = v.item(0)
#     yaw_0 = yaw.item(0)
#     x_dot_0 = v_0*np.cos(yaw_0)
#     y_dot_0 = v_0*np.sin(yaw_0)
#     s_current = np.matrix([[x_pos.item(0)], [y_pos.item(0)], [x_dot_0], [y_dot_0]])
#     C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
#     ak = kalman_no_loop(a, C, s_current, P_current)
#     b = [[5,6,7,8]]
#     bk = kalman_no_loop(b, C, ak[0], ak[1])
#     c = [[9,10,1.1,0]]
#     ck = kalman_no_loop(c, C, bk[0], bk[1])
#     #print ak
#     print s_current.T
#     print ak[0].T
#     print bk[0].T
    

        
    
