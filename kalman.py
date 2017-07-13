#!/usr/bin/env python
import numpy as np
import math

def kalman_no_loop(state):
    
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    state = np.matrix(state)
    rows = state.shape[0]
    cols = state.shape[1]
    
    if cols == 4:
        x_pos = state[:,0]
        y_pos = state[:,1]
        yaw = state[:,2]
        v = state[:,3]
        
    elif cols == 7:
        x_pos = state[:,0]
        y_pos = state[:,1]
        phi = state[:,2]
        yaw = state[:,3]
        delta = state[:,4]
        phi_dot = state[:,5]
        v = state[:,6]
    
    #state_table = table(...)
    
    v_0 = v.item(0)
    yaw_0 = yaw.item(0)
    x_dot_0 = v_0*math.cos(yaw_0)
    y_dot_0 = v_0*math.sin(yaw_0)
    s_current = np.matrix([[x_pos.item(0)], [y_pos.item(0)], [x_dot_0], [y_dot_0]])
    
    t_step = 1/50.
    A = np.matrix([[1, 0 , t_step, 0], [0, 1, 0, t_step], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    C = np.matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    P_current = eye4
    R = 4*eye4
 
    kalman_state = np.matrix(np.zeros((4,rows), dtype=float))
    
    
    for i in range(len(x_pos)):
        
        kalman_state[:, i] = s_current
        
        s_new = A*s_current
        P_new = A*P_current*A.T
        
        if cols ==7:
            x_actual = state.item(i, 0)
            y_actual = state.item(i, 1)
            psi_actual = state.item(i, 3)
            v_actual = state.item(i, 6)
        elif cols == 4:
            x_actual = state.item(i, 0)
            y_actual = state.item(i, 1)
            psi_actual = state.item(i, 2)
            v_actual = state.item(i, 3)
        x_dot_actual = v_actual*math.cos(psi_actual)
        y_dot_actual = v_actual*math.sin(psi_actual)

        z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
        
        G = P_new*C.T*((C*P_new*C.T + R).I)
        s_new = s_new + G*(z - C*s_new)
        P_new = (eye4 - G*C)*P_new
        
        s_current = s_new
        P_current = P_new
        
    return kalman_state.T   
        
        
    
