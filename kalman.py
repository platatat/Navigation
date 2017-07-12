#!/usr/bin/env python
import numpy as np
import math

def kalman_no_loop(state):
    
    state = np.matrix(state)
    x_pos = state[:,0]
    y_pos = state[:,1]
    phi = state[:,2]
    yaw = state[:,3]
    delta = state[:,4]
    phi_dot = state[:,5]
    v = state[:,6]
    
    #state_table = table(...)
    
    v_0 = v(0)
    yaw_0 = yaw(0)
    x_dot0 = v_0*math.cos(yaw_0)
    y_dot0 = v_0*math.sin(yaw_0)
    s_current = [x_pos(0), y_pos(0), x_dot_0, y_dot_0]
    
    t_step = 1/50.
    A = np.matrix([[1, 0 , t_step, 0], [0, 1, 0, t_step], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    C = np.matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    P_current = np.matrix([1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1])
    R = np.matrix([4, 0, 0, 0], [0, 4, 0, 0], [0, 0, 4, 0], [0, 0, 0, 4])
    r = state.shape[0]
    c = state.shape[1]
    kalman_state = np.zeros((4,r), dtype=int)
    
    for i in range(len(x_pos)):
        
        #kalman_state(:, i) = s_current
        
        s_new = A*s_current
        P_new = A*P_current*A.T
        
        x_actual = state(i, 0)
        y_actual = state(i, 1)
        psi_actual = state(i, 3)
        v_actual = state(i, 6)
        x_dot_actual = v_actual*math.cos(psi_actual)
        y_dot_actual = v_actual*math.sin(psi_actual)

        
        
        
    
