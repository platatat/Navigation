"""
Contains kalman filter functions that take in raw data and output filtered data.
Originally translated from MATLAB. Math used is explained in great detail at:
https://home.wlu.edu/~levys/kalman_tutorial/
"""

import numpy as np

def kalman_retro(state):
    """Kalman filter that is used to retroactively filter previously collected
    gps data."""
    
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    state = np.matrix(state)
    rows = state.shape[0]

    x_pos = state[:,0]
    y_pos = state[:,1]
    yaw = state[:,2]
    v = state[:,3]
    t_step = state[:,4]
    
    v_0 = v.item(0)
    yaw_0 = yaw.item(0)
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    s_current = np.matrix([[x_pos.item(0)], [y_pos.item(0)], [x_dot_0], [y_dot_0]])
    
    P_current = eye4
    R = np.matrix([[6.25, 0, 0, 0], [0, 6.25, 0, 0], [0,0,100,0], [0,0,0,100]])
    C = eye4
 
    kalman_state = np.matrix(np.zeros((4,rows), dtype=float))
    
    
    for i in range(rows):
        
        A = np.matrix([[1, 0 , t_step.item(i)/1000., 0], [0, 1, 0, t_step.item(i)/1000.], [0, 0, 1, 0], [0, 0, 0, 1]]) 
        
        kalman_state[:, i] = s_current
        
        s_new = A*s_current
        P_new = A*P_current*A.T + (np.matrix([[.0001, 0, 0 ,0], [0, .0001, 0, 0], [0,0,.0001,0], [0,0,0,.0001]]))
        
    
        x_actual = state.item(i, 0)
        y_actual = state.item(i, 1)
        psi_actual = state.item(i, 2)
        v_actual = state.item(i, 3)
        x_dot_actual = v_actual*np.cos(psi_actual)
        y_dot_actual = v_actual*np.sin(psi_actual)

        z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
        
        G = P_new*C.T*((C*P_new*C.T + R).I)
        s_new = s_new + G*(z - C*s_new)
        P_new = (eye4 - G*C)*P_new
        
        s_current = s_new
        P_current = P_new
        
    return kalman_state.T

def kalman_real_time(state, s_current, P_current):
    """Kalman filter that can be run in real time. state is a 5x1 matrix of raw
    x, y, yaw, velocity, and time step data. s_current is the kalman state, usually taken from the
    previous call to this function, and P_current is the observation, also taken from
    previous call to function.
    
    Returns the new s and P after filter has been run, which can be used in the next call to the function
    as its s_current and P_current
    """
    
    #4x4 identity matrix
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    #make sure state is a matrix
    state = np.matrix(state)

    x_pos = state[:,0]
    y_pos = state[:,1]
    yaw = state[:,2]
    v = state[:,3]
    t_step = state[:,4]
    
    v_0 = v.item(0) # initial velocity
    yaw_0 = yaw.item(0) #initial yaw
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    
    A = np.matrix([[1, 0 , t_step/1000., 0], [0, 1, 0, t_step/1000.], [0, 0, 1, 0], [0, 0, 0, 1]])
    C = eye4 
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