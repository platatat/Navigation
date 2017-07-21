import numpy as np

def kalman_no_loop(state, C):
    """Kalman filter from MATLAB. Input is matrix of gps data, output is the Kalman matrix"""
    
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
    R = 4*eye4
 
    kalman_state = np.matrix(np.zeros((4,rows), dtype=float))
    
    
    for i in range(rows):
        
        A = np.matrix([[1, 0 , t_step.item(i)/1000., 0], [0, 1, 0, t_step.item(i)/1000.], [0, 0, 1, 0], [0, 0, 0, 1]]) 
        
        kalman_state[:, i] = s_current
        
        s_new = A*s_current
        P_new = A*P_current*A.T + eye4
        
    
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

if __name__ == '__main__':
    C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
    state = np.matrix([[1,2,3,4],[5,6,7,8],[9,10,1.1,0]])
    print kalman_no_loop(state,C)
    
