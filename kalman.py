"""
Contains kalman filter functions that take in raw data and output filtered data.
Originally translated from MATLAB. Math used is explained in great detail at:
https://home.wlu.edu/~levys/kalman_tutorial/
"""

import numpy as np
import geometry


def kalman(sensors, dt=10e6):
    # Get start and end time.
    start_time = sensors.gps.timestamp[0]
    end_time = sensors.gps.timestamp[-1]

    # Keep track of index into sensor arrays so it's easier to know when the simulation time
    # passes though a sensor data point timestamp.
    gps_index = 0

    # Get initial state. State vector is (x, y, x_dot, y_dot).
    x_init = sensors.gps.x[0]
    y_init = sensors.gps.y[0]
    xdot_init = sensors.gps.speed[0] * np.cos(sensors.gps.yaw[0])
    ydot_init = sensors.gps.speed[0] * np.sin(sensors.gps.yaw[0])
    state = np.matrix([[x_init], [y_init], [xdot_init], [ydot_init]])

    # State estimate covariance matrix.
    P = np.identity(4)

    # Sensor covariance matrix.
    R = np.matrix([[6.25, 0, 0, 0], [0, 6.25, 0, 0], [0,0,100,0], [0,0,0,100]])

    # TODO: not sure what this one does...
    C = np.identity(4)

    kalman_state = []

    # Run the filter.
    for t in np.arange(start_time, end_time, dt):
        # State update matrix.
        A = np.identity(4)
        A[0, 2] = dt / 1.e9
        A[1, 3] = dt / 1.e9

        # Predict new state.
        # TODO: why do we need to add the identity here?
        state = A * state
        P = A * P * A.T + np.identity(4) * (dt / 1.e9)

        # Check for sensor data during this timestep.
        if sensors.gps.timestamp[gps_index] < t:
            gps_index += 1

            # Update state using sensor observations.
            x_observed = sensors.gps.x[gps_index]
            y_observed = sensors.gps.y[gps_index]
            yaw_observed = sensors.gps.yaw[gps_index]
            speed_observed = sensors.gps.speed[gps_index]
            xdot_observed = speed_observed * np.cos(yaw_observed)
            ydot_observed = speed_observed * np.sin(yaw_observed)

            z = np.matrix([[x_observed], [y_observed], [xdot_observed], [ydot_observed]])

            # Compute kalman gain.
            G = P * C.T * (C * P * C.T + R).I
            state += G * (z - C * state)
            P = (np.identity(4) - G * C) * P

            # Catch up to most recent sensor measurement.
            while sensors.gps.timestamp[gps_index] < t:
                gps_index += 1
                print 'Skipping sensor data, timestep might be too small.'

        kalman_state.append(state)

    return np.array(kalman_state)



def kalman_retro(raw_state):
    """Kalman filter that is used to retroactively filter previously collected
    raw data. Input raw_state is a 5x1 matrix of raw x, y, yaw, velocity, and time step data.
    
    Returns kalman state, which is a 4x1 matrix of filtered x, y, x_dot, and y_dot data.
    """
    
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    raw_state = np.matrix(raw_state)
    rows = raw_state.shape[0]

    x_pos = raw_state[:,0]
    y_pos = raw_state[:,1]
    yaw = raw_state[:,2]
    v = raw_state[:,3]
    t_step = raw_state[:,4]
    
    v_0 = v.item(0)
    yaw_0 = yaw.item(0)
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    s_current = np.matrix([[x_pos.item(0)], [y_pos.item(0)], [x_dot_0], [y_dot_0]])
    
    P_current = eye4
    #Try this for later tuning
    R = np.matrix([[6.25, 0, 0, 0], [0, 6.25, 0, 0], [0,0,100,0], [0,0,0,100]])
    C = eye4
 
    kalman_state = np.matrix(np.zeros((4,rows), dtype=float))
    
    old_xy = (raw_state.item(0,0), raw_state.item(0,1))
    for i in range(rows):
        
        A = np.matrix([[1, 0 , t_step.item(i)/1000., 0], [0, 1, 0, t_step.item(i)/1000.], [0, 0, 1, 0], [0, 0, 0, 1]]) 
        
        kalman_state[:, i] = s_current
        
        s_new = A*s_current
        P_new = A*P_current*A.T + (np.matrix([[.0001, 0, 0 ,0], [0, .0001, 0, 0], [0,0,.0001,0], [0,0,0,.0001]]))
        
        #Check for outliers
        if geometry.distance((raw_state.item(i,0),raw_state.item(i,1)), old_xy) < 20:
            x_actual = raw_state.item(i, 0)
            y_actual = raw_state.item(i, 1)
            old_xy = (x_actual, y_actual)
        else:
            x_actual = old_xy[0]
            y_actual = old_xy[1]
        
        psi_actual = raw_state.item(i, 2)
        v_actual = raw_state.item(i, 3)
        x_dot_actual = v_actual*np.cos(psi_actual)
        y_dot_actual = v_actual*np.sin(psi_actual)

        z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
        
        G = P_new*C.T*((C*P_new*C.T + R).I)
        s_new = s_new + G*(z - C*s_new)
        P_new = (eye4 - G*C)*P_new
        
        s_current = s_new
        P_current = P_new
        
    return kalman_state.T

def kalman_real_time(raw_state, s_current, P_current):
    """
    Kalman filter that can be run in real time. Input raw_state is a 5x1
    matrix of raw x, y, yaw, velocity, and time step data. s_current is
    the kalman state, usually taken from the previous call to this
    function, and P_current is the observation, also taken from previous
    call to function.

    Returns the new s and P after filter has been run, which can be used
    in the next call to the function as its s_current and P_current. The
    first entry of the tuple is the Kalman state, as a row matrix (x, y,
    x_dot, y_dot). The second entry of the tuple is the prediction
    error, as a 4x4 square matrix.
    """

    #4x4 identity matrix
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    #make sure raw_state is a matrix
    raw_state = np.matrix(raw_state)

    x_pos = raw_state[:,0]
    y_pos = raw_state[:,1]
    yaw = raw_state[:,2]
    v = raw_state[:,3]
    t_step = raw_state[:,4]
    
    v_0 = v.item(0) # initial velocity
    yaw_0 = yaw.item(0) #initial yaw
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    
    A = np.matrix([[1, 0 , t_step/1000., 0], [0, 1, 0, t_step/1000.], [0, 0, 1, 0], [0, 0, 0, 1]])
    C = eye4 
    R = 4*eye4
 
    s_new = A*s_current
    P_new = A*P_current*A.T + (np.matrix([[.0001, 0, 0 ,0], [0, .0001, 0, 0], [0,0,.0001,0], [0,0,0,.0001]]))
    
    #Check for outliers
    old_xy = (s_current.item(0), s_current.item(1))
    if geometry.distance((raw_state.item(0),raw_state.item(1)), old_xy) < 20:
        x_actual = raw_state.item(0)
        y_actual = raw_state.item(1)
    else:
        x_actual = old_xy[0]
        y_actual = old_xy[1]
        
    psi_actual = raw_state.item(2)
    v_actual = raw_state.item(3)
    x_dot_actual = v_actual*np.cos(psi_actual)
    y_dot_actual = v_actual*np.sin(psi_actual)

    z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
    
    
    G = P_new*C.T*((C*P_new*C.T + R).I)
    s_new = s_new + G*(z - C*s_new)
    P_new = (eye4 - G*C)*P_new
        
    return (s_new, P_new)
