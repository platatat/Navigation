# Navigation

## Overview

This repo contains code that relates to navigation, including the navigation algorithm itself, nodes for ROS communication, a bike simulation, GPS visualization scripts, web path retrieval, and data smoothing/sensor fusion. 

The goal of the code contained in this repo is to allow the bike to use various sensors to autonomously follow a predefined path as accurately as possible.

Before getting into the details of each file below, here are some important things to know right off the bat:

* The file titled **nav.py** contains the main navigation algorithms. Our algorithm takes in a "bike state", which contains info about the bike at its current state (such as location or speed), along with the predifined set of waypoints that make up the path, and it outputs a desired front wheel angle. This output angle should help steer the bike correctly along/towards the path.

* The file titled **loop.py** contains the code for running the bike simulation. The initial bike state and waypoints can be edited to test out different scenarios. Simply run this file to see the simulation in action.

* All files with "_node" are used for ROS communication. Generally, these files rely on bike sensors like GPS or IMU to publish and/or subscribe to other ROS nodes. ROS can be somewhat confusing to understand at first -- for more information see the first 4-5 chapters of  [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/).

* All files with "kalman" relate to data smoothing/sensor fusion. As of now, these files use something similar to a kalman filter to get more accurate location data. The math behind the kalman filter is explained in detail [here](https://home.wlu.edu/~levys/kalman_tutorial/).

 ## Known Bugs

---
## Documentation
* [bikeSim.py](#bikeSim)
* [bikeState.py](#bikeState)
* [constants.py](#constants)
* [geometry.py](#geometry)
* [gps_assisted_simulator_node.py](#gps_assisted_simulator_node)
* [kalman.py](#kalman)
* [kalman_driver.py](#kalman_driver)
* [kalman_node.py](#kalman_node)
* [loop.py](#loop)
* [mapModel.py](#mapModel)
* [map_node.py](#map_node)
* [nav.py](#nav)
* [navigation_node.py](#navigation_node)
* [requestHandler.py](#requestHandler)
* [simulator_node.py](#simulator_node)
* [vis_node.py](#vis_node)

All files with "_node" are used for ROS communication.

### <a name="bikeSim"></a> bikeSim.py
Module that simulates the bikes dynamics.

Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
Bike object| new_state(bike, steerD)| Returns: next bikeState given current state [bike] and direction [steerD]

---

### <a name="bikeState"></a> bikeState.py
Defines the Bike object which represents the autonomous bike for the purposes of navigation. Not to be confused with Bike_State class in Arduino code.

  
  Class Definition:

Type | Instance Attribute | Description
:-------------: |:-------------:| :-----:
float | xB | Bike position's x-coordinate
float | yB | Bike position's y-coordinate
float | phi | Lean angle (radians)
float | psi | Direction/heading/yaw (radians)
float | delta | Steer angle (radians)
float | w_r | Lean rate (m/s)
float | v | Speed (m/s)
  
  Methods: //TODO

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
array? | vector(self) | Returns the bike's unit vector
void | update(self, bike) | Updates the bike state
array? | rhs(self, steerD) | Modifies state object to turn it into the next state using bike dynamics. Equivalent to rhs in Matlab


---

### <a name="constants"></a> constants.py
Definitions of various constants for Navigation code. //TODO
  
Constant Names | Significance | Used In
:-------------: |:-------------:| :-----:
G, L, B, H | Constants from the Matlab bike dynamics simulation | bikeSim.py
C | Trail? | ?
K1 | Gain from old nav algorithm | Not used
K2 | Gain from old nav algorithm | Not used
K3 | Gain from old nav algorithm | Not used
TIMESTEP | ? | ?
PAUSE | ? | ?
MAX_Steer| Bike's maximumum steer angle, used in clamping steerD | nav.py
PID_DIST_GAIN | Gain used for distance contribution of PID | nav.py
PID_ANGLE_GAIN | Gain used for angle contribution of PID | Not used
RAD_TO_DEG | Ratio of radians to degrees | nav.py
TURN_LOOKAHEAD_DIST | A lookahead distance | Not used
NEXT_TURN_GAIN | Gain used for next turn contribution of PID| nav.py
BIKE_LENGTH | Bike length used in quintic | nav.py
QUINTIC_LOOKAHEAD | Lookahead distance used in quintic | nav.py
QUINTIC_SAMPLE_LENGTH | Constant used in quintic polynomial | nav.py
BASE_LOOKAHEAD | ? | Not used
LOOKAHEAD_ANGLE_GAIN| Gain used for lookahead angle | Not used
MIN_TURN_RADIUS| Bike's minimum turning radius, determined using MAX_STEER | nav.py
ANIM_INTERVAL| Miliseconds between blit frames | loop.py

---
### <a name="geometry"></a> geometry.py
Module that contains many convenience functions that perform mathematical calculations. Used in __

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
array? | unit_vector(p1, p2) |  Returns: the unit vector given 2D points [p1] and [p2]
array? | threeD_unit_vector(p1, p2) | Returns: the unit vector given points [p1] and [p2]
float? | dist2(p1, p2) |  Returns: the square of the distance between [p1] and [p2]
float? | distance(p1, p2) | Returns: the distance between points [p1] and [p2]
int? | get_sign(v) | Returns: the sign of the number [v]
array? | nearest_point_on_path(path, point) | Returns: the nearest point on the [path] to the [point]
float? | angle_from_path(bike_direction, p1, p2) | Returns: angle that the bicycle has to turn to approach line [p1] to [p2]
float? | distance_from_path(point, target_path) | Returns: calculates the distance from [point] to [target_path]
float? | line_slope(line) | Returns: slope of [line]
float? | angle_between_vectors(v1, v2) | Returns: angle between vectors [v1] and [v2]. Used in angle_between_two_lines
float? | angle_between_two_lines(line1, line2) | Returns: angle between [line1] and [line2]
float? | dot_product(v1, v2) | Returns: mathematical dot product of vectors [v1] and [v2]
array? | intersect_circle_path(path, r, c) | Returns: points of intersection between [path] and circle with radius [r] and center [c]

---
### <a name="gps_assisted_simulator_node"></a> gps_assisted_simulator_node.py
This file is simply simulator_node.py, but it reads from the GPS ROS stream and updates its internal state based on that.

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_bike_from_gps(data) | Takes the incoming data from the GPS and updates our state with it
void | update_graph(data) | 
void | path_parse(data) |
void | listener() |

---
### <a name="kalman"></a> kalman.py
Contains kalman filter functions that take in raw data and output filtered data.
Originally translated from MATLAB. Math used is explained in great detail at:
https://home.wlu.edu/~levys/kalman_tutorial/


Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
np.matrix | kalman_retro(state) | Kalman filter that is used to retroactively filter previously collected raw data.
np.matrix array | kalman_real_time(state, s_current, P_current) | Kalman filter that can be run in real time to filter incoming raw data.		
---
---
### <a name="loop"></a> loop.py
Main Navigation file. Runs a loop that gets navigation command, passes it thorugh simulation, and gets the new updated state.

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | loop_pyqtgraph(nav, bike, map_model) | Animation using PyQtGraph
void | loop_matplotlib(nav, bike, map_model) | Animation using Matplotlib with adding/removing patches
void | loop_matplotlib_blitting(nav, bike, map_model, blitting=True) | Animation using Matplotlib with blitting and callbacks
dict? | find_display_bounds(waypoints) | Given a set of waypoints, return {xlim, ylim} that can fit them on a graph

			
---

### <a name="mapModel"></a> mapModel.py
Virtual model of the map in which the simulated bike is navigating. Every Route is made up of paths, which are lines between two points.


Class Definition:

Type | Instance Attribute | Description
:-------------: |:-------------:| :-----:
Bike | bike | Bike object
array | paths | Array of path segments. Each element is an array of 2 waypoints that make up a path segment.
array | waypoints | Waypoints that make up the route
array? | obstacles | Obstacles on the map?

Methods:

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
void | init_paths(self, waypoints) | Initializes paths fron input waypoints
void | add_path(self, p1, p2) | Adds a new path from point p1 to point p2 at the end of the path list
void | add_point(self, p) | Adds a new point p to the list of waypoints. If it is not the first point, appends a new path
void | close_path(self)| Adds a path from the last point to the first point in the waypoints list
void | draw_circle(self, center, r, n_points, degrees = 2*np.pi) | Draws a circle with given characteristics

---
### <a name="nav"></a> nav.py
Navigation algorithms. Contains methods that return the desired steering angle given a bike state and list of paths.

Methods:

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
void | clamp_steer_angle(self, steerD) | Limits a steer angle to within [-MAX_STEER, +MAX_STEER]
int? | find_closest_path(self, point) | Returns the index of the nearest path to the given point from the list of paths (stored in self.map_model.paths)
float? | pid_controller(self) | Returns a desired steering angle for the bike. Two parts: a regular PID controller, and a component that detects when a turn is coming up so we can start turning early
float? | create_lookahead_correction(self, current_path, bike)| Looks at the path ahead and returns a steering angle correction
float? | quintic_steering_angle(self, dist_error, angle_error, curvature_error, L, s)| Returns the y-coordinate of the quintic polynomial given s as the x-coordinate
float? | quintic(self) | Returns a desired steering angle for the bike based on quintic algorithm
float? | get_steering_angle(self) | Calls another function to calculate the steering angle. This function is part of the external interface of this class. All external users of this class should call this method instead of the other steering-angle-calculation methods.


---
### <a name="requestHandler"></a> requestHandler.py
File that handles obtaining GPS waypoints from website. Users can submit waypoints on a Google Maps model on the website, and this file makes requests to the website to obtain those waypoints.

Team members can sign in and submit waypoints at: https://abserver-168813.appspot.com

Functions: //TODO

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
array? | convert(latitude, longitude) | Mercator map projection
array? | convert2(latitude, longitude) | 
array? | math_convert(latitude, longitude) | Converts latitude and longitude to local coordinates that bike can use to navigate.
void | bearing_from_origin(origin, latitude, longitude)| potato
void | parse_json(presets=False) | Parses JSON response from http request


---
### <a name="navigation_node"></a> navigation_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | callback(data) | 
void | path_parse(data) |
void | update_bike_state(data) |
void | update_bike_xy(data) |
void | keyboard_update(data) |
void | talker() |


---
### <a name="map_node"></a> map_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
array? | setup_dimension() | 
void | map_server() |


---
### <a name="simulator_node"></a> simulator_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_graph(data) |
void | path_parse(data) |
void | listener() | 

---
### <a name="vis_node"></a> vis_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_graph(data) |
void | path_parse(data) |
void | listener() | 

