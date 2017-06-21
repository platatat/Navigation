# nav.py
import numpy as np
import math
import sys
import geometry
import bikeState
#imported bikSim to be able to calculate overshoots distance
import bikeSim
import mapModel
from constants import *

class Nav(object):


	def __init__(self, map_model):
		""" Nav initializer """
		self.map_model = map_model
		self.target_path = 0

		# Used by loop.py for visualization
		self.lookahead_point = (0, 0)
		self.dropped_point = (0, 0)

		# State for turn lookaheads
		self.first_segment = True

		num_paths = len(self.map_model.paths)

		# Precompute when we need to turn
		self.lookahead_distances = [0] * num_paths
		self.lookahead_angles = [0] * num_paths # Holds angles between segments
		self.lookahead_factors = [0] * num_paths
		for index, curr_segment in enumerate(self.map_model.paths):

			# Compute the angle the current segment makes with the x-axis
			curr_vector = (curr_segment[1][0] - curr_segment[0][0], curr_segment[1][1] - curr_segment[0][1])
			curr_angle = math.atan2(curr_vector[1], curr_vector[0])

			# Compute the angle the next segment makes with the x-axis
			next_index = (index + 1) % len(self.map_model.paths)
			next_segment = self.map_model.paths[next_index]
			next_vector = (next_segment[1][0] - next_segment[0][0], next_segment[1][1] - next_segment[0][1])
			next_angle = math.atan2(next_vector[1], next_vector[0])

			# Find the angle between the current path segment and the next path segment
			angle_diff = next_angle - curr_angle

			# Fix range of angle (this happens if the path is right-to-left)
			if abs(angle_diff) > math.pi:
				angle_diff -= math.copysign(2 * math.pi, angle_diff)

			print("Segment {} to segment {}: {} degrees".format(index, index + 1 % num_paths, angle_diff * RAD_TO_DEG))
			self.lookahead_angles[index] = angle_diff

			# If the turn angle is less than 90 degrees, use the sin-based formula
			angle_diff = abs(angle_diff)
			curr_factor = 1
			if angle_diff < math.pi / 2:
				curr_factor = math.sin(angle_diff)
			else:
				curr_factor = math.tan((math.pi - angle_diff))
			self.lookahead_factors[index] = curr_factor
			curr_lookahead = MIN_TURN_RADIUS * curr_factor
			print("{} => {}".format(abs(angle_diff) * RAD_TO_DEG, curr_lookahead))
			self.lookahead_distances[index] = curr_lookahead

	def clamp_steer_angle(self, steerD):
		"""Limits a steer angle to within [-MAX_STEER, +MAX_STEER]"""
		if (steerD > MAX_STEER):
			steerD = MAX_STEER
		elif (steerD < -MAX_STEER):
			steerD = -MAX_STEER

		return steerD

	def direction_to_turn(self):
		""" Returns: the steer command """
		self.target_path = self.find_closest_path()
		bike_pos = (self.map_model.bike.xB, self.map_model.bike.yB)
		distance = np.abs(geometry.distance_from_path(bike_pos, self.map_model.paths[self.target_path]))
		delta = np.abs(self.displacement_to_turn())
		# delta = 7.81166015145 # undershoot of -0.808198581543 (setting delta equal to result of overshoot2)
		# delta = delta + 6.11776258705 # VALUE I calculated using my overshoot calculation functions (gives undershoot of -0.808198581543)
		delta = delta

		if delta<distance:
			return self.turn_perp()
		else:
			return self.turn_parallel()

	def find_closest_path(self, point):
		""" Finds and returns the closest path to the given point from the list of paths """
		closest_distance = sys.maxint
		closest_path = 0
		for path_index, path in enumerate(self.map_model.paths):
			nearest_point = geometry.nearest_point_on_path(path, point)
			distance_to_bike = geometry.distance(nearest_point, point)
			if (closest_distance > distance_to_bike):
				closest_distance = distance_to_bike
				closest_path = path_index
		return closest_path

	def displacement_to_turn(self, b = None, target_path = None):
		""" Returns: the delta which represents the distance at which the bike
		should enter the s curve based on its turning radius """
		if target_path is None:
			target_path = self.target_path
		p = geometry.unit_vector(self.map_model.paths[target_path][0], self.map_model.paths[target_path][1])
		if b is None:
			b = self.map_model.bike.vector
		R = np.array([[p[0], p[1]], [-p[1], p[0]]])
		p_R = R.dot(p)
		b_R = R.dot(b)
		phi = (np.arccos(b_R[0])-np.pi/2)%(2*np.pi)
		r = self.map_model.bike.turning_r
		delta = r+r*np.sin(phi)
		return delta


	def turn_parallel(self):
		""""""
		target_path = self.map_model.paths[self.target_path]
		path_vector = geometry.unit_vector(target_path[0], target_path[1])
		path_perp = np.array([-path_vector[1], path_vector[0]])
		return self.turn_helper(path_perp)


	def turn_perp(self):
		""""""
		target_path = self.map_model.paths[self.target_path]
		path_vector = geometry.unit_vector(target_path[0], target_path[1])
		p_orig = path_vector
		bike_pos = (self.map_model.bike.xB, self.map_model.bike.yB)
		sign = geometry.get_sign(geometry.distance_from_path(bike_pos, self.map_model.paths[self.target_path]))
		path_vector = path_vector*sign
		return self.turn_helper(path_vector)


	def turn_helper(self, path_vector):
		""""""
		bike_vector = self.map_model.bike.vector
		dot_product = np.sum(bike_vector*path_vector)
		turn = 0 if np.abs(dot_product)<.01 else geometry.get_sign(dot_product)*(-1)
		facing_away = np.sum(np.array([-path_vector[1], path_vector[0]])*bike_vector)>0
		if turn == 0 and facing_away:
			return 1
		return turn


	def calc_overshoot(self):
		"""Returns: calculated next overhsoot distance of the bike"""
		bike = self.map_model.bike
		bike_copy = bikeState.Bike(bike.xB, bike.yB, bike.phi, bike.psi, bike.delta, bike.w_r, bike.v)
		map_model_copy = mapModel.Map_Model(bike_copy, self.map_model.waypoints, self.map_model.obstacles, self.map_model.paths)
		nav_copy = Nav(map_model_copy)
		point_found = False

		while (point_found != True):

			steerD = nav_copy.direction_to_turn()
			updated_bike = bikeSim.new_state(nav_copy.map_model.bike, steerD)
			bike_copy = updated_bike
			nav_copy.map_model.bike = bike_copy # Maybe this is not necessary because before we modified bike object folder

			path_angle = geometry.line_angle(nav_copy.map_model.paths[nav_copy.target_path])
			bike_angle = nav_copy.map_model.bike.psi
			if (math.fabs(path_angle - bike_angle) < 0.005): # If caught in infinite loop means that this value should be greater
				point_found = True

		point = (nav_copy.map_model.bike.xB, nav_copy.map_model.bike.yB)
		distance = geometry.distance_from_path(point, self.map_model.paths[self.target_path])

		return distance


	def calc_overshoot2(self):
		"""Returns: value of delta based on the displacement to turn of the bike """
		bike = self.map_model.bike
		bike_copy = bikeState.Bike(bike.xB, bike.yB, bike.phi, bike.psi, bike.delta, bike.w_r, bike.v)
		map_model_copy = mapModel.Map_Model(bike_copy, self.map_model.waypoints, self.map_model.obstacles, self.map_model.paths)
		nav_copy = Nav(map_model_copy)
		initial_position = (map_model_copy.bike.xB, map_model_copy.bike.yB)
		point_found = False

		while (point_found != True):

			steerD = nav_copy.turn_parallel()
			updated_bike = bikeSim.new_state(nav_copy.map_model.bike, steerD)
			bike_copy = updated_bike
			nav_copy.map_model.bike = bike_copy # Maybe this is not necessary because before we modified bike object folder

			path_angle = geometry.line_angle(nav_copy.map_model.paths[nav_copy.target_path])
			bike_angle = nav_copy.map_model.bike.psi
			if (math.fabs(path_angle - bike_angle) < 0.007):
				point_found = True

		final_position = (nav_copy.map_model.bike.xB, nav_copy.map_model.bike.yB)
		dist = geometry.distance(initial_position, final_position)
		angle_path = geometry.line_angle(nav_copy.map_model.paths[nav_copy.target_path])
		angle_line = geometry.line_angle((final_position, initial_position))
		# angle_between = np.abs(angle_path - angle_line)
		angle_between = np.abs(geometry.angle_between_two_lines(nav_copy.map_model.paths[nav_copy.target_path], (initial_position, final_position)))

		delta = np.sin(angle_between)*dist

		return delta # vertical displacement of final position from initial position (delta)


	def close_enough(self):
		""" Returns: true if bike is close enough to the target path line so that 
		p-controller takes over, false otherwise 

		Professor Ruina comment: This should also take into account the angle b/w the bike
		and the path """
		path = self.map_model.paths[0]
		path_vector = geometry.unit_vector(path[0], path[1])
		bike_vector = self.map_model.bike.vector
		angle_from_path = geometry.angle_between_vectors(bike_vector, path_vector)

		distance = geometry.distance_from_path((self.map_model.bike.xB, self.map_model.bike.yB), self.map_model.paths[self.target_path])
		# return ( ( np.abs(distance) < 3 ) and ( np.abs(angle_from_path) < math.pi/2.0) )
		return True


	def controller_direction_to_turn(self):
		""" pd controller """
		bike_position = (self.map_model.bike.xB, self.map_model.bike.yB)
		path = self.map_model.paths[self.original_find_closest_path()]
		#self.target_path = self.find_closest_path()#QUICK 
		self.target_path = 0

		print"TARGET PATH IS", self.original_find_closest_path()

		path_vector = geometry.unit_vector(path[0], path[1])
		bike_vector = self.map_model.bike.vector
		angle_from_path = geometry.angle_between_vectors(bike_vector, path_vector)
		path_perp = np.array([-path_vector[1], path_vector[0]])
		dot = geometry.dot_product(path_perp, bike_vector)
		angle_from_path = angle_from_path * np.sign(dot) # from -pi to pi

		# angle_from_path = self.map_model.bike.psi - geometry.angle_between_two_lines(path, ((0,0),(1,0))) # from -pi to pi
		# print "ANGLLLLE", angle_from_path

		# angle_from_path = geometry.angle_between_two_lines(self.map_model.bike.vector, geometry.unit_vector(path[0], path[1]))
		# path_perp = 
		
		k1 = .35 #gain for distance correction
		k2 = 1 #gain for angle correction
		nearest_p = geometry.nearest_point_on_path(self.map_model.paths[self.target_path], (self.map_model.bike.xB, self.map_model.bike.yB))
		# print "NEAREST_P IS", nearest_p
		perp_v = geometry.threeD_unit_vector(nearest_p, (self.map_model.bike.xB, self.map_model.bike.yB))

		# find which way the cross product, perp_v vs path_v is pointing to determine the sign 
		path_v = geometry.threeD_unit_vector(self.map_model.paths[self.target_path][0], self.map_model.paths[self.target_path][1])

		print "PERP+V", perp_v
		print "PATH_V", path_v
		cross = np.cross(perp_v, path_v)
		print "CROSS", cross[2]

		sign = np.sign(cross[2])

		dist = geometry.distance((self.map_model.bike.xB, self.map_model.bike.yB), nearest_p)
		# d = geometry.distance_from_path((self.map_model.bike.xB, self.map_model.bike.yB), self.map_model.paths[self.target_path]) #distance #IT HAS A BUG
		# should be signed perpendicular distance from target path 

		d = dist * sign * (-1)
		print "ACTUAL TARGET", self.map_model.paths[self.target_path]
		print "signed angle from path", angle_from_path
		print "signed distance from path", d
		# d = np.abs(d)
		steerD = k1*d + k2*angle_from_path
		print "steeeerD", steerD
		if (steerD > MAX_STEER):
			# print "one"
			steerD = MAX_STEER
		elif (steerD < -MAX_STEER):
			# print "two"
			steerD = -MAX_STEER
		#else don't do anything
		print "steerD is", steerD
		#normalize
		# steerD = steerD / MAX_STEER
		return steerD
	
	def pure_pursuit(self):
		"""Uses lookahead point distance ld from bike to return a steering angle"""
		ld = 3 #lookahead distance
		bike_vector = self.map_model.bike.vector
		psi = self.map_model.bike.psi #bike direction (angle)
		opp = math.sin(psi) * ld 
		adj = math.cos(psi) * ld
		goalpoint_position = (self.map_model.bike.xB + adj, self.map_model.bike.yB + opp)
		path = self.map_model.paths[self.find_closest_path(goalpoint_position)] #path closest to lookahead point
		gp_on_path = geometry.nearest_point_on_path(path, goalpoint_position)
		self.lookahead_point = gp_on_path # Exposed for the visualization code
		print "bike position", (self.map_model.bike.xB, self.map_model.bike.yB)
		lookahead_vector = geometry.unit_vector((self.map_model.bike.xB, self.map_model.bike.yB), gp_on_path)
		print "l vector", lookahead_vector
		print "bike vector", bike_vector
		lookahead_angle = geometry.angle_between_vectors(bike_vector, lookahead_vector)
		lookahead_perp = np.array([-lookahead_vector[1], lookahead_vector[0]])
		dot = geometry.dot_product(lookahead_perp, bike_vector)
		lookahead_angle = lookahead_angle * np.sign(dot)
		print "l angle", lookahead_angle
		k = 2*math.sin(lookahead_angle)/ld
		print "k is ", k
		L = 0.9144
		steerD = math.atan(k*L)
		print "steeeerD", steerD
		if (steerD > MAX_STEER):
			steerD = MAX_STEER
		elif (steerD < -MAX_STEER):
			steerD = -MAX_STEER
		return steerD

	def pure_pursuit_waypoint(self):
		"""Uses next waypoint as lookahead point to return a steering angle"""
		
		ld = 5 #lookahead distance
		bike_vector = self.map_model.bike.vector
		bike_position = (self.map_model.bike.xB, self.map_model.bike.yB)
		print "bike position", bike_position
		path = self.map_model.paths[self.find_closest_path(bike_position)]
		lookahead_vector = geometry.unit_vector(bike_position, path[1]) #vector from bike position to lookahead point
		self.lookahead_point = path[1] # Exposed for the visualization code
		print "l vector", lookahead_vector
		print "bike vector", bike_vector
		lookahead_angle = geometry.angle_between_vectors(bike_vector, lookahead_vector)
		lookahead_perp = np.array([-lookahead_vector[1], lookahead_vector[0]])
		dot = geometry.dot_product(lookahead_perp, bike_vector)
		lookahead_angle = lookahead_angle * np.sign(dot)
		print "l angle", lookahead_angle
		k = 2*math.sin(lookahead_angle)/ld #curvature
		print "k is ", k
		L = 0.9144 #wheelbase distance
		steerD = math.atan(k*L)
		print "steeeerD", steerD
		if (steerD > MAX_STEER):
			steerD = MAX_STEER
		elif (steerD < -MAX_STEER):
			steerD = -MAX_STEER
		return steerD
	
	def pure_pursuit_2(self):
		"""ADD SPEC HERE"""

		ld = 10 #lookahead distance
		bike_vector = self.map_model.bike.vector
		bike_position = (self.map_model.bike.xB, self.map_model.bike.yB)
		path_index = self.find_closest_path(bike_position)
		self.closest_path_index = path_index # Exposed for visualizer
		# print("CLOSEST PATH INDEX IS {}".format(path_index))
		path = self.map_model.paths[path_index]
		start_point_on_path = geometry.nearest_point_on_path(path, bike_position)
		path_vector = geometry.unit_vector(path[0], path[1])

		# gp_on_path is the candidate lookahead point, located (hopefully) on the current path segment
		gp_on_path = (start_point_on_path[0] + path_vector[0]*ld, start_point_on_path[1] + path_vector[1]*ld)
		self.dropped_point = gp_on_path

		#if goalpoint goes beyond path
		if not geometry.is_between(path[0], path[1], gp_on_path):
			dist_past_end = geometry.distance(gp_on_path, path[1])
			next_path = self.map_model.paths[(path_index + 1) % len(self.map_model.paths)]
			next_path_vector = geometry.unit_vector(next_path[0],next_path[1])
			gp_on_path = (next_path[0][0] + next_path_vector[0]*dist_past_end,
				      next_path[0][1] + next_path_vector[1]*dist_past_end)

		self.lookahead_point = gp_on_path # Exposed for the visualization code
		lookahead_vector = geometry.unit_vector((self.map_model.bike.xB, self.map_model.bike.yB), gp_on_path)
		lookahead_angle = geometry.angle_between_vectors(bike_vector, lookahead_vector)
		lookahead_perp = np.array([-lookahead_vector[1], lookahead_vector[0]])
		dot = geometry.dot_product(lookahead_perp, bike_vector)
		lookahead_angle = lookahead_angle * np.sign(dot)
		# print "l angle", lookahead_angle
		k = 2*math.sin(lookahead_angle)/ld # kappa = curvature
		# print "k is ", k
		L = 0.9144
		steerD = math.atan(k*L)
		# print "steeeerD", steerD
		if (steerD > MAX_STEER):
			steerD = MAX_STEER
		elif (steerD < -MAX_STEER):
			steerD = -MAX_STEER
		return steerD

	def pid_with_cutting(self):
		"""Two parts here: a regular PID controller, and a component
		that detects when a turn is coming up so we can start turning
		early."""

		bike = self.map_model.bike
		bike_pos = (bike.xB, bike.yB)

		# Determine the path segment closest to the bike
		self.closest_path_index = self.find_closest_path(bike_pos)
		path = self.map_model.paths[self.closest_path_index]

		# Get a unit vector perpendicular to the path
		path_vector = (path[1][0] - path[0][0], path[1][1] - path[0][1])
		path_perp_vector = np.array((-path_vector[1], path_vector[0]))
		path_perp_unit_vector = path_perp_vector / np.linalg.norm(path_perp_vector)

		# Project vector from path[0] to bike onto the path perp unit vector to get signed distance
		# (positive when the bike is to the left of the path)
		path_to_bike_vector = np.array((bike.xB - path[0][0], bike.yB - path[0][1]))
		signed_dist = np.dot(path_perp_unit_vector, path_to_bike_vector)

		# Get angle between this path segment and the x-axis
		path_angle = math.atan2(path_vector[1], path_vector[0])
		while path_angle < 0:
			path_angle += 2 * math.pi

		# Force bike angle between 0 and 2pi
		corrected_bike_psi = bike.psi
		while corrected_bike_psi < 0:
			corrected_bike_psi += 2 * math.pi
		#print("Bike psi of {} corrected to {}".format(bike.psi, corrected_bike_psi))

		#print("Path angle is {}, bike angle  is {}".format(path_angle * RAD_TO_DEG, corrected_bike_psi * RAD_TO_DEG))

		# Get angle between bike and path
		angle_diff = corrected_bike_psi - path_angle

		if abs(angle_diff) > math.pi:
			angle_diff -= math.copysign(2 * math.pi, angle_diff)
		#print("Angle diff of {} corrected to {}".format((corrected_bike_psi - path_angle) * RAD_TO_DEG, angle_diff * RAD_TO_DEG))

		distance_contribution = PID_DIST_GAIN * MAX_STEER * math.tanh(signed_dist)
		angle_contribution = (1.5 if (abs(signed_dist) > 1.5) else 2) * angle_diff
		next_turn_contribution = NEXT_TURN_GAIN * self.create_lookahead_correction(path, bike)

		if next_turn_contribution:
			distance_contribution = 0
			angle_contribution = 0

		describe_angle = lambda angle: "right" if angle > 0 else "left"
		print("dist = {:.4f} ({})\tangle = {:.4f} ({})\tnext_turn = {:.4f} ({})"
		      .format(distance_contribution, describe_angle(distance_contribution),
			      angle_contribution, describe_angle(angle_contribution),
		      next_turn_contribution, describe_angle(next_turn_contribution)))

		# The original purpose of this if statement was to detect the case where the bike was turning
		# towards the path and it was going to end up going perpendicular - in this case we needed to
		# amp up the angle-fixing and de-emphasize the distance fixing.
		if abs(angle_diff) > MAX_ACCEPTABLE_ANGLE_DIFF:
			if False:#abs(angle_diff) > math.pi / 4.0:
				distance_contribution /= 2.5
				angle_contribution *= 2
			else:
				distance_contribution /= 1.5

		steerD = distance_contribution + angle_contribution + next_turn_contribution
		return self.clamp_steer_angle(steerD)

	def create_lookahead_correction(self, current_path, bike):

		"""Looks at the path ahead and returns a steering angle correction."""

		# Project bike-to-path-endpoint and bike-to-path-start vectors onto path unit vector
		path_start_to_bike_vector =  np.array((current_path[0][0] - bike.xB, current_path[0][1] - bike.yB))
		path_endpoint_to_bike_vector = np.array((current_path[1][0] - bike.xB, current_path[1][1] - bike.yB))
		path_vector = (current_path[1][0] - current_path[0][0], current_path[1][1] - current_path[0][1])
		path_unit_vector = np.array(path_vector) / np.linalg.norm(path_vector)
		dist_from_start = -np.dot(path_start_to_bike_vector, path_unit_vector)
		dist_until_endpoint = np.dot(path_endpoint_to_bike_vector, path_unit_vector)

		# Calculate angle wrt x-axis of current path segment
		path_angle = math.atan2(path_vector[1], path_vector[0])

		lookahead_factor = self.lookahead_factors[self.closest_path_index]
		curr_turn_radius = (0.91 * math.cos(bike.phi)) / (MAX_STEER * math.cos(0.4))
		lookahead_distance = self.lookahead_distances[self.closest_path_index]#(curr_turn_radius * lookahead_factor + 1) * 2
		#print("bike.phi = {} deg, curr_turn_radius = {} -> lookahead distance: {}".format(bike.phi * RAD_TO_DEG, curr_turn_radius, lookahead_distance))
		prev_segment_index = (self.closest_path_index - 1) % len(self.lookahead_distances)
		prev_lookahead_distance = self.lookahead_distances[prev_segment_index]
		#print("{} vs {}".format(dist_until_endpoint, lookahead_distance))
		#print("[prev] {} from start, prev lookahead was {}".format(dist_from_start, prev_lookahead_distance))
		if dist_until_endpoint < lookahead_distance:

			# Find the angle between this segment and the next one
			next_segment_index = (self.closest_path_index + 1) % len(self.map_model.paths)
			next_segment = self.map_model.paths[next_segment_index]
			next_segment_vector = (next_segment[1][0] - next_segment[0][0], next_segment[1][1] - next_segment[0][1])

			# Calculate angle between next segment and x-axis
			next_segment_angle = math.atan2(next_segment_vector[1], next_segment_vector[0])
			segment_angle_diff = next_segment_angle - path_angle

			# If the path is right-to-left (and perhaps other cases), fix range of angle
			if abs(segment_angle_diff) > math.pi:
				segment_angle_diff -= math.copysign(2 * math.pi, segment_angle_diff)
			return math.copysign(MAX_STEER, -segment_angle_diff)#-segment_angle_diff * NEXT_TURN_GAIN
		elif not self.first_segment and dist_from_start < prev_lookahead_distance:
			
			# Find the angle between this segment and the previous one
			prev_segment = self.map_model.paths[prev_segment_index]
			prev_segment_vector = (prev_segment[1][0] - prev_segment[0][0], prev_segment[1][1] - prev_segment[0][1])

			prev_segment_angle = math.atan2(prev_segment_vector[1], prev_segment_vector[0])
			prev_segment_angle_diff = prev_segment_angle - path_angle

			# If the path is right-to-left (and perhaps other cases), fix range of angle
			if abs(prev_segment_angle_diff) > math.pi:
				prev_segment_angle_diff -= math.copysign(2 * math.pi, prev_segment_angle_diff)

			return math.copysign(MAX_STEER, prev_segment_angle_diff)
		else:
			return 0

	def quintic_steering_angle(self, dist_error, angle_error, curvature_error, L, s):
		# Equations from pages 15-16 of:
		# http://ri.cmu.edu/pub_files/pub3/singh_sanjiv_1991_1/singh_sanjiv_1991_1.pdf
		a2 = 2 * curvature_error
		a3 = -(13 * curvature_error * L**2 + 12 * angle_error * L + 20 * dist_error) / (2 * L**3)
		a4 = (9 * curvature_error * L**2 + 8 * angle_error * L + 15 * dist_error) / (L**4)
		a5 = -(7 * curvature_error * L**2 + 6 * angle_error * L + 12 * dist_error) / (2 * L**5)
		error_space_curvature = 2 * a2 + 6 * a3 * s + 12 * a4 * s ** 2 + 20 * a5 * s ** 3
		return math.atan(error_space_curvature * BIKE_LENGTH)

	def quintic(self):
		bike = self.map_model.bike
		bike_pos = (bike.xB, bike.yB)

		# The quintic demands three parameters: distance error, heading
		# error, and curvature error. We could calculate curvature error
		# with a queue and a table but nah it seems to be doing fine
		# anyway.

		# Determine the path segment closest to the bike
		self.closest_path_index = self.find_closest_path(bike_pos)
		path = self.map_model.paths[self.closest_path_index]

		# Get a unit vector perpendicular to the path
		path_vector = (path[1][0] - path[0][0], path[1][1] - path[0][1])
		path_perp_vector = np.array((-path_vector[1], path_vector[0]))
		path_perp_unit_vector = path_perp_vector / np.linalg.norm(path_perp_vector)

		# Project vector from path[0] to bike onto the path perp unit vector to get signed distance
		# (positive when the bike is to the left of the path)
		path_to_bike_vector = np.array((bike.xB - path[0][0], bike.yB - path[0][1]))
		signed_dist = np.dot(path_perp_unit_vector, path_to_bike_vector)

		# Force bike angle between -pi and pi
		corrected_bike_psi = bike.psi
		while corrected_bike_psi > math.pi: corrected_bike_psi -= math.pi
		while corrected_bike_psi < -math.pi: corrected_bike_psi += math.pi

		# Get angle between this path segment and the x-axis
		path_angle = math.atan2(path_vector[1], path_vector[0])

		# Get angle between bike and path
		angle_diff = corrected_bike_psi - path_angle

		# Correct for taking the wrong perpendicular vector
		# (if the path is pointing right-to-left)
		if abs(angle_diff) > math.pi / 2:
			angle_diff -= math.copysign(math.pi, angle_diff)

		curvature_error = 0.0
		steerD = -self.quintic_steering_angle(signed_dist, angle_diff,
			curvature_error, QUINTIC_LOOKAHEAD, QUINTIC_SAMPLE_LENGTH)

		steerD += self.create_lookahead_correction(path, bike)

		return self.clamp_steer_angle(steerD)

	def get_steering_angle(self):
		"""Calls another function to calculate the steering angle.
		This function is part of the external interface of this class.
		All external users of this class should call this method
		instead of the other steering-angle-calculation methods."""

		# This code keeps the self.first_segment variable updated,
		# which is used by the create_lookahead_correction function.
		# It's not in an individual algo function because multiple
		# algos use create_lookahead_correction.
		#if self.first_segment and getattr(self, "closest_path_index", 1):
		#	self.first_segment = False

		return self.pid_with_cutting()
