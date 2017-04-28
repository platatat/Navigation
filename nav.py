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


	def find_closest_path(self):
		""" Finds and returns the closest path to the bike from the list of paths """
		closest_distance = sys.maxint
		closest_path = 0
		bike_position = (self.map_model.bike.xB, self.map_model.bike.yB)
		for path_index in range(len(self.map_model.paths)):
			nearest_point = geometry.nearest_point_on_path(self.map_model.paths[path_index], bike_position)
			distance_to_bike = geometry.distance(nearest_point, bike_position)
			if (closest_distance > distance_to_bike):
				closest_distance = distance_to_bike
				closest_path = path_index 
		disp_next = self.displacement_to_turn(target_path = (closest_path+1)%len(self.map_model.paths))
		target_path = (closest_path+1)%len(self.map_model.paths)
		distance_next = geometry.distance_from_path(bike_position, self.map_model.paths[target_path])
		if disp_next - np.abs(distance_next)>-0.01:
			closest_path = np.mod(closest_path + 1,len(self.map_model.paths))
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
			# print "VALUEE", math.fabs(path_angle - bike_angle)
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
			# print "VALUEE", math.fabs(path_angle - bike_angle)
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
		path = self.map_model.paths[self.find_closest_path()]
		path_vector = geometry.unit_vector(path[0], path[1])
		bike_vector = self.map_model.bike.vector
		angle_from_path = geometry.angle_between_vectors(bike_vector, path_vector)

		distance = geometry.distance_from_path((self.map_model.bike.xB, self.map_model.bike.yB), self.map_model.paths[self.target_path])
		# print "angle from line is ", angle_from_path
		# print "distance is ", distance
		# return ( ( np.abs(distance) < 3 ) and ( np.abs(angle_from_path) < math.pi/2.0) )
		return True


	def controller_direction_to_turn(self):
		""" pd controller """
		path = self.map_model.paths[self.find_closest_path()]
		self.target_path = self.find_closest_path()#QUICK 


		print"TARGET PATH IS", self.find_closest_path()

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







