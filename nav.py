# nav.py

"""Contians Nav class and methods that make up the navigation algorithm.
Input for algorithm is bike state and waypoints, output is front wheel angle."""

import numpy as np
import math
import sys
import geometry
import bikeState
import mapModel
from constants import *

# DEBUGGING PRINT STATEMENTS
DEBUG_PRINT_CONTRIBS = False

class Nav(object):


	def __init__(self, map_model):
		""" Nav initializer """
		self.map_model = map_model
		self.target_path = 0

		# Used by loop.py for visualization
		self.lookahead_point = (0, 0)
		self.dropped_point = (0, 0)

		# State for PID controller
		self.last_dist_error = 0
		self.last_ang_error = 0
		self.dist_error_integral = 0

		# Timing for PID controller
		self.dt = 0.01

		# State for turn lookaheads
		self.first_segment = True

		num_paths = len(self.map_model.paths)

		# Precompute when we need to turn
		self.lookahead_distances = [0] * num_paths
		for index, curr_segment in enumerate(self.map_model.paths):

			# Compute angle between current segment and +x-axis
			curr_vector = (curr_segment[1][0] - curr_segment[0][0],
				curr_segment[1][1] - curr_segment[0][1])
			curr_angle = math.atan2(curr_vector[1], curr_vector[0])

			# Compute the angle between next segment and +x-axis
			next_index = (index + 1) % len(self.map_model.paths)
			next_segment = self.map_model.paths[next_index]
			next_vector = (next_segment[1][0] - next_segment[0][0],
				next_segment[1][1] - next_segment[0][1])
			next_angle = math.atan2(next_vector[1], next_vector[0])

			# Find angle between current and next path segments
			angle_diff = next_angle - curr_angle

			# Keep angle_diff between -math.pi and +math.pi
			if abs(angle_diff) > math.pi:
				angle_diff -= math.copysign(2 * math.pi,
					angle_diff)

			# pick formula based on angle_diff
			angle_diff = abs(angle_diff)
			curr_factor = 1

			# If the angle is less than 90 degrees...
			if angle_diff < math.pi / 2:

				# use the sin-based formula
				curr_factor = math.sin(angle_diff)
			else:

				# use the tan-based formula
				curr_factor = math.tan(angle_diff / 2)

			# We just calculated a factor, so multiply that
			# by the minimum turning radius
			curr_lookahead = MIN_TURN_RADIUS * curr_factor

			# If the angle is greater than 45 degrees, subtract some
			if math.pi / 4 < angle_diff < math.pi / 2:
				curr_lookahead -= 0.5
			if math.pi / 6 < angle_diff < math.pi / 4:
				curr_lookahead -= 0.25

			#print("{} => {}".format(abs(angle_diff) * RAD_TO_DEG,
			#	curr_lookahead))
			self.lookahead_distances[index] = curr_lookahead

	def clamp_steer_angle(self, steerD):
		"""Limits a steer angle to within [-MAX_STEER, +MAX_STEER]"""
		if (steerD > MAX_STEER):
			steerD = MAX_STEER
		elif (steerD < -MAX_STEER):
			steerD = -MAX_STEER

		return steerD

	def find_closest_path(self, point):
		"""
		Finds and returns the closest path to the given point
		from the list of paths (stored in self.map_model.paths)
		"""
	        # Make a list of paths with indices: [(0, path 1), ...]
		indexed_paths = enumerate(self.map_model.paths)

                def dist_to_bike(path):
                    """Returns the distance between a path and the given point"""
                    nearest_pt = geometry.nearest_point_on_path(path, point)
                    return geometry.distance(nearest_pt, point)

                # Get the path with the smallest distance from the point
		min_index, min_dist = min(indexed_paths, key=lambda p: dist_to_bike(p[1]))

		return min_index

	def pid_controller(self):
		"""
		Two parts here: a regular PID controller, and a
		component that detects when a turn is coming up so we
		can start turning early.

		Two field state variables used, besides bike:
		self.last_dist_error and self.last_ang_error, which
		store the last errors in distance and angle,
		respectively.
		"""

		bike = self.map_model.bike
		bike_pos = (bike.xB, bike.yB)
		print('Bike x:  {} Bike y: {}'.format(bike.xB, bike.yB))
		# Determine the path segment closest to the bike
		self.closest_path_index = self.find_closest_path(bike_pos)
		path = self.map_model.paths[self.closest_path_index]

		# Get a unit vector perpendicular to the path
		path_vector = (path[1][0] - path[0][0], path[1][1] - path[0][1])
		path_perp_vector = np.array((-path_vector[1], path_vector[0]))
		path_perp_unit_vector = (path_perp_vector /
			np.linalg.norm(path_perp_vector))

		# Project vector from path[0] to bike onto the path
		# perpendicular unit vector to get signed distance
		# (positive when the bike is to the left of the path)
		path_to_bike_vector = np.array((bike.xB - path[0][0],
			bike.yB - path[0][1]))
		signed_dist = np.dot(path_perp_unit_vector,
			path_to_bike_vector)

		# Get angle between this path segment and the x-axis
		path_angle = math.atan2(path_vector[1], path_vector[0])
		while path_angle < 0:
			path_angle += 2 * math.pi

		# Force bike angle between 0 and 2pi
		corrected_bike_psi = bike.psi
		while corrected_bike_psi < 0:
			corrected_bike_psi += 2 * math.pi
		while corrected_bike_psi > 2 * math.pi:
			corrected_bike_psi -= 2 * math.pi

		# Get angle between bike and path
		angle_diff = corrected_bike_psi - path_angle

		while abs(angle_diff) > math.pi:
			angle_diff -= math.copysign(2 * math.pi, angle_diff)

		# Derivative term for distance
		d_dist_contrib = (signed_dist - self.last_dist_error) / self.dt
		d_dist_contrib *= PID_D_DIST_GAIN

		# Derivative term for angle
		d_ang_contrib = (angle_diff - self.last_ang_error) / self.dt
		d_ang_contrib *= PID_D_ANG_GAIN

		# Integral term for distance
		self.dist_error_integral += signed_dist * self.dt
		i_dist_contrib = PID_I_DIST_GAIN * self.dist_error_integral

		# Emphasize distance further away from the path
		distance_gain = (PID_DIST_GAIN *
			(2 if (abs(signed_dist) > 3) else 1))
		distance_contribution = (distance_gain * MAX_STEER *
			math.tanh(signed_dist))

		# If we're right next to the path, emphasize the angle
		angle_gain = PID_ANGLE_GAIN
		if abs(signed_dist) > 1.5:
			angle_gain *= 1.05
		angle_contribution = angle_gain * angle_diff
		next_turn_contribution = (NEXT_TURN_GAIN *
			self.create_lookahead_correction(path, bike))

		# Debug statement to show each contribution
		if DEBUG_PRINT_CONTRIBS:
			describe_angle = lambda angle: "right" if angle > 0 else "left"
			print(("dist = {:.4f} ({})\tangle = {:.4f} ({})\t" +
				"next_turn = {:.4f} ({})").format(distance_contribution,
				describe_angle(distance_contribution),
				angle_contribution, describe_angle(angle_contribution),
				next_turn_contribution,
				describe_angle(next_turn_contribution)))

		# If we're turning, the turn takes priority over other
		# contributions
		if next_turn_contribution:
			steerD = next_turn_contribution
		else:
			steerD = (distance_contribution + angle_contribution +
					d_dist_contrib + d_ang_contrib +
					i_dist_contrib)

		# Store current errors in state variables
		self.last_dist_error = signed_dist
		self.last_ang_error = angle_diff

		return self.clamp_steer_angle(steerD)

	def create_lookahead_correction(self, current_path, bike):
		"""
		Looks at the path ahead and returns a steering angle
		correction.
		"""

		# Project bike-to-path-endpoint and bike-to-path-start
		# vectors onto path unit vector
		path_start_to_bike_vector =  np.array((current_path[0][0] -
			bike.xB, current_path[0][1] - bike.yB))
		path_endpoint_to_bike_vector = np.array((current_path[1][0] -
			bike.xB, current_path[1][1] - bike.yB))
		path_vector = (current_path[1][0] - current_path[0][0],
			current_path[1][1] - current_path[0][1])
		path_unit_vector = (np.array(path_vector) /
			np.linalg.norm(path_vector))
		dist_from_start = -np.dot(path_start_to_bike_vector,
			path_unit_vector)
		dist_until_endpoint = np.dot(path_endpoint_to_bike_vector,
			path_unit_vector)

		# Calculate angle wrt x-axis of current path segment
		path_angle = math.atan2(path_vector[1], path_vector[0])

		lookahead_distance =\
			self.lookahead_distances[self.closest_path_index]
		prev_segment_index = ((self.closest_path_index - 1) %
			len(self.lookahead_distances))
		prev_lookahead_distance =\
			self.lookahead_distances[prev_segment_index]
		if dist_until_endpoint < lookahead_distance:

			# Find the angle between this segment and the next one
			next_segment_index = ((self.closest_path_index + 1) %
				len(self.map_model.paths))
			next_segment = self.map_model.paths[next_segment_index]
			next_segment_vector = (next_segment[1][0] -
				next_segment[0][0], next_segment[1][1] -
				next_segment[0][1])

			# Calculate angle between next segment and x-axis
			next_segment_angle = math.atan2(next_segment_vector[1],
				next_segment_vector[0])
			segment_angle_diff = next_segment_angle - path_angle

			# If the path is right-to-left (and perhaps
			# other cases), fix range of angle
			if abs(segment_angle_diff) > math.pi:
				segment_angle_diff -= math.copysign(2 * math.pi,
					segment_angle_diff)
			return math.copysign(MAX_STEER, -segment_angle_diff)
		elif (not self.first_segment and
			dist_from_start < prev_lookahead_distance):

			# Find the angle between this segment and the
			# previous one
			prev_seg = self.map_model.paths[prev_seg_index]
			prev_seg_vector = (prev_seg[1][0] - prev_seg[0][0],
				prev_seg[1][1] - prev_seg[0][1])

			prev_seg_angle = math.atan2(prev_seg_vector[1],
				prev_seg_vector[0])
			prev_seg_angle_diff = prev_seg_angle - path_angle

			# If the path is right-to-left (and perhaps
			# other cases), fix range of angle
			if abs(prev_seg_angle_diff) > math.pi:
				prev_seg_angle_diff -= math.copysign(
					2 * math.pi, prev_seg_angle_diff)

			return math.copysign(MAX_STEER, prev_seg_angle_diff)
		else:
			return 0

	def get_steering_angle(self):
		"""Calls another function to calculate the steering angle.
		This function is part of the external interface of this class.
		All external users of this class should call this method
		instead of the other steering-angle-calculation methods."""

		return self.pid_controller()
