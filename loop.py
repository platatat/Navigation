import nav
import mapModel
import math
import bikeState
import bikeSim
from constants import *
import geometry
import requestHandler

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import collections  as mc
from matplotlib.path import Path
from matplotlib.patches import Wedge, PathPatch

def main_loop(nav, bike):
	""" This is the main loop that gets the nav command, passes it through bike dynamics
	to get the updated state and plots the new state """
	k = 0 
	steerD = 0
	iters = 0

	# For plotting the bicycle
	axes = plt.gca()

	# Holds past locations of the bike, for plotting
	bike_trajectory = [(bike.xB, bike.yB)]

	# We need to keep this around to clear it after path updates
	path_patch = None

	prev_bike_patch = None

	while (k < 2000):

		# Draw the trajectory of the bike
		if path_patch:
			path_patch.remove()
		path_patch = PathPatch(Path(bike_trajectory), fill=False,
				       linewidth=2)
		axes.add_patch(path_patch)

		# Plot the bike as a wedge pointing in the direction bike.psi
		if prev_bike_patch:
			prev_bike_patch.remove()
		bike_heading = bike.psi * (180/math.pi) # Converted to degrees
		wedge_angle = 45 # The angle covered by the wedge
		bike_polygon = Wedge((bike.xB, bike.yB), 0.2,
				     bike_heading - wedge_angle / 2 + 180,
				     bike_heading + wedge_angle / 2 + 180, fc="black")
		axes.add_patch(bike_polygon)
		prev_bike_patch = bike_polygon
		plt.show()
		plt.pause(0.0000001)

		bike_trajectory.append((bike.xB, bike.yB))

		# Steer vs Time
		# plt.scatter((k+1)*TIMESTEP, bike.delta)
		# plt.show()
		# plt.pause(0.00001)

		# if (nav.close_enough()):
		#	steerD = nav.controller_direction_to_turn() #pd cotnroller takes over
		#	print "pd Controller takes over"
		# else:
		#	steerD = nav.direction_to_turn()
		# #	steerD = steerD * MAX_STEER * (-1)
		# if iters == 85:
		#	iters = 0
		#	steerD = nav.controller_direction_to_turn()
		# iters+=1
		print "STEER D IS", steerD
		# steerD = nav.controller_direction_to_turn() #pd cotnroller takes over
		steerD = nav.controller_direction_to_turn()
		# print steerD
		# if new state has new target path then recalculate delta
		bike.update(bikeSim.new_state(bike, steerD))
		# if k == 20:
		#	print "HELLOOOO", nav.calc_overshoot()
			# print "HELLOOOO", nav.calc_overshoot()
		# path_angle = geometry.line_angle(nav.map_model.paths[nav.target_path])
		# bike_angle = nav.map_model.bike.pi

		# print "ANGLE BETWEEN", math.fabs(path_angle - bike_angle)

		k = k + 1

		# When it crosses the line... ?


if __name__ == '__main__':
	
	new_bike = bikeState.Bike(0, 0, 0.1, 0, math.pi/6.0, 0, 3.57)
	# waypoints = requestHandler.parse_json(True)
	# waypoints = [(0,0), (20, 5), (40, 5)]
	waypoints = [(0,0), (50, 5)]
	new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
	new_nav = nav.Nav(new_map_model)
	print "PATHS", new_nav.map_model.paths
	# print new_nav.direction_to_turn()
	# print new_bike.rhs(new_nav.direction_to_turn())
	# PLOTTING
	plt.ion() # enables interactive plotting
	paths = new_map_model.paths
	fig = plt.figure()
	fig.set_dpi(100) #dots per inch
	ax = plt.axes(xlim=(0, 20), ylim=(0, 20))
	lc = mc.LineCollection(paths, linewidths=2, color = "blue")
	ax.add_collection(lc)
	plt.show() 
	main_loop(new_nav, new_bike)
