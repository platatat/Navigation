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

def main_loop(nav, bike):
	""" This is the main loop that gets the nav command, passes it through bike dynamics
	to get the updated state and plots the new state """
	k = 0 
	steerD = 0
	iters = 0

	while (k < 2000):

		#plotting
		plt.scatter(bike.xB, bike.yB)
		plt.show()
		plt.pause(0.0000001)

		# Steer vs Time
		# plt.scatter((k+1)*TIMESTEP, bike.delta)
		# plt.show()
		# plt.pause(0.00001)

		# if (nav.close_enough()):
		# 	steerD = nav.controller_direction_to_turn() #pd cotnroller takes over
		# 	print "pd Controller takes over"
		# else:
		# 	steerD = nav.direction_to_turn()
		# # 	steerD = steerD * MAX_STEER * (-1)
		# if iters == 85:
		# 	iters = 0
		# 	steerD = nav.controller_direction_to_turn()
		# iters+=1
		print "STEER D IS", steerD
		# steerD = nav.controller_direction_to_turn() #pd cotnroller takes over
		steerD = nav.controller_direction_to_turn()
		# print steerD
		# if new state has new target path then recalculate delta
		bike.update(bikeSim.new_state(bike, steerD))
		# if k == 20:
		# 	print "HELLOOOO", nav.calc_overshoot()
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
 	lc = mc.LineCollection(paths, linewidths=2, color = "black")
	ax.add_collection(lc)
	plt.show() 
	main_loop(new_nav, new_bike)
	