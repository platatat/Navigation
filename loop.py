import sys
if sys.platform == "darwin":
	# Mac
	import matplotlib
	matplotlib.use('TkAgg')

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
from matplotlib import collections as mc
from matplotlib.path import Path
from matplotlib.patches import Wedge, PathPatch, Circle

def loop_using_patches(nav, bike, map_model):
	"""This function uses adding and removing patches to animate the bike."""
	plt.ion() # enables interactive plotting
	paths = map_model.paths
	fig = plt.figure()
	ax = plt.axes(**find_display_bounds(map_model.waypoints))
	lc = mc.LineCollection(paths, linewidths=2, color = "blue")
	ax.add_collection(lc)
	plt.show()

	# For plotting the bicycle
	axes = plt.gca()

	# Holds past locations of the bike, for plotting
	bike_trajectory = [(bike.xB, bike.yB)]

	# We need to keep this around to clear it after path updates
	path_patch = None

	prev_bike_patch = None
	prev_lookahead_patch = None

	# Main animation loop
	while True:

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
		bike_polygon = Wedge((bike.xB, bike.yB), 0.3,
				     bike_heading - wedge_angle / 2 + 180,
				     bike_heading + wedge_angle / 2 + 180, fc="black")
		axes.add_patch(bike_polygon)
		prev_bike_patch = bike_polygon
		plt.show()
		plt.pause(0.00000000000001)

		bike_trajectory.append((bike.xB, bike.yB))

		steerD = nav.get_steering_angle()
		# if new state has new target path then recalculate delta
		bike.update(bikeSim.new_state(bike, steerD))

def loop_using_animation(nav, bike, map_model, blitting=True):
	"""This code uses blitting and callbacks to simulate the
	bike."""
	figure, axes = plt.figure(), plt.axes(**find_display_bounds(map_model.waypoints))

	# Square aspect ratio for the axes
	axes.set_aspect("equal")

	paths = new_map_model.paths

	# Draw the paths
	lc = mc.LineCollection(paths, linewidths=2, color = "blue")
	axes.add_collection(lc)

	# Paths won't change, so capture them
	figure.canvas.draw()
	background = [figure.canvas.copy_from_bbox(axes.bbox)]

	# Create bike polygon
	bike_heading = bike.psi * (180/math.pi) # heading is psi, but in degrees
	wedge_angle = 45 # The angle covered by the wedge (degrees)
	theta1 = bike_heading - wedge_angle / 2 + 180
	theta2 = bike_heading + wedge_angle / 2 + 180
	bike_polygon = Wedge((bike.xB, bike.yB), 1, theta1, theta2, fc="black")
	bike_polygon.set_zorder(10)
	axes.add_artist(bike_polygon)

	# Create bike trajectory
	bike_trajectory_polygon = axes.plot([0, 0], [0, 0], "g")[0]

	# Set up trajectory data
	bike_traj_x = [bike.xB] # Just the x-coords
	bike_traj_y = [bike.yB] # Just the y-coords
	add_traj_x = bike_traj_x.append
	add_traj_y = bike_traj_y.append

	# Create lookahead point
	lookahead_polygon = Circle((bike.xB, bike.yB), 1)
	axes.add_artist(lookahead_polygon)

	# Create dropped point
	dropped_polygon = Circle((bike.xB, bike.yB), 1, fc="red")
	axes.add_artist(dropped_polygon)

	# Create current line highlight
	current_line = axes.plot([0, 0], [0, 0], "r")[0]
	axes.add_artist(current_line)

	# Set up resizing handlers
	listener_id = [None]
	def safe_draw():
		canvas = figure.canvas
		if listener_id[0]: canvas.mpl_disconnect(listener_id[0])
		canvas.draw()
		listener_id[0] = canvas.mpl_connect("draw_event", grab_background)
	def grab_background(event=None):
		transient_polygons = (bike_polygon, lookahead_polygon,
				      current_line, dropped_polygon)
		for polygon in transient_polygons:
			polygon.set_visible(False)
		safe_draw()
		background[0] = figure.canvas.copy_from_bbox(figure.bbox)
		for polygon in transient_polygons:
			polygon.set_visible(True)
		blit()
	def blit():
		figure.canvas.restore_region(background[0])
		axes.draw_artist(bike_polygon)
		figure.canvas.blit(axes.bbox)
	listener_id[0] = figure.canvas.mpl_connect("draw_event", grab_background)

	# This timer runs simulation steps and draws the results
	figure_restore = figure.canvas.restore_region
	get_steering_angle = nav.get_steering_angle
	simulation_step = lambda angle: bike.update(bikeSim.new_state(bike, angle))
	figure_blit = figure.canvas.blit
	def full_step():
		figure_restore(background[0])
		simulation_step(get_steering_angle())

		# Update bike polygon properties and redraw it
		wedge_dir = bike.psi * (180/math.pi) + 180
		bike_pos = (bike.xB, bike.yB)
		bike_polygon.set(center = bike_pos,
				 theta1 = wedge_dir - wedge_angle / 2,
				 theta2 = wedge_dir + wedge_angle / 2)
		axes.draw_artist(bike_polygon)

		# Update trajectory and redraw it
		add_traj_x(bike.xB)
		add_traj_y(bike.yB)
		bike_trajectory_polygon.set_xdata(bike_traj_x)
		bike_trajectory_polygon.set_ydata(bike_traj_y)
		axes.draw_artist(bike_trajectory_polygon)

		# Update and redraw lookahead point
		lookahead_polygon.center = nav.lookahead_point
		#axes.draw_artist(lookahead_polygon)

		# Update and redraw dropped point
		dropped_polygon.center = nav.dropped_point
		#axes.draw_artist(dropped_polygon)

		# Update and redraw highlight for current closest line
		curr_path_segment = paths[nav.closest_path_index]
		current_line.set_xdata([curr_path_segment[0][0], curr_path_segment[1][0]])
		current_line.set_ydata([curr_path_segment[0][1], curr_path_segment[1][1]])
		axes.draw_artist(current_line)

		# Redraw bike
		figure_blit(axes.bbox)

	# Start the update & refresh timer
	if blitting:
		figure.canvas.new_timer(interval=ANIM_INTERVAL, callbacks=[(full_step, [], {})]).start()
	else:
		FuncAnimation(figure, full_step, frames=xrange(0,200))

	# Display the window with the simulation
	plt.show()

def find_display_bounds(waypoints):
	"""Given a set of waypoints, return {xlim, ylim} that can fit them."""
	xlim = [99999, -99999] # min, max
	ylim = [99999, -99999] # min, max
	padding = 5
	for waypoint in waypoints:
		if waypoint[0] < xlim[0]:
			xlim[0] = waypoint[0]
		elif waypoint[0] > xlim[1]:
			xlim[1] = waypoint[0]

		if waypoint[1] < ylim[0]:
			ylim[0] = waypoint[1]
		elif waypoint[1] > ylim[1]:
			ylim[1] = waypoint[1]
	xlim, ylim = (xlim[0] - padding, xlim[1] + padding), (ylim[0] - padding, ylim[1] + padding)
	return {"xlim": xlim, "ylim": ylim}

if __name__ == '__main__':
	new_bike = bikeState.Bike(0, 0, 0.1, 0, 0, 0, 3.57)
	# waypoints = requestHandler.parse_json(True)
	#waypoints = [(0,0), (20, 5), (40, 5)]
	#waypoints = [(0,0), (50, 0)]
	#waypoints = [(0,0), (20, 5), (40, -5), (60, 10), (100, -20), (40, -50), (0,-10), (0, 0)]
	#waypoints = [(40, 0), (20, -10), (0, 0)]
	#waypoints = [(0,0), (50, 0), (50, 50), (0, 50), (0,0)]
	#waypoints = [(0, 0), (10, 0), (20, 5), (25, 15), (12, 20), (0, 15), (0, 0)]
	waypoints = [(0, 0), (20, 0), (40, 10), (50, 30), (24, 40), (0, 30), (0, 0)]
	new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
	new_nav = nav.Nav(new_map_model)
	# print "PATHS", new_nav.map_model.paths

	###########################
	# WAIT FOR USER TO CONFIRM SO WE CAN SEE THE NAV INIT OUTPUT
	raw_input("Press ENTER to continue")	

	# If we're not on a Mac, use blitting (it's better)
	if False:#sys.platform == "darwin":
		loop_using_patches(new_nav, new_bike, new_map_model)
	else:
		loop_using_animation(new_nav, new_bike, new_map_model)
