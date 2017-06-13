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
from matplotlib.patches import Wedge, PathPatch, Circle

def loop_using_animation(nav, bike, map_model):
	"""This code uses blitting and callbacks to simulate the
	bike."""
	figure, axes = plt.figure(), plt.axes(xlim=(-5, 50), ylim=(-5, 50))

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
	axes.add_artist(bike_polygon)

	# Set up resizing handlers
	listener_id = [None]
	def safe_draw():
		canvas = figure.canvas
		if listener_id[0]: canvas.mpl_disconnect(listener_id[0])
		canvas.draw()
		listener_id[0] = canvas.mpl_connect("draw_event", grab_background)
	def grab_background(event=None):
                bike_polygon.set_visible(False)
		safe_draw()
		background[0] = figure.canvas.copy_from_bbox(figure.bbox)
                bike_polygon.set_visible(True)
		blit()
	def blit():
		figure.canvas.restore_region(background[0])
		axes.draw_artist(bike_polygon)
		figure.canvas.blit(axes.bbox)
	listener_id[0] = figure.canvas.mpl_connect("draw_event", grab_background)

	# Set up trajectory
	trajectory_pts = []

	# This timer runs simulation steps and draws the results
	figure_restore = figure.canvas.restore_region
	get_steering_angle = nav.pure_pursuit
	simulation_step = lambda angle: bike.update(bikeSim.new_state(bike, angle))
        figure_blit = figure.canvas.blit
        def full_step():
		figure_restore(background[0])
		simulation_step(get_steering_angle())

		# Update bike polygon properties
		wedge_dir = bike.psi * (180/math.pi) + 180
		bike_polygon.set(center = (bike.xB, bike.yB),
				 theta1 = wedge_dir - wedge_angle / 2,
				 theta2 = wedge_dir + wedge_angle / 2)
		axes.draw_artist(bike_polygon)
		figure_blit(axes.bbox)

        # Start the update & refresh timer
        figure.canvas.new_timer(interval=0, callbacks=[(full_step, [], {})]).start()

        # Display the window with the simulation
        plt.show()


if __name__ == '__main__':
	new_bike = bikeState.Bike(0, 0, 0.1, 0, math.pi/6.0, 0, 3.57)
	# waypoints = requestHandler.parse_json(True)
	#waypoints = [(0,0), (20, 5), (40, 5)]
	#waypoints = [(0,0), (50, 5)]
	waypoints = [(0,0), (20, 5), (40, 5), (60, 0), (70, -10)]
	new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
	new_nav = nav.Nav(new_map_model)
	# print "PATHS", new_nav.map_model.paths

	loop_using_animation(new_nav, new_bike, new_map_model)
