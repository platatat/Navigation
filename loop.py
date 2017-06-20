import sys
import nav
import mapModel
import math
import bikeState
import bikeSim
from constants import *
import geometry
import requestHandler

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

import numpy as np

def loop_experimental(nav, bike, map_model):
	traces = [dict()]
	qt_app = QtGui.QApplication([])
	qt_win = pg.GraphicsWindow(title="Basic plotting examples")
	qt_win.resize(800, 600)
	qt_win.setWindowTitle("Testing!")

	# Stores every item in the "trajectory" plot
	plot_items = [dict()]

	# This ViewBox will hold the bike and trajectory
	viewbox = qt_win.addViewBox(col=0, row=1, lockAspect=1.0)

	# Make an item for the bike
	bike_polygon = QtGui.QPolygonF()
	for _ in xrange(3): bike_polygon.append(QtCore.QPointF(0, 0))
	bikeItem = QtGui.QGraphicsPolygonItem(bike_polygon)
	plot_items[0]["bikeitem"] = bikeItem
	plot_items[0]["bike"] = bike_polygon
	bikeItem.setPen(pg.mkPen(None))
	bikeItem.setBrush(pg.mkBrush('r'))
	viewbox.addItem(bikeItem)

	# Make an item for the static (given) path
	paths = map_model.paths
	static_path = QtGui.QPainterPath()
	static_path.moveTo(paths[0][0][0], paths[0][0][1])
	for each_segment in paths:
		static_path.lineTo(*each_segment[1])
	static_path_item = QtGui.QGraphicsPathItem(static_path)
	static_path_item.setPen(pg.mkPen('g'))
	static_path_item.setBrush(pg.mkBrush(None))
	viewbox.addItem(static_path_item)

	# Make an item for the trajectory
	traj_path = QtGui.QPainterPath()
	traj_path.moveTo(bike.xB, bike.yB)
	plot_items[0]["traj"] = traj_path
	traj_path_item = QtGui.QGraphicsPathItem(traj_path)
	traj_path_item.setPen(pg.mkPen('b'))
	traj_path_item.setBrush(pg.mkBrush(None))
	plot_items[0]["trajitem"] = traj_path_item
	viewbox.addItem(traj_path_item)

	def traj_update():
		plot_items[0]["traj"].lineTo(bike.xB, bike.yB)
		plot_items[0]["trajitem"].setPath(plot_items[0]["traj"])
	traj_timer = QtCore.QTimer()
	traj_timer.timeout.connect(traj_update)
	traj_timer.start(100)

	get_steering_angle = nav.get_steering_angle
	simulation_step = lambda angle: bike.update(bikeSim.new_state(bike, angle))
	def update():
		simulation_step(get_steering_angle())
		x1, y1 = bike.xB, bike.yB
		x2, y2 = bike.xB + 2.0 * math.cos(bike.psi + 13 * math.pi / 12), bike.yB + 2.0 * math.sin(bike.psi + 13 * math.pi / 12)
		x3, y3 = bike.xB + 2.0 * math.cos(bike.psi + 11 * math.pi / 12), bike.yB + 2.0 * math.sin(bike.psi + 11 * math.pi / 12)
		#x1, y1, x2, y2, x3, y3 = tuple(int(num) for num in (x1, y1, x2, y2, x3, y3))
		new_polygon = QtGui.QPolygonF()
		for each_point in ((x1, y1), (x2, y2), (x3, y3)): new_polygon.append(QtCore.QPointF(*each_point))
		plot_items[0]["bikeitem"].setPolygon(new_polygon)
		#plot_items[0]["bike"][0].setX(bike.xB)
		#plot_items[0]["bike"][0].setY(bike.yB)
		#plot_items[0]["bike"][1].setX(
		#plot_items[0]["bike"][1].setX(bike.xB + math.sin(bike.psi - math.pi / 12))
		#plot_items[0]["bike"][2].setX(bike.xB + math.cos(bike.psi + math.pi / 12))
		#plot_items[0]["bike"][2].setX(bike.xB + math.sin(bike.psi + math.pi / 12))
		#plot_items[0]["bikeitem"].setPolygon(plot_items[0]["bike"])
		#plot_items[0]["bike"].setRect(bike.xB, bike.yB, 1, 1)
	
	anim_timer = QtCore.QTimer()
	anim_timer.timeout.connect(update)
	anim_timer.start(10)

	QtGui.QApplication.instance().exec_()

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
	new_bike = bikeState.Bike(0, 5, 0.5, 0, 0, 0, 3.57)
	# waypoints = requestHandler.parse_json(True)
	#waypoints = [(0,0), (20, 5), (40, 5)]
	waypoints = [(0,0), (50, 0)]
	#waypoints = [(0,0), (20, 5), (40, -5), (60, 10), (100, -20), (40, -50), (0,-10), (0, 0)]
	#waypoints = [(40, 0), (20, -10), (0, 0)]
	#waypoints = [(0,0), (50, 0), (50, 50), (0, 50), (0,0)]
	#waypoints = [(0, 0), (10, 0), (20, 5), (25, 15), (12, 20), (0, 15), (0, 0)]
	#waypoints = [(0, 0), (20, 0), (40, 10), (50, 30), (24, 40), (0, 30), (0, 0)]
	new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
	new_nav = nav.Nav(new_map_model)
	# print "PATHS", new_nav.map_model.paths

	loop_experimental(new_nav, new_bike, new_map_model)
