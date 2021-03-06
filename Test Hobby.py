import math
import numbers, types, operator, cmath

from math import radians, degrees, sin, cos, sqrt, atan2, asin
from numpy import linalg, linspace, tile


#####MODULES#####
import sys
sys.path.insert(0, './import')


from conversion import Haversine, East, North, LLH2ENU, NavDictionary
from spline import hobby
#####MODULES#####


#####GRAPH#####
import numpy as np
import matplotlib.pyplot as plt
#####GRAPH#####


def test():
	global llh_base
	global spline
	spline_count = 1
	ELLIPSE_LONG = 40
	ELLIPSE_SHORT = 60
	ELLIPSE_HEIGHT = 0

	
	llh_base = [-86.59418, 34.698646, 187.148395]
	current_latitude = -86.59418
	current_longitude = 34.698646
	current_height = 187.148395
	current_roll = 0
	current_pitch = 0
	current_heading = 20
	velocities = [0,0,0]
	

	llh = [current_latitude, current_longitude, current_height]

	rph = [current_roll, current_pitch, current_heading]
	enu = LLH2ENU(llh_base, llh)
	spline = hobby(enu, ELLIPSE_HEIGHT, ELLIPSE_LONG, ELLIPSE_SHORT, rph, bezier_points=50)
	enu_d = [spline[spline_count][0]-enu[0], spline[spline_count][1]-enu[1], spline[spline_count][2]-enu[2]]
	
	NavDict = NavDictionary(0, enu_d, rph, velocities)

	
	print(NavDict.get("point_distance"))
	print(NavDict.get("xy_waypoint_angle"))
	print(NavDict.get("xy_angle_difference"))
	print(NavDict.get("yz_angle_difference"))
	
	for i in range(len(spline)):
		plt.scatter(spline[i][0], spline[i][1])
	
	
	plt.show()
	print (spline)









test()
