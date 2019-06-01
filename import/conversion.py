import math
import numbers, types, operator, cmath

from math import radians, degrees, sin, cos, sqrt, atan2, asin
from numpy import linalg, linspace, tile

def NavDictionary(vehicle_type, enu_d, rph, velocities):
    WaypointAngles = WaypointAngle(enu_d)
    return {
        "point_distance" : NavVector(enu_d)[vehicle_type],
        "xy_waypoint_angle" : WaypointAngles[0],
        "yz_waypoint_angle" : WaypointAngles[1],
        "xy_angle_difference" : AngleDifference(WaypointAngles[0], rph[2]),
        "yz_angle_difference" : AngleDifference(WaypointAngles[1], rph[1]),
        "ground_speed" : NavVector(velocities)[0],
        "air_speed" : NavVector(velocities)[1],
        }

#LLH2ENU
def LLH2ENU(llh_base, llh):
    return [East(llh_base, llh[0], llh[1]), North(llh_base, llh[0], llh[1]), llh[2]]


def Haversine(lat1, lon1, lat2, lon2):
 
    R = 6378137 # Earth radius in kilometers

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c

def North(llh_base, lat2, lon2):

    R = 6378137 # Earth radius in kilometers
    lat1 = llh_base[0]
    lon1 = llh_base[1]
    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c



def East(llh_base, lat2, lon2):

    R = 6378137 # Earth radius in kilometers
    lat1 = llh_base[0]
    lon1 = llh_base[1]
    dLat = radians(lat2 - lat2)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat2)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c


def AngleDifference(point_angle, vehicle_orientation):
    angle_difference = point_angle - vehicle_orientation
    if angle_difference > 180:
        angle_difference = -360 + angle_difference
    if angle_difference < -180:
        angle_difference = 360 + angle_difference
    return angle_difference

#find 2D and 3D hypotenuse
def NavVector(vector_array):
    #(2D for ground vehicle, 3D for air vehicle)
    return [math.hypot(vector_array[0], vector_array[1]), math.hypot((math.hypot(vector_array[0], vector_array[1])), vector_array[2])] 

#find horizontal and vertical tangent angle
def WaypointAngle(enu_d):
    #(2D for ground vehicle, 3D for air vehicle)
    return [math.degrees(math.atan2(enu_d[0], enu_d[1])),  math.degrees(math.atan2(enu_d[2], math.hypot(enu_d[0], enu_d[1])))]