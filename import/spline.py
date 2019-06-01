import math
import numbers, types, operator, cmath

from math import radians, degrees, sin, cos, sqrt, atan2, asin
from numpy import linalg, linspace, tile

from conversion import WaypointAngle, AngleDifference

def hobby(enu, ellipse_height, ellipse_long, ellipse_short, rph, tension=1, offset=[0, 0], cycle=True, debug=False, bezier_points=50, compensate=1) :
	rotation_angle = rph[2]
	if (compensate == 1):
		spline = hobby(enu, ellipse_height, ellipse_long, ellipse_short, rph, bezier_points=bezier_points, compensate=0)
		enu_d = [spline[1][0]-enu[0], spline[1][1]-enu[1], spline[1][2]-enu[2]]

		rotation_angle = AngleDifference(WaypointAngle(enu_d)[0], 0) 
	

	points = [[enu[0], enu[1], enu[2]], [enu[0], enu[1], enu[2]+ellipse_height], [enu[0], enu[1], enu[2]], [enu[0], enu[1], enu[2]+ellipse_height]]
    
	ellipse_add = [[0, 0], [-ellipse_long/2, ellipse_short/2], [-ellipse_long, 0], [-ellipse_long/2, -ellipse_short/2]]
    
	rotation_matrix = [math.cos(radians(rotation_angle)), math.sin(radians(rotation_angle)), math.sin(radians(rotation_angle)), math.cos(radians(rotation_angle))]

	for coord in range (len(ellipse_add)):
		ellipse_add[coord] = [ellipse_add[coord][0]*rotation_matrix[0] - ellipse_add[coord][1]*rotation_matrix[1], ellipse_add[coord][0]*rotation_matrix[2] + ellipse_add[coord][1]*rotation_matrix[3]]
		points[coord] = [ellipse_add[coord][0] + points[coord][0], ellipse_add[coord][1] + points[coord][1], points[coord][2]]




	if len(offset) == 2:
		offset.append(0)

	if cycle:
		points.insert(len(points), points[0])

	Npoints = len(points)
	z = [[float('nan'), float('nan'), float('nan')]] * Npoints #points
	w = [[float('nan'), float('nan'), float('nan')]] * Npoints #unit vectors
	tin = [[float('nan'), float('nan'), float('nan')]] * Npoints #tension of curve in to point
	tout = [[float('nan'), float('nan'), float('nan')]] * Npoints #tension of curve out from point

	for x in range(Npoints):
		pp = points[x]

		w[x] = [float('nan')] * 3
		tout[x] = tension
		tin[x] = tension

		if isinstance(pp[0], (list)):
			if len(pp[0]) == 2:
				#for input in the form pp := { [x1 y1 z1] s1 t1_in t1_out }

				veclen = len(pp)

				if len(pp[0]) == 2:
					z[x] = offset + [pp[0], 0]

				else:
					z[x] = offset + pp[0]

				if veclen >= 2 and isinstance(pp[1], (int, long, float, complex)):
					if len(pp[1]) == 0:
						w[x] = [math.degrees(math.cos(pp[1])), math.degrees(math.sin(pp[1])), 0]

					elif len(pp[1]) == 1:
						w[x] = [pp[1], 0]

					elif len(pp[1]) == 2:
						w[x] = pp[1]

				if veclen >=3 and isinstance(pp[2], (int, long, float, complex)):
					tin[x] = pp[2]

				if veclen == 4 and isinstance(pp[3], (int, long, float, complex)):
					tout[x] = pp[3]
		

		else:
			#if input in the form of pp := [x1 y1 z1]
			
			if len(pp) == 2:
				z[x] = list(map(operator.add, offset, [pp, 0]))
			else:
				z[x] = list(map(operator.add, offset, pp))

	if all(math.isnan(item) for item in w[0]):
		if cycle:
			w[0] = list(map(operator.sub, z[1], z[len(z)-2]))
		else:
			w[0] = list(map(operator.sub, z[1], z[0]))
		w[0] = w[0]/linalg.norm(w[0])

	if all(math.isnan(item) for item in w[len(w)-1]):
		if cycle:
			w[len(w)-1] = list(map(operator.sub, z[1], z[len(z)-2]))
		else:
			w[len(w)-1] = list(map(operator.sub, z[len(z)-1], z[len(z)-2]))
		w[len(w)-1] = w[len(w)-1]/linalg.norm(w[len(w)-1])
	
	for ii in range (1, Npoints-1):
		if all(math.isnan(item) for item in w[ii]):
			w[ii] = list(map(operator.sub, z[ii+1], z[ii-1]))
		w[ii] = w[ii]/linalg.norm(w[ii])


	#calculate control points and plot bezier curve segments

	q = [[float('nan'), float('nan'), float('nan')]] * ((bezier_points-1) * (Npoints-1))
       
       
	for ii in range(Npoints-1):
		
		theta   = arg(w[ii]) - arg(list(map(operator.sub, z[ii+1], z[ii])))
		phi     = arg(list(map(operator.sub, z[ii+1], z[ii]))) - arg(w[ii+1])

		rho     = velocity_rho(theta, phi)
		sigma   = velocity_sigma(theta, phi)

		spline = bezier(
			z[ii],
			z[ii]+rho/(3*tout[ii])*linalg.norm(list(map(operator.sub, z[ii+1], z[ii])))*w[ii],
			z[ii+1]-sigma/(3*tin[ii+1])*linalg.norm(list(map(operator.sub, z[ii+1], z[ii])))*w[ii+1],
			z[ii+1],
			bezier_points)
		#print (spline)
		for row in range (bezier_points-1):
			
			q[(((ii)*(bezier_points-1)) + row)] = spline[row]

	return q




def arg(w):
	o = cmath.phase(complex(w[0], w[1]))
	return o


def velocity_rho(theta, phi):
	a = 1.597
	b = 0.07
	c = 0.37

	st = math.sin(theta)
	ct = math.cos(theta)
	sp = math.sin(phi)
	cp = math.cos(phi)

	alpha = a * (st-b*sp )* (sp-b*st) * (ct-cp)
	rho   = (2+alpha) / (1+(1-c) * ct+c * cp)

	return rho


def velocity_sigma(theta, phi):
	a = 1.597
	b = 0.07
	c = 0.37

	st = math.sin(theta)
	ct = math.cos(theta)
	sp = math.sin(phi)
	cp = math.cos(phi)

	alpha = a * (st-b*sp )* (sp-b*st) * (ct-cp)
	sigma = (2-alpha) / (1+(1-c) * cp+c * ct)

	return sigma

def bezier(P1, P2, P3, P4, N):
	t = linspace(0,1,N)
	
	c1 = 3*(P2 - P1)
	c2 = 3*(P1 - 2*P2 + P3)
	c3 = 3*(P2-P3) -P1 + P4

	tiled = tile(P1, [N, 1])
	Q = list(addl(addl(addl(map(lambda x: x*c3, map(lambda x: x**3, t)), map(lambda x: x*c2, map(lambda x: x**2, t))), map(lambda x: x*c1, t)), tiled))

	return Q

def addl(l1, l2):
	return list(map(operator.add, l1, l2))
