#!/usr/bin/python

# bb_vfh.py

# Example of how to write a controller that uses vfh
# K Nickels MArch 17 2014

# to run player server - "player bb_vfh.cfg" in one window
# to run controller - "python bb_vfh.py" in another window

import sys,os,math
# this should be whereever "playerc.py" is.  
# On linux, you can find this out with "locate playerc.py"
sys.path.append('/usr/local/lib/python2.7/site-packages/')
from playercpp import *


def dtor (deg):
    return deg*math.pi/180.0;


# Create a client object
#c = playerc_client('localhost', 6665, 0);
c = PlayerClient("localhost", 6665, 0);

# Connect it
#if c.connected() != 0:
#  raise AttributeError("No constructor defined")

# Create a proxy for position2d:0
p = Position2dProxy(c,0)
#if p.subscribe(PLAYERC_OPEN_MODE) != 0:
#  raise playerc_error_str()

pl = PlannerProxy(c,0);

#if pl.subscribe(PLAYERC_OPEN_MODE) != 0:
#  raise playerc_error_str()

# Create a proxy for sonar ranger:0
s = RangerProxy(c,0)

#if s.subscribe(PLAYERC_OPEN_MODE) != 0:
#  raise ValueError(playerc_error_str())

# Retrieve the geometry
#if s.get_geom() != 0:
#  raise playerc_error_str()

# read once to get things going
#if c.read() == None:
#  raise playerc_error_str()

# set the goal position
gx = 5.0
gy = 5.0
ga = 0.0
pl.SetGoalPose(gx,gy,ga);
#pl.enable(1);

# path_done seems not reliable...
# while (not pl.path_done):
distance_epsilon = 0.3
angle_epsilon = dtor(5)
#while ( math.sqrt( (pl.px-pl.gx)**2+ (pl.py-pl.gy)**2) > distance_epsilon or\
#        abs(  (pl.pa-pl.ga) > angle_epsilon )):
math1 = math.sqrt( (pl.GetPx()-pl.GetGx())**2+ (pl.GetPy()-pl.GetGy())**2)
resultado1 = math.sqrt( (pl.GetPx()-pl.GetGx())**2+ (pl.GetPy()-pl.GetGy())**2) > distance_epsilon
resultado2 = abs(  (pl.GetPa()-pl.GetGa()) > angle_epsilon )
pl.SetGoalPose(gx,gy,ga);
print 'gx = %d' % pl.GetGx()
print 'gy = %d' % pl.GetGy()
print 'ga = %d' % pl.GetGa()
print 'px = %d' % pl.GetPx()
print 'py = %d' % pl.GetPy()
print 'pa = %d' % pl.GetPa()
print 'math = %d' % math1
print 'resultado1 = %d' % resultado1
print 'resultado2 = %d' % resultado2
while ( resultado1 or\
        resultado2):
  # read the sensors, etc.
    if c.read() == None:
        raise playerc_error_str()
    print 'Is done? %d' % pl.GetPathDone()

    print 'Current pose (%.1f %.1f %.1f), Goal (%.1f %.1f %.1f)' %\
            (pl.GetPx(),pl.GetPy(),pl.GetPa(),pl.GetGx(),pl.GetGy(),pl.GetGa())

print 'Is done? %d' % pl.GetPathDone()
print 'done.'
