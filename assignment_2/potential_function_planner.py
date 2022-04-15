#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import numpy as np
from math import atan2, pi
from numpy.linalg import norm
from helper import *


#import other helper files if any


rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
f=open('input.txt','r')
lines=f.readlines()
i=0
sparam=[]
col=[]
poly=[]
for line in lines:
    if line =="\n":
        i+=1
    elif i == 0:
        ln=line.split()[0].split(",")
        sparam.append([float(l) for l in ln])
        j=1
    elif i == j:
        ln=line.split()[0].split(",")
        col.append([float(l) for l in ln])
    else:
        poly.append(col)
        j=i
        col=[]
        ln=line.split()[0].split(",")
        col.append([float(l) for l in ln])
        
poly.append(col)

f.close()
ps=np.array(sparam[0])
pg=np.array(sparam[1])
pc0=np.zeros(2)
step=np.array(sparam[2])
pc=ps
path=pc
##
dg=2
xi=0.8
eta=0.8
qt=2

dU=1
path=pc

#setting result as initial location
result = MoveXYResult()
result.pose_final.x = ps[0]
result.pose_final.y = ps[1]
result.pose_final.theta = 0 #in radians (0 to 2pi)

while norm(dU)>0.001:
    dUa=0
    dUr=0
    if norm(pg-pc)<=dg:
        dUa=xi*(pc-pg)
    else:
        dUa=dg*xi*(pc-pg)/norm(pc-pg)

    for P in poly:
        d=computeDistancePointToPolygon(pc,P) 
        c=computeNearPointToPolygon(pc, P)
        if d<=qt:
            dUr+=eta*((qt**-1)-(d**-1))*(d**-3)*(pc-c)
        else:
            dUr+=0

    dU=dUa+dUr
    pc0=pc
    pc=pc-step*dU
    path=np.vstack([path,pc])
    v=(pc-pc0)/norm(pc-pc0)
    th=atan2(v[1],v[0])
    if th<0:
        th += 2*pi
                     
    wp = MoveXYGoal()
    wp.pose_dest.x = pc[0]
    wp.pose_dest.y = pc[1]
    wp.pose_dest.theta =th #theta is the orientation of robot in radians (0 to 2pi)
    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()

    # getting updated robot location
    result = client.get_result()
    pc=np.array([result.pose_final.x,result.pose_final.y])

print("completed: ", pc)
f=open('output.txt','w')
for ln in path:
    f.write(str(ln[0])+','+str(ln[1])+'\n')
f.close()
plotpath(poly,path)