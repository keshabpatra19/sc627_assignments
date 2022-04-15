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
i=0;
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
print('Data read complete')
ps=np.array(sparam[0])
pg=np.array(sparam[1])
pc0=np.zeros(2)
step=np.array(sparam[2])
pc=ps
path=pc
dt=np.zeros(0)
#setting result as initial location
result = MoveXYResult()
result.pose_final.x = ps[0]
result.pose_final.y = ps[1]
result.pose_final.theta = 0 #in radians (0 to 2pi)

pc=np.array([result.pose_final.x,result.pose_final.y])
while norm(pg-pc)>step: #replace true with termination condition
    d=[computeDistancePointToPolygon(pc,P) for P in poly]
    if min(d)<step:
        idx=np.argmin(d)
        crm=pc
        dt=norm(pg-pc)
        pc0=pc
        v=computeTangentVectorToPolygon(pc,poly[idx])
        pc=pc+step*v
        print('Hit obstacle at:',pc)
        while min(norm(path-pc,2,axis=1))> (step-0.03):
            v=computeTangentVectorToPolygon(pc,poly[idx]) 
            pc=pc+step*v
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

            #getting updated robot location
            result = client.get_result()
            pc=np.array([result.pose_final.x,result.pose_final.y])
            crm=np.vstack([crm,pc])
            dt=np.vstack([dt,norm(pg-pc)])
        print('Circumnavigation Complete at', pc)
        dti=np.argmin(dt)
        dtm=dt[dti]
        path=np.vstack([path,crm])
        if len(crm)-dti>dti:
            while norm(pg-pc)>dtm:
                v=computeTangentVectorToPolygon(pc,poly[idx])
                pc=pc+step*v
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

                #getting updated robot location
                result = client.get_result()
                pc=np.array([result.pose_final.x,result.pose_final.y])
                path=np.vstack([path,pc])
            print('Exiting Obstacle at', pc)
        else:
            while norm(pg-pc)>dtm:
                v=-1*computeTangentVectorToPolygon(pc,poly[idx]) 
                pc=pc+step*v
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

                #getting updated robot location
                result = client.get_result()
                pc=np.array([result.pose_final.x,result.pose_final.y])
                path=np.vstack([path,pc])
            print('Exiting Obstacle')
    v=(pg-pc)/norm(pg-pc)
    pc=pc+step*v
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

    #getting updated robot location
    result = client.get_result()
    pc=np.array([result.pose_final.x,result.pose_final.y])
    path=np.vstack([path,pc])
#write to output file (replacing the part below)
print("Success")
f=open('output_1.txt','w')
for ln in path:
    f.write(str(ln[0])+','+str(ln[1])+'\n')
f.close()
plotpath(poly,path)