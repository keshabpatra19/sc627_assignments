import numpy as np
import math
from numpy.linalg import norm
import matplotlib.pyplot as plt
pt1=[1,2];
pt2=[2,2];
q=[1.5, 5];

def computeLineThroughTwoPoints(p1, p2):
    a=[0,0,0];
    a[0]=p2[1]-p1[1]
    a[1]=-(p2[0]-p1[0])
    a[2]=(p2[0]-p1[0])*p1[1]-(p2[1]-p1[1])*p1[0]
    p=norm(a[:2],2)
    a=a/p
    return a

def computeDistancePointToLine(q, p1, p2):
    d=computeLineThroughTwoPoints(p1, p2)
    q.append(1)
    d.tolist()
    return np.dot(d,q)

def computeDistancePointToSegment(q, p1, p2):
    px=p2[0]-p1[0]
    py=p2[1]-p1[1]
    qx=q[0]-p1[0]
    qy=q[1]-p1[1]

    u=(px*qx+py*qy)/(px*px+py*py)

    if u > 1:
        u=1
    elif u < 0:
        u=0

    x=p1[0]+u*px
    y=p1[1]+u*py

    dx=q[0]-x
    dy=q[1]-y
    d=(dx*dx+dy*dy)**0.5
    return d

def computeNearPointToSegment(q, p1, p2):
    px=p2[0]-p1[0]
    py=p2[1]-p1[1]
    qx=q[0]-p1[0]
    qy=q[1]-p1[1]

    u=(px*qx+py*qy)/(px*px+py*py)

    if u > 1:
        u=1
    elif u < 0:
        u=0

    x=p1[0]+u*px
    y=p1[1]+u*py

    dx=q[0]-x
    dy=q[1]-y
    return [x,y]

def computeNearPointToPolygon(q, P):
    l=len(P)
    d=np.zeros(l)
    for i in range(0,l-1):
        d[i]=computeDistancePointToSegment(q,P[i],P[i+1])
    d[l-1]=computeDistancePointToSegment(q,P[l-1],P[0])
    id=np.argmin(d)
    if id==l-1:
        idb=0
    else:
        idb=id+1
    c=computeNearPointToSegment(q, P[id], P[idb])
    return c

def computeDistancePointToPolygon(q,P):
    l=len(P)
    d=np.zeros(l)
    for i in range(0,l-1):
        d[i]=computeDistancePointToSegment(q,P[i],P[i+1])
    d[l-1]=computeDistancePointToSegment(q,P[l-1],P[0])
    return min(d)

def computeTangentVectorToPolygon(q,P):
    l=len(P)
    d=np.zeros(l)
    dp=np.zeros(2)
    v=np.zeros(2)
    for i in range(l-1):
        d[i]=computeDistancePointToSegment(q,P[i],P[i+1])
    d[l-1]=computeDistancePointToSegment(q,P[l-1],P[0])

    idx=np.argmin(d)
    if idx==l-1:
        idb=0
    else:
        idb=idx+1
    dp[0]=norm(np.array(P[idx])-np.array(q),2)
    dp[1]=norm(np.array(P[idb])-np.array(q),2)
    if dp[0]<dp[1]:
        idp=idx
    else:
        idp=idb
    
    if abs(d[idx]-min(dp))<=0.001:
        th=math.atan2((q[0]-P[idp][0]),-(q[1]-P[idp][1]))
        v[0]=math.cos(th)
        v[1]=math.sin(th)
    else:
        v=(np.array(P[idb])-np.array(P[idx]))/norm(np.array(P[idb])-np.array(P[idx]))

    return (v)

def plotpath(poly,path):
    sq=plt.Polygon(poly[0],fc='r')                    
    sq1=plt.Polygon(poly[1],fc='g')
    plt.gca().add_patch(sq)
    plt.gca().add_patch(sq1)
    plt.plot(path[:,0],path[:,1])
    plt.axis([-5, 10, -5, 10])
    plt.show()
    return 0

#l = computeLineThroughTwoPoints(pt1, pt2)
#print(l)
#d=computeDistancePointToLine(q, pt1, pt2)
#print(d)
#dst=computeDistancePointToSegment(q, pt1, pt2)

