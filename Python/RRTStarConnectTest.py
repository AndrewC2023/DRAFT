# -*- coding: utf-8 -*-
"""
Created on Wed May  8 15:22:00 2024

@author: Xavier Kipping
"""
from matplotlib import pyplot as plt
import numpy as np 
from RRTStarConnect import RRTstar

rrtest = RRTstar(np.array([500,500,-500]),np.array([0.0001,0.0001,0.0001]),[-500,500],[-500,500],[-500,500],1000)
#objects = rrtest.randObj(50,20,40)
objL = [[100,100,-100,30],[400,400,-400,30],[100,100,-500,10],[300,390,-300,40],[-30,200,-200,10],[-300,200,-100,20],[533,478,61,100],[96.78,203.4,-559,100]]
objects = rrtest.plcObj(objL)
finalpath = rrtest.run(100)

ax = plt.figure().add_subplot(projection='3d')
for i in objects:
    u,v = np.mgrid[0:2*np.pi:20j,0:np.pi:10j]
    x = i[0] + i[3] * np.cos(u)*np.sin(v)
    y = i[1] + i[3] * np.sin(u)*np.sin(v)
    z = i[2] + i[3] * np.cos(v)
    ax.plot_wireframe(x,y,z,color="r")

finalpath = np.array(finalpath)
ax.plot(finalpath.T[0],finalpath.T[1],finalpath.T[2],color="black")
ax.scatter(500,500,-500,color="blue")
ax.scatter(0,0,0,color="green")
ax.set_xlabel("x-values")
ax.set_ylabel("y-values")
ax.set_aspect("equal")
plt.show()



