# -*- coding: utf-8 -*-
"""
Created on Wed May  8 10:05:10 2024

@author: Xavier Kipping
"""

import numpy as np
from matplotlib import pyplot as plt
from random import randint
from math import isnan

# Node class is used to define the last point in the path
class Node:
    
    def __init__(self,point):
        self.nodeNum = None
        #self.parent = None
        #self.children = []
        self.location = point
        self.path = []
    
    def calcCost(self):
       self.cost = 0
       for i in range(len(self.path)):
           if i != 0:
               dist = self.path[i] - self.path[i - 1]
               distSqr = 0
               for j in range(len(dist)):
                   distSqr += dist[j]*dist[j]
               self.cost += (distSqr)**0.5
       return self.cost
    
    def closest_point(self,point):
        prox = []
        for i in range(len(self.path)):
            px = (point[0] - self.path[i][0])*(point[0] - self.path[i][0])
            py = (point[1] - self.path[i][1])*(point[1] - self.path[i][1])
            pz = (point[2] - self.path[i][2])*(point[2] - self.path[i][2])
            prox.append((px + py + pz)**0.5)
        vclose = self.path[np.argmin(prox)]
        proxc = prox[np.argmin(prox)]
        ind = np.argmin(prox)
        return vclose,proxc,ind
    def checkCol(self,RRT):
        safe = True
        for y in range(len(self.path)):
            if y != 0:
                x1 = self.path[y-1]
                x2 = self.path[y]
        for z in RRT.stat_obj:
            x3 = z[0:3]
            topV = np.cross((x2 - x1),(x3 - x1))
            top = (topV[0]**2 + topV[1]**2 + topV[2]**2)**0.5
            botV = x2 - x1
            bot = (botV[0]**2 + botV[1]**2 + botV[2]**2)**0.5
            d = top/bot
            if d < z[3]:
                safe = False
        return safe
    
    
class RRTstar:
    
    def __init__(self,start,end,xrange,yrange,zrange,gfidel):
        self.start = start
        self.end = end
        self.xrange = xrange
        self.yrange = yrange
        self.zrange = zrange
        self.xSpace = np.linspace(xrange[0],xrange[1],gfidel)
        self.ySpace = np.linspace(yrange[0],yrange[1],gfidel)
        self.zSpace = np.linspace(zrange[0],zrange[1],gfidel)
        self.stat_obj = []
        stNode = Node(start)
        stNode.nodeNum = 0
        stNode.path = [start]
        eNode = Node(end)
        eNode.nodeNum = -1
        eNode.path = [end]
        self.tree = [stNode]
        self.btree = [eNode]
        self.ctree = []
    
    def randObj(self,mean,std,num):
        for i in range(num):
            length = np.random.normal(mean,std)
            xi = randint(self.xrange[0],self.xrange[1])
            yi = randint(self.yrange[0],self.yrange[1])
            zi = randint(self.zrange[0],self.zrange[1])
            self.stat_obj.append([xi,yi,zi,length])
        return self.stat_obj
    
    # This requires the input to be a list of [xi,yi,zi,length] for objects
    def plcObj(self,objL):
        self.stat_obj = objL
        return self.stat_obj
    
    def run(self,iterations,plot=True):
        done = False
        runs = 0
        if plot:
            ax1 = plt.figure().add_subplot(projection='3d')
            ax1.scatter(self.start[0],self.start[1],self.start[2],color="black")
            ax1.scatter(self.end[0],self.end[1],self.end[2],color="green")
            
        while not done:
            if runs >= iterations:
                done = True
            runs += 1
            print("iteration " + str(runs))
            # Generate a new point
            xnew = randint(self.xrange[0],self.xrange[1])
            ynew = randint(self.yrange[0],self.yrange[1])
            znew = randint(self.zrange[0],self.zrange[1])
            
            # vnew is the vector from this point to the origin
            vnew = np.array([xnew,ynew,znew])
            
            # Create a Node object for the new point
            nNode = Node(vnew)
            #nNode.nodeNum = nodeCount
            #nodeCount += 1
            
            # Find the closest point to this new point
            
            start = runs % 2 == 0
            if start:
                valL = []
                vcloseL = []
                for i in range(len(self.tree)):
                    startVec,startVal,sind = self.tree[i].closest_point(vnew)
                    valL.append(startVal)
                    vcloseL.append([startVec,startVal,sind,i])
                mindex = np.argmin(valL)
                vclose = vcloseL[mindex][0]
                proxc = vcloseL[mindex][1]
                sind = vcloseL[mindex][2]
                sind2 = vcloseL[mindex][3]
            else:
                valL = []
                vcloseL = []           
                for i in range(len(self.btree)):
                    endVec,endVal,eind = self.btree[i].closest_point(vnew)
                    vcloseL.append([endVec,endVal,eind,i])
                    valL.append(endVal)
                mindex = np.argmin(valL)
                vclose = vcloseL[mindex][0]
                proxc = vcloseL[mindex][1]
                eind1 = vcloseL[mindex][2]
                eind2 = vcloseL[mindex][3]

            vprox = vnew - vclose
            
            # Ensure that the stepping direction is small enough
            if proxc > 100:
                ihat = vprox[0] / proxc
                jhat = vprox[1] / proxc
                khat = vprox[2] / proxc
                vprox = np.array([ihat,jhat,khat]) * 100
                vnew = vclose + vprox
            
            # The program assumes that there are no collisions
            safe = True
            
            # Check to see if any of the static objects lie 
            # in between the old point and the new point
            
            x1 = vclose
            x2 = vnew
            
            for i in self.stat_obj:
                x3 = i[0:3]
                topV = np.cross((x2 - x1),(x3 - x1))
                top = (topV[0]*topV[0] + topV[1]*topV[1] + topV[2]*topV[2])**0.5
                botV = x2 - x1
                bot = (botV[0]*botV[0] + botV[1]*botV[1] + botV[2]*botV[2])**0.5
                d = top/bot
                if d < (i[3]+20) or isnan(d):
                    safe = False

            
            if safe:
                ax1.scatter(vnew[0],vnew[1],vnew[2],color="red")
                ax1.plot([vclose[0],vnew[0]],[vclose[1],vnew[1]],[vclose[2],vnew[2]],color="red")
                if start:
                    # Create the a copy of the tree up to the split
                    treecopy = []
                    for i in range(len(self.tree[sind2].path[0:sind+1])):
                        treecopy.append(self.tree[sind2].path[i])
                    # Add 0 to the end of the tree as a placeholder
                    treecopy.append(vnew)
                    nNode.path = treecopy
                    self.tree.append(nNode)
                    print("A node was added to the tree")
                    
                    
                else:
                    # Create the a copy of the tree 
                    treecopy = []
                    treecopy.append(vnew)
                    plen = len(self.btree[eind2].path)
                    for i in range(len(self.btree[eind2].path[eind1:plen])):
                        treecopy.append(self.btree[eind2].path[eind1+i])
                    nNode.path = treecopy
                    self.btree.append(nNode)
                
                
                # Now to write the algorithm that rewires the path
                for i in range(len(self.tree)):
                    itval = len(self.tree[i].path)
                    for j in range(itval)[1:itval]:
                        it = self.tree[i].path[j] - vnew
                        itM = (it[0]**2 + it[1]**2 + it[2]**2)
                        if itM < 120000:
                            pathcopy = []
                            for k in range(len(self.tree[i].path)):
                                pathcopy.append(self.tree[i].path[k])
                            nodi = Node(0)
                            pathcopy[j] = vnew
                            nodi.path = pathcopy
                            costi = nodi.calcCost()
                            cols = True
                            x1 = vnew
                            x2 = pathcopy[j-1]
                            for z in self.stat_obj:
                                x3 = z[0:3]
                                topV = np.cross((x2 - x1),(x3 - x1))
                                top = (topV[0]*topV[0] + topV[1]*topV[1] + topV[2]*topV[2])**0.5
                                botV = x2 - x1
                                bot = (botV[0]*botV[0] + botV[1]*botV[1] + botV[2]*botV[2])**0.5
                                d = top/bot
                                if d < (z[3]+20) or isnan(d):
                                    cols = False
                            if pathcopy[j][0] != pathcopy[-1][0] and pathcopy[j][1] != pathcopy[-1][1]:
                                x1 = vnew
                                x2 = pathcopy[j+1]
                                for z in self.stat_obj:
                                    x3 = z[0:3]
                                    topV = np.cross((x2 - x1),(x3 - x1))
                                    top = (topV[0]*topV[0] + topV[1]*topV[1] + topV[2]*topV[2])**0.5
                                    botV = x2 - x1
                                    bot = (botV[0]*botV[0] + botV[1]*botV[1] + botV[2]*botV[2])**0.5
                                    d = top/bot
                                    if d < (z[3]+20) or isnan(d):
                                        cols = False
                                if costi < self.tree[i].calcCost() and cols:
                                    self.tree[i].path = pathcopy
                
                for i in range(len(self.btree)):
                    itval = len(self.btree[i].path)
                    for j in range(itval)[0:itval-1]:
                        it = self.btree[i].path[j] - vnew
                        if (it[0]**2 + it[1]**2 + it[2]**2) < 120000:
                            pathcopy = []
                            for k in range(len(self.btree[i].path)):
                                pathcopy.append(self.btree[i].path[k])
                            nodi = Node(0)
                            pathcopy[j] = vnew
                            nodi.path = pathcopy
                            costi = nodi.calcCost()
                            cols = nodi.checkCol(self)
                            if costi < self.btree[i].calcCost() and cols:
                                self.btree[i].path = pathcopy
                
                # See if there are any collisions by connecting with
                # The btree, if not then the algorithm can end
                if runs == iterations:
                    for i in range(len(self.tree)):
                        for j in range(len(self.btree)):
                            for k in range(len(self.tree[i].path)):
                                for l in range(len(self.btree[j].path)):
                                    val = self.tree[i].path[k] - self.btree[j].path[l]
                                    dist = (val[0]*val[0] + val[1]*val[1] + val[2]*val[2])**0.5
                                    if dist < 120000:
                                        safe = True
                                        x1 = self.tree[i].path[k]
                                        x2 = self.btree[j].path[l]
                                        for m in self.stat_obj:
                                            x3 = m[0:3]
                                            topV = np.cross((x2 - x1),(x3 - x1))
                                            top = (topV[0]**2 + topV[1]**2 + topV[2]**2)**0.5
                                            botV = x2 - x1
                                            bot = (botV[0]**2 + botV[1]**2 + botV[2]**2)**0.5
                                            d = top/bot
                                            if d < (m[3]+20) or isnan(d):
                                                safe = False
                                        if safe:
                                            pathcopy = []
                                            for m in range(len(self.tree[i].path[0:k+1])):
                                                pathcopy.append(self.tree[i].path[m])
                                            for m in range(len(self.btree[j].path[l:len(self.btree[j].path)])):
                                                pathcopy.append(self.btree[j].path[l+m])
                                            nodi = Node(0)
                                            nodi.path = pathcopy
                                            self.ctree.append(nodi)
                                        
                    
                    
                                            
        costL = []
        for i in range(len(self.ctree)):
                costL.append(self.ctree[i].calcCost())
            
        indx = np.argmin(costL)
        if plot:
            for i in self.stat_obj:
                u,v = np.mgrid[0:2*np.pi:20j,0:np.pi:10j]
                x = i[0] + i[3] * np.cos(u)*np.sin(v)
                y = i[1] + i[3] * np.sin(u)*np.sin(v)
                z = i[2] + i[3] * np.cos(v)
                ax1.plot_wireframe(x,y,z,color="orange")
            ax1.set_xlabel("x-values")
            ax1.set_ylabel("y-values")
            ax1.set_zlabel("z-values")
        return self.ctree[indx].path
        
        
        
                            
                    
                    
            
            
        