# -*- coding: utf-8 -*-
"""
Created on Wed May 15 14:41:58 2019

@author: user
"""

import numpy as np
from util import *

class RobotPlanner:
    
    def __init__(self, boundary, blocks, start, goal):
        self.boundary = boundary
        self.blocks = blocks
        
        self.res = 0.1
        sizex = int(np.ceil((self.boundary[0,3] - self.boundary[0,0])/self.res ))
        sizey = int(np.ceil((self.boundary[0,4] - self.boundary[0,1])/self.res ))
        sizez = int(np.ceil((self.boundary[0,5] - self.boundary[0,2])/self.res ))
        self.mmap = np.zeros([sizex,sizey,sizez],dtype=np.int8)
        self.cost = np.zeros([sizex,sizey,sizez],dtype=np.float64)
        self.start = tuple(num_to_grid(start,self.res,self.boundary))
        self.goal = tuple(num_to_grid(goal,self.res,self.boundary))
        for i in range(blocks.shape[0]):
            x1 = max(int(np.ceil((self.blocks[i,0]-self.res - self.boundary[0,0])/self.res) -1),0)
            x2 = max(int(np.ceil((self.blocks[i,3]+self.res - self.boundary[0,0])/self.res) -1),0)
            y1 = max(int(np.ceil((self.blocks[i,1]-self.res - self.boundary[0,1])/self.res) -1),0)
            y2 = max(int(np.ceil((self.blocks[i,4]+self.res - self.boundary[0,1])/self.res) -1),0)
            z1 = max(int(np.ceil((self.blocks[i,2]-self.res - self.boundary[0,2])/self.res) -1),0)
            z2 = max(int(np.ceil((self.blocks[i,5]+self.res - self.boundary[0,2])/self.res) -1),0)
            self.mmap[x1:x2+1,y1:y2+1,z1:z2+1] = 1   
        self.mmap[tuple(self.goal)] = 0
        self.V = [self.start]
        self.E = {}
        self.path = {}
        
    def plan(self,start,goal):
        s_in = num_to_grid(start,self.res,self.boundary)
        newrobotpos = np.copy(s_in)
        tt = tic()
        if self.goal not in self.V:
            while True:
                x_rand = samplefree(self.mmap, self.goal)
                x_near = nearest(self.V,x_rand)
                x_new = steer(x_near,x_rand,self.res,self.boundary)
                if collisionfree(x_near,x_new,self.mmap,self.res,self.boundary):
                    Xnear = near(self.V,x_new,1,self.res,self.boundary)# self.r
                    self.V.append(tuple(x_new))
                    c_min = self.cost[tuple(x_near)] + dist_grid(x_near,x_new,self.res,self.boundary)
                    x_min = x_near
                    for x in Xnear:
                        if collisionfree(x,x_new,self.mmap,self.res,self.boundary):
                            if self.cost[tuple(x)] + dist_grid(x,x_new,self.res,self.boundary) < c_min:
                                x_min = x
                                c_min = self.cost[tuple(x)] + dist_grid(x,x_new,self.res,self.boundary)
                    self.cost[tuple(x_new)] = c_min
                    self.E[tuple(x_new)] = tuple(x_min)
                    for x in Xnear:
                        if collisionfree(x,x_new,self.mmap,self.res,self.boundary):
                            if self.cost[tuple(x_new)] + dist_grid(x_new,x,self.res,self.boundary) < self.cost[tuple(x)]:
                                self.E[tuple(x)] = tuple(x_new)
                if self.goal in self.V:
                    break
                if tic() - tt > 1.5:
                    break
        elif self.path == {}:
            keys = self.E.keys()
            tem = self.goal
            while tem in keys and tem != self.start:
                f = self.E[tem]
                self.path[f] = tem
                tem = f
        else:
            newrobotpos = np.array(self.path[tuple(s_in)])
        
        return newrobotpos
    
#    def plan(self,start,goal):
#        newrobotpos = np.copy(s_in)
#        n = 2000
#        tt = tic()
#        while True:
#            x_rand = samplefree(self.mmap, self.goal)
#            x_near = nearest(self.V,x_rand)
#            x_new = steer(x_near,x_rand,self.res,self.boundary)
#            if self.mmap(x_new) == 0:
#                Xnear = near(self.V,x_new,1,self.res,self.boundary)# self.r
#                self.V.append(x_new)
#                c_min = self.cost[tuple(x_near)] + dist(x_near,x_new)
#                for x in Xnear:
#                    if collisionfree(x,x_new):
#                        if self.cost[tuple(x)] + dist(x,x_new) < c_min:
#                            x_min = x
#                            c_min = self.cost[tuple(x)] + dist(x,x_new)
#                self.cost[tuple(x_new)] = c_min
#                self.E[x_new] = x_min
#                for x in Xnear:
#                    if collisionfree(x,x_new):
#                        if self.cost(tuple(x_new)) + dist(x_new,x) < self.cost(tuple(x)):
#                            self.E[x] = x_new
#            if x_new = self.goal:
#                self.path = [self.goal]
#                tem = self.goal
#                while tem in D.keys():
#                    tem = self.E[tem]
#                    self.path.append(tem)
#                break
#            if tic() - tt > 1.9:
#                break
#        newrobotpos = 
#        return newrobotpos
    
    
    
    
    
    