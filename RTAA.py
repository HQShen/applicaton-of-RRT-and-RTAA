# -*- coding: utf-8 -*-
"""
Created on Fri May 17 15:46:19 2019

@author: user
"""


import numpy as np
from util import *

class RobotPlanner:
    
    def __init__(self, boundary, blocks, start, goal):
        self.boundary = boundary
        self.blocks = blocks
        self.s = start
        self.g = goal
        self.res = 0.25
        self.sizex = int(np.ceil((self.boundary[0,3] - self.boundary[0,0])/self.res ))
        self.sizey = int(np.ceil((self.boundary[0,4] - self.boundary[0,1])/self.res ))
        self.sizez = int(np.ceil((self.boundary[0,5] - self.boundary[0,2])/self.res ))
        self.mmap = np.zeros([self.sizex,self.sizey,self.sizez],dtype=np.int8)
        self.heu = np.zeros([self.sizex,self.sizey,self.sizez],dtype=np.float64)
        self.start = tuple(num_to_grid(start,self.res,self.boundary))
        self.goal = tuple(num_to_grid(goal,self.res,self.boundary))
        for i in range(blocks.shape[0]):
            x1 = max(int(np.ceil((self.blocks[i,0] - self.res - self.boundary[0,0])/self.res) -1),0)
            x2 = max(int(np.ceil((self.blocks[i,3] + self.res - self.boundary[0,0])/self.res) -1),0)
            y1 = max(int(np.ceil((self.blocks[i,1] - self.res - self.boundary[0,1])/self.res) -1),0)
            y2 = max(int(np.ceil((self.blocks[i,4] + self.res - self.boundary[0,1])/self.res) -1),0)
            z1 = max(int(np.ceil((self.blocks[i,2] - self.res - self.boundary[0,2])/self.res) -1),0)
            z2 = max(int(np.ceil((self.blocks[i,5] + self.res - self.boundary[0,2])/self.res) -1),0)
            self.mmap[x1:x2+1,y1:y2+1,z1:z2+1] = 1
        # self.mmap[self.goal] = 0
        self.OPEN = []
        self.CLOSED = []
        self.path = {}
        
    def plan(self,start,goal):
        if dist(start,self.g) < 1:
            return self.goal
        s_in = num_to_grid(start,self.res,self.boundary)
        g_in = num_to_grid(goal,self.res,self.boundary)
        map_g = np.zeros([self.sizex,self.sizey,self.sizez],dtype=np.float64)
        self.OPEN = [tuple(s_in)]
        self.CLOSED = []
        self.path = {}
        self.heu[self.goal] = 0
        newrobotpos = np.copy(s_in)
        n = 500
        for t in range(n):
            x_expand = self.findmin(self.heu, map_g, self.OPEN)
            self.OPEN.remove(x_expand)
            self.CLOSED.append(x_expand)
            map_g, self.path, self.OPEN, self.heu = self.expand(x_expand, map_g, self.heu, self.OPEN,\
                                           self.CLOSED, self.mmap, self.path,self.goal)
        xj = self.findmin(self.heu, map_g, self.OPEN)
        self._update_heu(xj,map_g)
        newrobotpos = self._findpos(xj, tuple(s_in))

        return newrobotpos
    
    def findmin(self, heu, map_g, OPEN):
        tv = np.inf
        for node in OPEN:
            if heu[node] + map_g[node] < tv:
                tv = heu[node] + map_g[node]
                x_min = node
        return x_min
    
    def expand(self, father, map_g, heu, OPEN, CLOSED, mmap, path, goal):
        [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
        dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
        dR = np.delete(dR,13,axis=1)
        numofdirs = 26
        for k in range(numofdirs):
            newrp = tuple(father + dR[:,k])
            if not self.valid(newrp) or newrp in CLOSED:
                continue
            if heu[newrp] == 0:
                heu[newrp] = dist(np.array(newrp), np.array(goal))
            if map_g[father] + dist(np.array(newrp), np.array(father)) < map_g[newrp] or map_g[newrp] == 0:
                map_g[newrp] = map_g[father] + dist(np.array(newrp), np.array(father))
                path[newrp] = father
                OPEN.append(newrp)
        return map_g, path, OPEN, heu
    
    def valid(self, newrp):
        v = True
        if newrp[0] < 0 or newrp[1] < 0 or newrp[2] < 0 or\
        newrp[0] > self.mmap.shape[0]-1 or\
        newrp[1] > self.mmap.shape[1]-1 or\
        newrp[2] > self.mmap.shape[2]-1:
            v = False
        elif self.mmap[newrp] == 1:
            v = False
        return v
            
    def _update_heu(self, xj, map_g):
        fj = map_g[xj] + self.heu[xj]
        for node in self.CLOSED:
            self.heu[node] = fj - map_g[node]
        
    def _findpos(self, xj, s_in):
        '''
        return : np.array
        '''
        tem = self.path[xj]
        if tem == s_in:
            return np.array(tem)
        while tem in self.path.keys():
            if self.path[tem] == s_in:
                return np.array(tem)
            else:
                tem = self.path[tem]
            
#    def update_heu(self, heu, map_g, xj, path, s_in):
#        tem = path[xj]
#        fj = map_g[xj] + heu[xj]
#        if tem in path.keys():
#            heu[tem] = fj - map_g[tem]
#            tem = path[tem]
#        heu[tem] = fj - map_g[tem]
#        return heu
            
            











