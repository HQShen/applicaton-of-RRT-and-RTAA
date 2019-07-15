# -*- coding: utf-8 -*-
"""
Created on Mon May 13 15:28:11 2019

@author: user
"""
import numpy as np
import time
import random

def num_to_grid(x,res,boundary):
    ans = np.zeros(3)
    ans[0] = max(int(np.ceil((x[0] - boundary[0,0])/res) -1),0)
    ans[1] = max(int(np.ceil((x[1] - boundary[0,1])/res) -1),0)
    ans[2] = max(int(np.ceil((x[2] - boundary[0,2])/res) -1),0)
    return ans.astype(np.int)

def dist(x,y):
    return np.linalg.norm(x-y)

def grid_to_num(x,res,boundary):
    ans = np.zeros(3)
    ans[0] = x[0] * res + boundary[0,0] + res/2
    ans[1] = x[1] * res + boundary[0,1] + res/2
    ans[2] = x[2] * res + boundary[0,2] + res/2
    return ans

def grids_to_nums(x,res,boundary):
    ans = np.zeros(x.shape)
    ans[:,0] = x[:,0] * res + boundary[0,0] + res/2
    ans[:,1] = x[:,1] * res + boundary[0,1] + res/2
    ans[:,2] = x[:,2] * res + boundary[0,2] + res/2
    return ans

def dist_grid(x,y,res,boundary):
    xx = grid_to_num(x,res,boundary)
    yy = grid_to_num(y,res,boundary)
    return dist(xx,yy)

def tic():
  return time.time()

def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

def samplefree(mmap, goal):
    sizex = mmap.shape[0]
    sizey = mmap.shape[1]
    sizez = mmap.shape[2]
    while True:
        x = random.choice(range(sizex))
        y = random.choice(range(sizey))
        z = random.choice(range(sizez))
        if mmap[x,y,z] == 0:
            tem = (x,y,z)
            ans = random.choices([tem,goal],weights = [0.9,0.1])
            break
    return np.array(ans[0])

def nearest(X,x_rand):
    d = np.inf
    for x in X:
        x = np.array(x)
        d_tem = dist(x,x_rand)
        if d_tem < d:
            x_near = x
            d = d_tem
    return x_near

def steer(x_near,x_rand, res, boundary):
    x1 = grid_to_num(x_near, res, boundary)
    x2 = grid_to_num(x_rand, res, boundary)
    tem = x2 - x1
    if np.linalg.norm(tem) < 1:
        x_new = x_rand
    else:
        x_new = x1 + 0.8 * tem/np.linalg.norm(tem)
        x_new = num_to_grid(x_new, res, boundary)
    return x_new

def near(V,x_new,r, res,boundary):
    Xnear = []
    x_new = grid_to_num(x_new, res,boundary)
    for v in V:
        v = np.array(v)
        vv = grid_to_num(v, res,boundary)
        if dist(vv,x_new) < r:
            Xnear.append(v)
    return Xnear

def collisionfree(x1,x2,mmap,res,boundary):
    if mmap[tuple(x2)] == 1 or mmap[tuple(x1)] == 1:
        return False
    pin = True
    xx1 = grid_to_num(x1, res, boundary)
    xx2 = grid_to_num(x2, res, boundary)
    tem = xx2 - xx1
    tem_len = np.linalg.norm(tem)
    for i in range(int(tem_len/ res)):
        xx_mid = xx1 + res * i * tem/tem_len
        x_mid = num_to_grid(xx_mid, res, boundary)
        if mmap[tuple(x_mid)] == 1:
            pin = False
            break
    return pin











