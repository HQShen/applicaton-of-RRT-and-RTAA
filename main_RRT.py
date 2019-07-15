# -*- coding: utf-8 -*-
"""
Created on Thu May 16 15:24:16 2019

@author: user
"""

import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import RRT
from util import *

def load_map(fname):
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = np.array(mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].tolist())
  blocks = np.array(mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].tolist())
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])  
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, verbose = True):
  # Instantiate a robot planner
  boundary, blocks = load_map(mapfile)
  RP = RRT.RobotPlanner(boundary, blocks,start,goal)
  
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)  
  
  # Main loop
  robotpos = np.copy(start)
  path = np.copy(start).reshape(1,3)
  numofmoves = 0
  distofmoves = 0
  timecost = 0
  success = True
  while True:
  
    # Call the robot planner
    t0 = tic()
    nr = RP.plan(robotpos, goal) # grid
    movetime = tic()-t0
    timecost += movetime
    print('move time: %f' % movetime)

    # See if the planner was done on time
    if movetime > 2:
      newrobotpos = robotpos-0.5 + np.random.rand(3)
    
    newrobotpos = grid_to_num(nr,RP.res,boundary)
    path = np.concatenate((path,newrobotpos.reshape(1,3)),axis = 0)
    distofmoves += sum((newrobotpos - robotpos)**2) 
    # Check if the commanded position is valid
    if sum((newrobotpos - robotpos)**2) > 1:
      print('ERROR: the robot cannot move so fast\n')
      success = False
    if nr[0] < 0 or nr[1] < 0 or nr[2] < 0 or\
            nr[0] > RP.mmap.shape[0]-1 or\
            nr[1] > RP.mmap.shape[1]-1 or\
            nr[2] > RP.mmap.shape[2]-1:
      print('ERROR: out-of-map robot position commanded\n')
      success = False
    for k in range(blocks.shape[0]):
      if RP.mmap[tuple(nr)]==1 :
        print('ERROR: collision... BOOM, BAAM, BLAAM!!!\n')
        success = False
        break
    if timecost > 24*60*60: # if it keeps finding the root for 1 day, then it fail
        success = False
    if( success is False ):
      break
    
    # Make the move
    robotpos = newrobotpos
    numofmoves += 1
    
    # Update plot
    if verbose:
      hs[0].set_xdata(robotpos[0])
      hs[0].set_ydata(robotpos[1])
      hs[0].set_3d_properties(robotpos[2])
      fig.canvas.flush_events()
      plt.show()
      
    # Check if the goal is reached
    if tuple(nr) == RP.goal:
      nodes = np.array(RP.V)
      nodes = grids_to_nums(nodes, RP.res, boundary)
      fig = plt.figure()
      ax = fig.add_subplot(111, projection='3d')
      hb = draw_block_list(ax,blocks)
      pp = ax.plot(path[:,0],path[:,1],path[:,2],'r',markersize=3,markeredgecolor='k')
      pp = ax.plot(nodes[:,0],nodes[:,1],nodes[:,2],'go',markersize=1,markeredgecolor='k')
      hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
      hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')
      ax.set_xlabel('X')
      ax.set_ylabel('Y')
      ax.set_zlabel('Z')
      ax.set_xlim(boundary[0,0],boundary[0,3])
      ax.set_ylim(boundary[0,1],boundary[0,4])
      ax.set_zlim(boundary[0,2],boundary[0,5])
      fig.canvas.flush_events()
      plt.show()
      break

  return success, numofmoves, distofmoves, timecost, len(RP.V)


def test_single_cube():    
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 6.0])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/single_cube.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves)
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)
  
def test_maze():
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/maze.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves)
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)
  
def test_window():
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/window.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves)
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)
  
def test_tower():
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/tower.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves)
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)
 
def test_flappy_bird():
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/flappy_bird.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves) 
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)

def test_room():
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/room.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves)
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)

def test_monza():
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  success, numofmoves,distofmoves,timecost,l = runtest('./maps/monza.txt', start, goal, True)
  print('Success: %r'%success)
  print('Number of Moves: %i'%numofmoves)
  print('Distance of Moves: %f'%distofmoves)
  print('Total time: %f'%timecost)
  print('number of nodes: %i'%l)

if __name__=="__main__":
  #test_single_cube()
  #test_maze()
  #test_flappy_bird()
  test_monza()
  #test_window()
  #test_tower()
  #test_room()
