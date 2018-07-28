#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16
import struct
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
class VL6180:
    def __init__(self, angle):
        self.distance = 255
        self.res = 0.1
        self.angle = angle
    def update(self, grid, cx, cy):
        angle = self.angle
        if angle == "lf":
            tiles = [(cx + 1, cy+1), (cx + 1, cy+2)]#, (cx +2, cy+2)]
        if angle == "lr":
            tiles = [(cx + 1, cy-1), (cx + 1, cy-2)]#, (cx +2, cy-2)]
        if angle == "rf":
            tiles = [(cx - 1, cy+1), (cx - 1, cy+2)]#, (cx -2, cy+2)]
        if angle == "rr":
            tiles = [(cx - 1, cy-1), (cx - 1, cy-2)]#, (cx -2, cy-2)]
        if angle == "fl":
            tiles = [(cx+1, cy+1), (cx+2, cy+1)]#, (cx +2, cy+2)]
        if angle == "fr":
            tiles = [(cx-1, cy+1), (cx-2, cy+1)]#, (cx -2, cy+2)]
        if angle == "r_l":
            tiles = [(cx+1, cy-1), (cx+2, cy-1)]#, (cx +2, cy-2)]
        if angle == "r_r":
            tiles = [(cx-1, cy-1), (cx-2, cy-1)]#, (cx -2, cy-2)]
        for i in range(0,3):
            if grid[tiles[i][0], tiles[i][1]] > 95:
                self.distance = 50 + 100*i
                self.interrupt()
                print(tiles[i])
                return
        self.distance = 255
    def interrupt(self):
        print("VL at "+ str(self.angle) +" detected obstacle. Stopping platform..")
        e_stop.publish(Int16(1))
    def setGridRes(self, res):
        self.res = res


vl_array = [
    VL6180("lf"),
    VL6180("lr"),
    VL6180("rf"),
    VL6180("rr"),
    VL6180("fl"),
    VL6180("fr"),
    VL6180("r_l"),
    VL6180("r_r")
    ]

def callbackTimer(event):
    for vl in vl_array:
        rospy.Publisher("/platform/distance/"+str(vl.angle), Int16, queue_size = 1).publish(Int16(vl.distance))
def callbackCM(costmap):
    global cmap
    cmap = costmap
    m = grid2matrix(costmap)
    center_x = cmap.info.width/2 -1 
    center_y = cmap.info.height/2 -1
    for vl in vl_array:
        vl.update(m, center_x, center_y)
def callbackUpdate(costmap_u):
    if cmap is None:
        print("waiting for initial costmap")
        return
    if costmap_u.x != 0 or costmap_u.y != 0:
        print("cannot handle costmap update")
        return
    global cmap
    cmap.data = costmap_u.data
    callbackCM(cmap)

def grid2matrix(occupancy_grid):
    res = occupancy_grid.info.resolution
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = occupancy_grid.data
    matrix = []
    for x in range(0,width):
        matrix.append([])
        for y in range(0,height):
            mval = data[x*width + y]
            matrix[x].append(mval)
    return np.array(matrix)

global cmap
cmap=None
rospy.init_node('vl6180')
e_stop = rospy.Publisher("/platform/e_stop", Int16, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callbackCM, queue_size=5)
rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate, callbackUpdate, queue_size=5)
rospy.Timer(rospy.Duration(0.5), callbackTimer)
r = rospy.Rate(10) # 10hz
rospy.spin()
