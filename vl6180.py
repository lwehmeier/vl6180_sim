#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16
import struct
from nav_msgs.msg import OccupancyGrid
import numpy as np
class VL6180:
    def __init__(self, angle):
        self.distance = 255
        self.res = 0.1
        self.angle = angle
    def update(self, grid):
        angle = self.angle
        cx = len(grid)/2
        cy = len(grid[0])/2
        if angle == "lf":
            tiles = [(cx + 1, cy), (cx + 1, cy+1), (cx +2, cy+2)]
        if angle == "lr":
            tiles = [(cx + 1, cy), (cx + 1, cy-1), (cx +2, cy-2)]
        for i in range(0,3):
            if grid[tiles[i][0], tiles[i][1]] > 90:
                self.distance = 50 + 100*i
                self.interrupt()
                return
        self.distance = 255
    def interrupt(self):
        rospy.Publisher("/platform/e_stop", Int16, queue_size=1).publish(Int16(1))
    def setGridRes(self, res):
        self.res = res


vl_lf = VL6180("lf")
vl_lr = VL6180("lr")

def callbackTimer(event):
    rospy.Publisher("/platform/distance/lf", Int16, queue_size = 1).publish(Int16(vl_lf.distance))
    rospy.Publisher("/platform/distance/lr", Int16, queue_size=1).publish(Int16(vl_lr.distance))
def callbackCM(costmap):
    m = grid2matrix(costmap)
    vl_lf.update(m)
    vl_lr.update(m)

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

rospy.init_node('vl6180')
rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callbackCM)
rospy.Timer(rospy.Duration(0.5), callbackTimer)
r = rospy.Rate(10) # 10hz
rospy.spin()
