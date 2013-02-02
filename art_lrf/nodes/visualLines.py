#!/usr/bin/env python
import roslib; roslib.load_manifest('art_lrf')
import rospy
from std_msgs.msg import String
from art_lrf.msg import Lines
from pylab import *
from keyboard import keyboard

def callback(data):
    print data.est_rho
    plotLines(data)

def visualLines():
    rospy.init_node('visualLines', anonymous=True)
    rospy.Subscriber("lrfLines", Lines , callback)
    rospy.spin()

def plotLines(data):
    perpDist = lambda x,y,rho, theta: ((x*cos(theta) + y*sin(theta)) - rho)
    imageSize = [100,100]
    yScale = 1./10
    xScale = 1./10
    distTol = 0.1
    center = [int(imageSize[0]/2), int(imageSize[1]/2)]
    grid = [[0 for j in range(imageSize[1])] for i in range(imageSize[0])]
    grid[center[0]][center[1]] = 10
    for i in range(imageSize[0]):
        for j in range(imageSize[1]):
            for k in range(len(data.est_rho)):
                d = perpDist((j - center[1])*xScale, (i - center[0])*yScale, data.est_rho[k], data.est_theta[k])
                if abs(d) < distTol:
                    grid[i][j] += 1
    imshow(grid)
    show()
    keyboard()
                

if __name__ == '__main__':
    visualLines()
