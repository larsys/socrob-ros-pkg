#!/usr/bin/env python

import sys
import math
import pickle
import os
import wx
import time

import re
import fcntl
import subprocess
import numpy as np
import matplotlib.pyplot as pyplot
import numpy.linalg as la
from Tkinter import *
from tkFont import *

import roslib; roslib.load_manifest('raposang_slam')
import rospy

from raposang_msgs.msg import RaposaSlam
from sensor_msgs.msg import PointCloud

LIM = 8

class Viewer(Frame):

	pose_list = [[],[],[]]
	pose_odo_list = [[],[],[]]
	
	pose = np.array([0.0, 0.0, 0.0])
	pose_odo = np.array([0.0, 0.0, 0.0])
	
	cov = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

	rot = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
	rot_odo = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
	
	ld_cov = []
	landmarks = []
	removal = []
	
	def __init__(self):
		self.name = "lala"

	def main(self):
				
		# Main loop
		pyplot.subplot(121)
		pyplot.hold(False)
		pyplot.plot(0.0, 0.0, label="slamxy")
		self.plotsettingszx()	
		pyplot.hold(True)
		pyplot.plot(0.0, 0.0, 'ro', label="posexy")
		
		pyplot.subplot(122)
		pyplot.hold(False)
		pyplot.plot(0.0, 0.0, label="slamyz")
		self.plotsettingsxy()
		pyplot.hold(True)
		pyplot.plot(0.0, 0.0, 'ro', label="poseyz")		
		
		# Setup ROS
		rospy.init_node('raposang_slam_monitor', anonymous=True)
		rospy.Subscriber("/raposang_slam/pose", RaposaSlam, self.handler_pose, queue_size=1)
		rospy.Subscriber("/raposang_slam/landmarks", PointCloud, self.handler_landmarks, queue_size=1)

		pyplot.show()

	def plotsettingszx(self):
		
		pyplot.title("Top View")
		pyplot.xlabel("z (meters)")
		pyplot.ylabel("x (meters)")
		pyplot.xlim((-2,8))
		pyplot.ylim((-8,2))
#		pyplot.xlim((-1,3))
#		pyplot.ylim((-2,2))
		pyplot.grid(True)		
				
	def plotsettingszy(self):
		
		pyplot.title("Top View")
		pyplot.xlabel("z (meters)")
		pyplot.ylabel("y (meters)")
		pyplot.xlim((-2,8))
		pyplot.ylim((-5,5))
		pyplot.grid(True)	
				
	def plotsettingsxy(self):
		
		pyplot.title("Side View")
		pyplot.xlabel("z (meters)")
		pyplot.ylabel("y (meters)")
		pyplot.xlim((-8,2))
		pyplot.ylim((-5,5))
#		pyplot.xlim((-1,3))
#		pyplot.ylim((-2,2))
		pyplot.grid(True)		

	def toRotMatrix(self, qw, qx, qy, qz):
		
		xx2 = 2.0 * qx * qx
		yy2 = 2.0 * qy * qy	
		zz2 = 2.0 * qz * qz
		
		wx2 = 2.0 * qw * qx
		wy2 = 2.0 * qw * qy	
		wz2 = 2.0 * qw * qz	
			
		xy2 = 2.0 * qx * qy
		yz2 = 2.0 * qy * qz
		zx2 = 2.0 * qz * qx
		
		r11 = 1 - yy2 - zz2
		r12 = xy2 - wz2
		r13 = zx2 + wy2
		r21 = xy2 + wz2
		r22 = 1 - xx2 - zz2
		r23 = yz2 - wx2
		r31 = zx2 - wy2
		r32 = yz2 + wx2
		r33 = 1 - xx2 - yy2
		
		return np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

	def drawcov(self, scale, cov_color, rx, ry, P):

		pi = 3.1415
		
		evals, evecs = np.linalg.eig(P)
		X = evecs

		t = np.arange(0, 2.0*pi, 0.1)

		cov_x = []
		cov_y = []

		for n in t:
			cov_x.append(scale*(evals[0]**0.5 * X[0,0] * np.cos(n) + evals[1]**0.5 * X[0,1] * np.sin(n)) + rx)
			cov_y.append(scale*(evals[0]**0.5 * X[1,0] * np.cos(n) + evals[1]**0.5 * X[1,1] * np.sin(n)) + ry)

		pyplot.fill(cov_x, cov_y, color=cov_color)

	def plotslam(self, x, y):
		
		pyplot.hold(False)
		pyplot.plot(self.pose_list[x], self.pose_list[y], 'k')
		pyplot.hold(True)
		pyplot.plot(self.pose_odo_list[x], self.pose_odo_list[y], 'blue')
		pyplot.hold(True)
		pyplot.arrow(self.pose[x], self.pose[y], self.rot[x,0], self.rot[y,0], color='r')
		pyplot.hold(True)
		pyplot.arrow(self.pose[x], self.pose[y], self.rot[x,1], self.rot[y,1], color='g')
		pyplot.hold(True)
		pyplot.arrow(self.pose[x], self.pose[y], self.rot[x,2], self.rot[y,2], color='b')
		pyplot.hold(True)
		pyplot.arrow(self.pose_odo[x], self.pose_odo[y], 0.5*self.rot_odo[x,0], 0.5*self.rot_odo[y,0], color='r')
		pyplot.hold(True)
		pyplot.arrow(self.pose_odo[x], self.pose_odo[y], 0.5*self.rot_odo[x,1], 0.5*self.rot_odo[y,1], color='g')
		pyplot.hold(True)
		pyplot.arrow(self.pose_odo[x], self.pose_odo[y], 0.5*self.rot_odo[x,2], 0.5*self.rot_odo[y,2], color='b')

	def update(self):
			
		self.pose_list[0].append(self.pose[0])
		self.pose_list[1].append(self.pose[1])
		self.pose_list[2].append(self.pose[2])

		self.pose_odo_list[0].append(self.pose_odo[0])
		self.pose_odo_list[1].append(self.pose_odo[1])
		self.pose_odo_list[2].append(self.pose_odo[2])
		
		pyplot.subplot(121)

		x = 2
		y = 0
		self.plotslam(x, y)
		self.plotsettingszx()			
		i = 0			
		for ld in self.landmarks:
				pyplot.hold(True)
				pyplot.plot(ld.z, ld.x, 'go', color=str(1.0-self.removal[i]))
				self.drawcov(1.0, 'yellow', ld.z, ld.x, np.array([[self.ld_cov[x*3+x+9*i], self.ld_cov[x*3+y+9*i]], [self.ld_cov[y*3+x+9*i], self.ld_cov[y*3+y+9*i]]]))
				i += 1			
		pyplot.hold(True)
		self.drawcov(10.0, 'purple', self.pose[x], self.pose[y], np.array([[self.cov[x*3+x], self.cov[x*3+y]], [self.cov[y*3+x], self.cov[y*3+y]]]))		
				
		pyplot.subplot(122)
		x = 0
		y = 1
		self.plotslam(x, y)		
		self.plotsettingsxy()	
		i = 0			
		for ld in self.landmarks:
				pyplot.hold(True)
				pyplot.plot(ld.x, ld.y, 'go', color=str(1.0-self.removal[i]))
				self.drawcov(1.0, 'yellow', ld.x, ld.y, np.array([[self.ld_cov[x*3+x+9*i], self.ld_cov[x*3+y+9*i]], [self.ld_cov[y*3+x+9*i], self.ld_cov[y*3+y+9*i]]]))
				i += 1			
		pyplot.hold(True)
		self.drawcov(10.0, 'purple', self.pose[x], self.pose[y], np.array([[self.cov[x*3+x], self.cov[x*3+y]], [self.cov[y*3+x], self.cov[y*3+y]]]))				


		pyplot.draw()
		
	def handler_landmarks(self, landmarks_msg):
		self.landmarks = landmarks_msg.points
		self.removal = landmarks_msg.channels[0].values
		self.ld_cov = landmarks_msg.channels[1].values
		self.update()
		
	def handler_pose(self, pose_msg):
		self.pose = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
		self.pose_odo = np.array([pose_msg.pose_odometry.position.x, pose_msg.pose_odometry.position.y, pose_msg.pose_odometry.position.z])		
		self.rot = self.toRotMatrix(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z)
		self.rot_odo = self.toRotMatrix(pose_msg.pose_odometry.orientation.w, pose_msg.pose_odometry.orientation.x, pose_msg.pose_odometry.orientation.y, pose_msg.pose_odometry.orientation.z)
		self.cov = pose_msg.covariance


if __name__ == '__main__':
	Viewer().main()
