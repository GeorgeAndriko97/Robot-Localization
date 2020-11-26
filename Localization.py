from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import numpy as np
import math
import rospy
import random
from util import rotateQuaternion, getHeading
from time import time


class PFLocaliser(PFLocaliserBase):
        
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        # ----- Set motion model parameters
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
	
	self.particlenumber=300
	#Mean and Variance
	mean = 0
	var = 5.0

	#Noises
	xnoise=0.5
	ynoise=0.5
	thetanoise=0.5
	#print(initialpose)
	
	#We are creating an array with Gaussian Random numbers
	self.poseArray = PoseArray()
	particle = []
	for i in range(self.particlenumber): 
		x = initialpose.pose.pose.position.x + np.random.normal(mean, var)*xnoise
		y = initialpose.pose.pose.position.y + np.random.normal(mean, var)*ynoise
		w = initialpose.pose.pose.orientation
		theta = random.random() * 360
		w= rotateQuaternion(w, math.radians(theta))
		particle = Pose()
		particle.position.x = x 
		particle.position.y = y
		particle.orientation = w
		self.poseArray.poses.append(particle)	
	#print(self.poseArray)	
	#print(self.poseArray.poses[0])
	#print("second value")
	#print(self.poseArray.poses[1])
	return(self.poseArray)
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
         """
	# We've calculated weights for each particle in the particlecloud	
	self.weights = [0] * self.particlenumber 
	for i in range(self.particlenumber): 
		self.weights[i] = self.sensor_model.get_weight(scan,self.poseArray.poses[i])

    	"""
	p_New_Weight = 0.0
	p_Old_weight = 0.0
  	Topweights = 0
		
	while j < range(self.particlenumber):
		
		p_Old_weight = self.WeightArray[[j],0]
		p_New_Weight = self.sensor_model.get_weight(scan, self.poseArray[[j],:])
		#p_New_Weight = self.sensor_model.get_weight(scan, self.poseArray.poses[[i],:])
				
		if p_New_Weight >= p_Old_weight:
			p_New_Weight += 1
		elif p_New_Weight < p_Old_weight:
			p_New_Weight -= 1
		
		self.WeightArray[[j],0] = p_New_Weight
		self.WeightArray[[j],1] = self.poseArray[[j],0]
		self.WeightArray[[j],2] = self.poseArray[[j],1]
		self.WeightArray[[j],3] = self.poseArray[[j],2]
	WeightArray = WeightArray[WeightArray[:,0].argsort()[::-1]] #Re-arrange in descending order
	
	for k in range(1000):
		if Topweights < 20:
			self.poseArray[[k],0] = self.WeightArray[[Topweights],1]
			self.poseArray[[k],1] = self.WeightArray[[Topweights],2]
			self.poseArray[[k],2] = self.WeightArray[[Topweights],3]
		else:
			Topweights -= 1
			
		Topweights += 1
		j += 1
	
	return(self.poseArray)
	"""
        pass


    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
	meanx = 0
	meany = 0
	meanz = 0
	meanw = 0
	#Get average of positions and orientations of each partical in the poseArray(ParticalCloud)
	for i in range(self.particlenumber): 
		#print(self.poseArray.poses[i])	
		meanx += self.poseArray.poses[i].position.x
		meany += self.poseArray.poses[i].position.y
		meanz += self.poseArray.poses[i].orientation.z
		meanw += self.poseArray.poses[i].orientation.w
		#meanw = getHeading(self.poseArray.poses[i].orientation)
		#print(meanz)
		#math.degrees(meanw)
		
		
	meanx=meanx/self.particlenumber
	meany=meany/self.particlenumber
	meanz=meanz/self.particlenumber
	meanw=meanw/self.particlenumber
	
	#We determined the location of our robot according to the average of particles' pos&orientation's 
	self.poseRobot = Pose()
	self.poseRobot.position.x = meanx 
	self.poseRobot.position.y = meany
	self.poseRobot.orientation.z = meanz
	self.poseRobot.orientation.w = -1 * meanw
        #print(self.poseRobot)
	return(self.poseRobot)

if __name__ == '__main__':
    n = PFLocaliser()
