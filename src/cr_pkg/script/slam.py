#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import numpy as np


from cr_pkg.msg import sensorfusion
from nav_msgs.msg import OccupancyGrid
import tf

# map width in meters
MAP_WIDTH = 4992

# map height in meters
MAP_HEIGHT = 4992

# the resolution of the map (the size (length and width) of each cell in the occupancy grid in meters per cell)
RESOLUTION = 0.2

# the origin coordinates of the map
MAP_ORIGIN_X = 0.03 - RESOLUTION * MAP_WIDTH / 2
MAP_ORIGIN_Y = 0.03 - RESOLUTION * MAP_HEIGHT / 2

# the number of the cells in the X-axis
CELLS_X = MAP_WIDTH

# the number of the cells in the Y-axis
CELLS_Y = MAP_HEIGHT


class SLAM:
    def __init__(self):
        self.odom = Odometry()
        # initialize the robot's position
        self.x = 0
        self.y = 0
        self.theta = 0
        self.prev_x = 0
        self.prev_y = 0
        self.prev_theta = 0
        self.prev_time = 0
        self.time = 0

        rospy.loginfo("After Initialize Odometry")
        # initialize the laser scan
        self.laser = LaserScan()
        self.ranges = []
        self.intensities = []
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.time_increment = 0
        self.scan_time = 0
        self.range_min = 0
        self.range_max = 0

        rospy.loginfo("After Initialize Laser Scan")

        # initialize the robot's velocity
        self.v = 0
        self.w = 0

        # initialize the robot's covariance
        self.sigma_x = 0
        self.sigma_y = 0

        rospy.loginfo("After Initialize Covariance")

        self.hits = np.full((CELLS_X, CELLS_Y), 1)
        self.misses = np.full((CELLS_X, CELLS_Y), 1)

        rospy.loginfo("After Initialize Hits and Misses")

        self.map_publisher = rospy.Publisher(
            '/map_slam', OccupancyGrid, queue_size=1)
        self.pose_publiser = rospy.Publisher(
            '/pose_slam', Odometry, queue_size=1)

        rospy.loginfo("After Initialize Map Publisher")
        self.sensor_fusion_subscriber = rospy.Subscriber(
            '/sensor_output', sensorfusion, callback=self.sensor_callback2)

    def sensor_callback2(self, msg):
        self.odom = msg.odm_data
        self.getPoseKF()
        self.update_laser(msg.laser_scan_data)

        readings = self.ranges
        angle_min = self.angle_min
        angle_max = self.angle_max
        angle_increment = self.angle_increment
        range_min = self.range_min
        range_max = self.range_max

        # initialize the map
        map = OccupancyGrid()
        map.header.frame_id = "robot_map"
        map.header.stamp = rospy.Time.now()
        map.info.resolution = RESOLUTION
        map.info.width = CELLS_X
        map.info.height = CELLS_Y

        x_robot = self.x
        y_robot = self.y

        self.hits = np.full((CELLS_X, CELLS_Y), 1)
        self.misses = np.full((CELLS_X, CELLS_Y), 1)

        for i in range(len(readings)):
            if readings[i] >= range_max or readings[i] <= range_min:
                continue
            
            angle = angle_min + i * angle_increment + self.theta

            end_x = x_robot + readings[i] * math.cos(angle)
            end_y = y_robot + readings[i] * math.sin(angle)

            map_x = int((end_x - MAP_ORIGIN_X) / RESOLUTION)
            map_y = int((end_y - MAP_ORIGIN_Y) / RESOLUTION)
            if map_x < 0 or map_x >= CELLS_X or map_y < 0 or map_y >= CELLS_Y:
                continue


            self.hits[map_y, map_x] += 1

            for j in range(1, int(readings[i]/RESOLUTION)):
                x = x_robot + j * RESOLUTION * math.cos(angle)
                y = y_robot + j * RESOLUTION * math.sin(angle)

                map_x = int((x - MAP_ORIGIN_X) / RESOLUTION)
                map_y = int((y - MAP_ORIGIN_Y) / RESOLUTION)

                if map_x < 0 or map_x >= CELLS_X or map_y < 0 or map_y >= CELLS_Y:
                    continue

                self.misses[map_y, map_x] += 1
        rospy.loginfo("After Update Map")
        # publish the map
        self.publish_map()
            
    
    

    def getPoseKF(self):
        # get data
        self.time = self.odom.header.stamp.secs 
        self.vX = self.odom.twist.twist.linear.x
        self.vY = self.odom.twist.twist.linear.y
        self.w = self.odom.twist.twist.angular.z
        self.obs_x = self.odom.pose.pose.position.x
        self.obs_y = self.odom.pose.pose.position.y
        orientation_z = self.odom.pose.pose.orientation
        self.obs_theta =  tf.transformations.euler_from_quaternion([orientation_z.x, orientation_z.y, orientation_z.z, orientation_z.w])[2]
       
        # predict state
        dt = self.time - self.prev_time
        self.x = self.prev_x + self.vX * dt * math.cos(self.theta)
        self.y = self.prev_y + self.vX * dt * math.sin(self.theta)
        self.theta = self.theta + self.w * dt
        # correct state
        k  = 0.8
        self.x = self.x + k * (self.obs_x - self.x)
        self.y = self.y + k * (self.obs_y - self.y)
        self.theta = self.theta + k * (self.obs_theta - self.theta)
        # update prev state
        self.prev_time = self.time
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta


    def update_laser(self, msg):
        self.laser = msg
        self.ranges = msg.ranges
        self.intensities = msg.intensities
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.time_increment = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        rospy.loginfo("updatinnnnnng")

    def update_odom(self, msg):
        self.odom = msg
        #self.x = msg.pose.pose.position.x
        #self.y = msg.pose.pose.position.y
        #orientation_z = msg.pose.pose.orientation
        #self.theta = tf.transformations.euler_from_quaternion([orientation_z.x, orientation_z.y, orientation_z.z, orientation_z.w])[2]
        self.time = msg.header.stamp.secs 
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        self.sigma_x = msg.pose.covariance[0]
        self.sigma_y = msg.pose.covariance[7]
        
        if self.prev_time != 0:
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_theta = self.theta
            self.prev_time = self.time
            
        else:
            self.prev_time = self.time
    
    def prediction(self):
        dt = self.time - self.prev_time
        self.x = self.x + self.v * dt * math.cos(self.theta)
        self.y = self.y + self.v * dt * math.sin(self.theta)
        self.theta = self.theta + self.w * dt
    
    def correction(self):
        K = 0.5
        
        
        self.x = self.x + K * (self.odom.pose.pose.position.x - self.x)
        self.y = self.y + K * (self.odom.pose.pose.position.y - self.y)
        orientation_z = self.odom.pose.pose.orientation
        self.theta = self.theta + K * (tf.transformations.euler_from_quaternion([orientation_z.x, orientation_z.y, orientation_z.z, orientation_z.w])[2] - self.theta)
    
    
    
    def publish_map(self):
        grid_map = ((self.hits / (self.hits + self.misses) * 100)).astype(np.int8)
        
        msg = OccupancyGrid()
        msg.header.frame_id = 'robot_map'
        msg.data = grid_map.flatten().tolist()

        
        # message metadata
        msg.info.resolution = RESOLUTION
        msg.info.width = CELLS_X
        msg.info.height = CELLS_Y
        msg.info.origin.position.x = MAP_ORIGIN_X
        msg.info.origin.position.y = MAP_ORIGIN_Y
        
        # publish the map
        self.map_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('slam')
    slam = SLAM()
    rospy.spin()