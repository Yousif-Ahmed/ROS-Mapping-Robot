#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from cr_pkg.msg import sensorfusion
# from tf.transformations import translation_matrix, quaternion_matrix

import numpy as np
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


class Mapper:
    def __init__(self):
        self.hits = np.full((CELLS_X, CELLS_Y), 1)
        self.misses = np.full((CELLS_X, CELLS_Y), 1)
        self.map_publisher = rospy.Publisher('/map_topic', OccupancyGrid)
        self.sensors_subscriber = None
        self.sensors_subscriber = rospy.Subscriber('/sensor_output', sensorfusion, callback = self.sensor_callback, queue_size=100)

    def publish_map(self):
        grid_map = (np.round_((self.hits / (self.hits + self.misses)) * 100)).astype(np.int8)
        
        msg = OccupancyGrid()
        msg.header.frame_id = 'robot_map'
        msg.header.stamp = rospy.Time.now()
        msg.data = grid_map.flatten().tolist()
        
        # message metadata
        msg.info.resolution = RESOLUTION
        msg.info.width = CELLS_X
        msg.info.height = CELLS_Y
        msg.info.origin.position.x = MAP_ORIGIN_X
        msg.info.origin.position.y = MAP_ORIGIN_Y
        
        
        # publish the map
        self.map_publisher.publish(msg)
    

    def sensor_callback(self, msg):
        # get the position of the sensor
        laser_data = msg.laser_scan_data
        odom_data = msg.odm_data
        rospy.loginfo(f"odom =  {msg}")

        # robot's pose
        x_robot = odom_data.pose.pose.position.x
        y_robot = odom_data.pose.pose.position.y
        rospy.loginfo("x_robot = %f", x_robot)
        rospy.loginfo("y_robot = %f", y_robot)
    
        orientation_z = odom_data.pose.pose.orientation
        orientation_z = tf.transformations.euler_from_quaternion([orientation_z.x, orientation_z.y, orientation_z.z, orientation_z.w])[2]
        rospy.loginfo("orientation_z = %f", orientation_z)

        angle_min = laser_data.angle_min
        angle_max = laser_data.angle_max
        angle_increment = laser_data.angle_increment
        
        readings = laser_data.ranges
        range_max = laser_data.range_max
        range_min = laser_data.range_min

        

        # calculate the cells that the laser hits
        for i in range(len(readings)):
            if readings[i] >= range_max or readings[i] <= range_min:
                continue
            
            angle = angle_min + i * angle_increment + orientation_z

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

        # publish the map
        self.publish_map()

    

    def run(self):
        # rospy.Rate(10)
        rospy.spin()




if __name__ == '__main__':
    rospy.init_node('mapping_with_known_poses')
    # rospy.loginfo('mapping_with_known_poses node started')
    
    
    mapper = Mapper()
    mapper.run()


    # translation_matrix = translation_matrix([x, y, 0])
    # quaternion_matrix = quaternion_matrix([orientation_x, orientation_y, orientation_z, orientation_w])

    # transformation_matrix = np.dot(translation_matrix, quaternion_matrix)

    # angles = np.array([x for x in np.arange(angle_min, angle_max, angle_increment)])

    # measurement_x = np.cos(angles) * readings
    # measurement_y = np.sin(angles) * readings

    # laser_points = np.array([measurement_x, measurement_y, np.zeros(len(measurement_x)), np.ones(len(measurement_x))])
    
    # result = np.dot(transformation_matrix, laser_points)

    # laser_x = result[0]
    # laser_y = result[1]

# No transform from [map] to [robot_map]