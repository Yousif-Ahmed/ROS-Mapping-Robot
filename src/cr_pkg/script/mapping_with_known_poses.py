#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from cr_pkg.msg import sensorfusion
# from tf.transformations import translation_matrix, quaternion_matrix

import numpy as np


# map width in meters
MAP_WIDTH = 4992

# map height in meters
MAP_HEIGHT = 4992  

# the resolution of the map (the size (length and width) of each cell in the occupancy grid in meters per cell)
RESOLUTION = 1.0

# the origin coordinates of the map
MAP_ORIGIN_X = -50.0
MAP_ORIGIN_Y = -50.0

# the number of the cells in the X-axis
CELLS_X = int(MAP_WIDTH / RESOLUTION)

# the number of the cells in the Y-axis
CELLS_Y = int(MAP_HEIGHT / RESOLUTION)


class Mapper:
    def __init__(self):
        rospy.loginfo('mapping_with_known_poses node started')
        self.hits = np.full((CELLS_X, CELLS_Y), 0)
        rospy.loginfo('map publisher created')
        self.misses = np.full((CELLS_X, CELLS_Y), 0)
        rospy.loginfo('map publisher created: misses')
        self.map_publisher = rospy.Publisher('/map_topic', OccupancyGrid)
        rospy.loginfo('map publisher created: hits')
        self.sensors_subscriber = None
        


    def publish_map(self):
        rospy.loginfo('publishing map')
        grid_map = (np.round_((self.hits / (self.hits + self.misses)) * 100)).astype(np.int8)
        
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
    

    def sensor_callback(self, msg):
        rospy.loginfo('sensor callback')
        # get the position of the sensor
        laser_data = msg.laser_scan_data
        odom_data = msg.odm_data

        # robot's pose
        x_robot = odom_data.pose.pose.position.x
        y_robot = odom_data.pose.pose.position.y

        robot_cell_x = int((x_robot - MAP_ORIGIN_X) / RESOLUTION)
        robot_cell_y = int((y_robot - MAP_ORIGIN_Y) / RESOLUTION)

        orientation_x = odom_data.pose.pose.orientation.x
        orientation_y = odom_data.pose.pose.orientation.y
        orientation_z = odom_data.pose.pose.orientation.z
        orientation_w = odom_data.pose.pose.orientation.w

        angle_min = laser_data.angle_min
        angle_max = laser_data.angle_max
        angle_increment = laser_data.angle_increment
        
        readings = laser_data.ranges
        range_max = laser_data.range_max
        range_min = laser_data.range_min

        angles = np.array([x for x in np.arange(angle_min, angle_max, angle_increment)])

        x_measurement = x_robot + readings * np.cos(angles)
        y_measurement = y_robot + readings * np.sin(angles)

        

        # calculate the cells that the laser hits
        for i in range(len(readings)):
            reading = readings[i]
            end_x = x_measurement[i]
            end_y = y_measurement[i]

            distance = reading / RESOLUTION

            dx = (end_x - x_robot)/distance
            dy = (end_y - y_robot)/distance
            xs = np.arange(x_robot, end_x + dx, dx).astype(np.int)
            ys = np.arange(y_robot, end_y + dy, dy).astype(np.int)
            min_length = min(len(xs), len(ys))
            xs = xs[:min_length]
            ys = ys[:min_length]
            self.misses[xs, ys] += 1

            hit_cell_x = int((end_x - MAP_ORIGIN_X) / RESOLUTION)
            hit_cell_y = int((end_y - MAP_ORIGIN_Y) / RESOLUTION)
            self.hits[hit_cell_x, hit_cell_y] += 1

        # publish the map
        self.publish_map()

    

    def run(self):
        rospy.loginfo('running')
        rospy.Rate(10)
        self.sensors_subscriber = rospy.Subscriber('/sensor_output', sensorfusion, callback = self.sensor_callback)
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