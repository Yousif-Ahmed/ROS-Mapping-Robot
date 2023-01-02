#!/usr/bin/env python3
import math

import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from cr_pkg.msg import SensorData


# Map width in meters
MAP_WIDTH = 1000

# Map height in meters
MAP_HEIGHT = 1000

# The resolution of the map (the size (length and width) of each cell in the occupancy grid in meters per cell)
RESOLUTION = 0.2

# The origin coordinates of the map
MAP_ORIGIN_X = 0.03 - RESOLUTION * MAP_WIDTH / 2
MAP_ORIGIN_Y = -0.032 - RESOLUTION * MAP_HEIGHT / 2

# The number of the cells in the X-axis
CELLS_X = MAP_WIDTH

# The number of the cells in the Y-axis
CELLS_Y = MAP_HEIGHT


class Mapper:
    def __init__(self):
        self.hits = np.full((CELLS_X, CELLS_Y), 1)
        self.misses = np.full((CELLS_X, CELLS_Y), 1)

        self.sensors_subscriber = rospy.Subscriber('/sensor_output', SensorData, callback=self.sensor_callback, queue_size=1)
        self.map_publisher = rospy.Publisher('/map_topic', OccupancyGrid , queue_size=1)

        self.laser_data = LaserScan()
        self.odom_data  = Odometry()

    def sensor_callback(self, msg):
        self.laser_data = msg.laser_scan_data
        self.odom_data = msg.odom_data

    def update_map(self):
        x_robot = self.odom_data.pose.pose.position.x
        y_robot = self.odom_data.pose.pose.position.y

        orientation_z = self.odom_data.pose.pose.orientation
        orientation_z = tf.transformations.euler_from_quaternion([orientation_z.x, orientation_z.y, orientation_z.z, orientation_z.w])[2]

        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        readings = self.laser_data.ranges
        range_max = self.laser_data.range_max
        range_min = self.laser_data.range_min

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

            # calculate the cells that the laser misses
            for j in range(1, int(readings[i]/RESOLUTION)):
                x = x_robot + j * RESOLUTION * math.cos(angle)
                y = y_robot + j * RESOLUTION * math.sin(angle)

                map_x = int((x - MAP_ORIGIN_X) / RESOLUTION)
                map_y = int((y - MAP_ORIGIN_Y) / RESOLUTION)

                if map_x < 0 or map_x >= CELLS_X or map_y < 0 or map_y >= CELLS_Y:
                    continue

                self.misses[map_y, map_x] += 1

        self.publish_map()

    def publish_map(self):
        grid_map = (np.round_((self.hits / (self.hits + self.misses)) * 100)).astype(np.int8)

        occupancy_map = OccupancyGrid()
        occupancy_map.header.frame_id = 'robot_map'
        occupancy_map.header.stamp = rospy.Time.now()
        occupancy_map.data = grid_map.flatten().tolist()

        # set the map meta data
        occupancy_map.info.resolution = RESOLUTION
        occupancy_map.info.width = CELLS_X
        occupancy_map.info.height = CELLS_Y
        occupancy_map.info.origin.position.x = MAP_ORIGIN_X
        occupancy_map.info.origin.position.y = MAP_ORIGIN_Y

        # publish the map
        self.map_publisher.publish(occupancy_map)

    def run(self):
        rate = rospy.Rate(2)
        
        while not rospy.is_shutdown():
            self.update_map()
            rate.sleep()
        
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('mapping_with_known_poses')
    print("Mapping with known poses node started")
    
    mapper = Mapper()

    try:
        mapper.run()
    except rospy.ROSInterruptException:
        pass
