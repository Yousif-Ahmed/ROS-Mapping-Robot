#!/usr/bin/env python3

import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from cr_pkg.msg import SensorData


class SensorFusion:
    def __init__(self):
        self.laser_subscriber = Subscriber("/scan_multi", LaserScan)
        self.odom_subscriber = Subscriber("/robot/robotnik_base_control/odom", Odometry)
        self.sensor_publisher = rospy.Publisher("/sensor_output", SensorData, queue_size=10)

        self.laser_data = None
        self.odom_data = None

    def sensor_callback(self,msg1 , msg2 ):
        msg = SensorData()
        msg.laser_scan_data = msg1
        msg.odom_data = msg2
        msg.header = rospy.Header()
        msg.header.stamp = rospy.Time.now()
        
        self.sensor_publisher.publish(msg)

    def run(self):
        ts = ApproximateTimeSynchronizer([self.laser_subscriber, self.odom_subscriber], 10, 0.01, allow_headerless=False)
        ts.registerCallback(self.sensor_callback)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("sensor_fusion")
    print("Sensor fusion node started")
    
    sensor_fusion = SensorFusion()
    sensor_fusion.run()
  