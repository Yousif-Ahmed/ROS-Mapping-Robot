#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cr_pkg.msg import sensorfusion
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SensorFusionClass:
    def __init__(self):
        self.laser_sub = Subscriber("/scan_multi", LaserScan)
        self.odom_sub = Subscriber("/robot/robotnik_base_control/odom", Odometry)
        self.sensor_pub = rospy.Publisher("/sensor_output", sensorfusion, queue_size=10)

        self.laser_data = None
        self.odom_data = None

    def laser_callback(self, msg):
        self.laser_data = msg

    def odom_callback(self, msg):
        self.odom_data = msg

    def sensor_callback(self,msg1 , msg2 ):
        msg = sensorfusion()
        msg.laser_scan_data = msg1
        msg.odm_data = msg2
        msg.header = rospy.Header()
        msg.header.stamp = rospy.Time.now()
        
        self.sensor_pub.publish(msg)

    def run(self):
        # rate = rospy.Rate(10)
        ts = ApproximateTimeSynchronizer([self.laser_sub, self.odom_sub], 10, 0.01, allow_headerless=False)
        ts.registerCallback(self.sensor_callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("sensor_fusion")
    sensor_fusion = SensorFusionClass()
    sensor_fusion.run()
  