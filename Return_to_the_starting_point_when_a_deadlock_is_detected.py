#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class NavigationNode:
    def __init__(self):
        rospy.init_node('dead_end_detector')
        
        # Подписки на данные SLAM
        self.sub_odom = rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.odom_cb)
        self.sub_cloud = rospy.Subscriber('/livox/points', PointCloud2, self.cloud_cb)
        
        self.current_pose = None
        self.current_cloud = None

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def cloud_cb(self, msg):
        self.current_cloud = msg

    def run(self):
        rate = rospy.Rate(5)  # 5 Гц
        while not rospy.is_shutdown():
            if self.current_cloud and self.current_pose:
                if is_dead_end(self.current_cloud):
                    rospy.logerr("Тупик! Инициирую возврат.")
                    emergency_return()
                    break
            rate.sleep()

if __name__ == "__main__":
    node = NavigationNode()
    node.run()