#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from pcl_helper import *

def publish_label(label_text, position):
        marker_arr = MarkerArray()
        label_marker = Marker()
        label_marker.header.frame_id = 'base'
        label_marker.header.stamp = rospy.Time.now()
        label_marker.ns = 'Text'
        label_marker.id = 1
        label_marker.type = Marker.TEXT_VIEW_FACING
        label_marker.action = Marker.ADD
        label_marker.text = label_text
        label_marker.scale.x = 0.5
        label_marker.scale.y = 0.5
        label_marker.scale.z = 0.5
        label_marker.color.r = 1.0
        label_marker.color.g = 1.0
        label_marker.color.b = 1.0
        label_marker.color.a = 1.0
        label_marker.pose.position.x = position[0]
        label_marker.pose.position.y = position[1]
        label_marker.pose.position.z = position[2]
        label_marker.pose.orientation.x = 0.0
        label_marker.pose.orientation.y = 0.0
        label_marker.pose.orientation.z = 0.0
        label_marker.pose.orientation.w = 0.5
        marker_pub.publish([label_marker])
        print('published!')


if __name__ == '__main__':

        marker_pub = rospy.Publisher('/rviz_visual_tools', MarkerArray, queue_size=1)
        rospy.init_node('marker_test', anonymous=True)

        #ros_cloud =  rospy.wait_for_message('/sensor_stick/point_cloud', PointCloud2)
        # print('got msg!')
        # pcl_cloud = ros_to_pcl(ros_cloud)
        publish_label('this damned plane', [1,1,1])

        while not rospy.is_shutdown():
                rospy.spin()
