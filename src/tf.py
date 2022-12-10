#!/usr/bin/env python
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    object_name = rospy.get_param('~object')
    topic_name = rospy.get_param('~topic')
    text_name = rospy.get_param('~text')
    marker_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)
    first_run = True
    marker = Marker()
    markerArray = MarkerArray()

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 0.7
    marker.scale.y = 0.7
    marker.scale.z = 0.7
    marker.color.a = 1.0
    marker.color.r = 0.2
    marker.color.g = 0.2
    marker.color.b = 0.5

    #marker2 = Marker()
    #marker2.header.frame_id = "map"
    #marker2.type = marker.SPHERE
    #marker2.action = marker.ADD
    #marker2.scale.x = 1.0
    #marker2.scale.y = 1.0
    #marker2.scale.z = 1.0
    #marker2.color.a = 0.5
    #marker2.color.r = 1.0
    #marker2.color.g = 0.38
    #marker2.color.b = 0.27
    # Set the pose of the marker

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', object_name, rospy.Time())
            

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            #print("Exception")
            continue



        marker.pose.position.x = trans.transform.translation.x
        marker.pose.position.y = trans.transform.translation.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = trans.transform.rotation.x
        marker.pose.orientation.y = trans.transform.rotation.y
        marker.pose.orientation.z = trans.transform.rotation.z
        marker.pose.orientation.w = trans.transform.rotation.w
        marker.text = text_name
        #marker_pub.publish(marker)

        #marker2.pose.position.x = trans2.transform.translation.x
        #marker2.pose.position.y = trans2.transform.translation.y
        #marker2.pose.position.z = 0
        #marker2.pose.orientation.x = trans2.transform.rotation.x
        #marker2.pose.orientation.y = trans2.transform.rotation.y
        #marker2.pose.orientation.z = trans2.transform.rotation.z
        #marker2.pose.orientation.w = trans2.transform.rotation.w
        if first_run:
                last_x = trans.transform.translation.x
                last_y = trans.transform.translation.y
                first_run = False
                markerArray.markers.append(marker)
        if (abs(last_x) - abs(trans.transform.translation.x)) > 2:
        
            #if (abs(last_y) - abs(trans.transform.translation.y)) > 0.5:

            markerArray.markers.append(marker)
            last_x = trans.transform.translation.x
            #markerArray.markers.append(marker2)
        else:
            last_x = last_x

        ids = 0
        for m in markerArray.markers:
            m.id = ids
            ids += 1
        marker_pub.publish(markerArray)
        
        # print(trans)

        rate.sleep()
