#!/usr/bin/env python
#######################################################################################
#The following code is based on tutorials from wiki.ros.org for 
#   1) tf tutorials in Python 
#   2) marker visualization tutorials in Python
#######################################################################################


import rospy                                        #ROS Library for Python
import tf2_ros                                      #ROS Library for Tf listening
from visualization_msgs.msg import Marker           #ROS Library for Markers
from visualization_msgs.msg import MarkerArray      #ROS Library for MarkerArrays


if __name__ == '__main__':
    rospy.init_node('tf_listener')                  #Node name

    tfBuffer = tf2_ros.Buffer()                     #Using Buffer function from tf2_ros
    listener = tf2_ros.TransformListener(tfBuffer)  #Creating a transform listener instance

    rate = rospy.Rate(10.0)                         #Defining rate of publishing 
    object_name = rospy.get_param('~object')        #Getting object name from namespace param
    topic_name = rospy.get_param('~topic')          #Getting topic name for object from namespace param
    text_name = rospy.get_param('~text')            #Getting text name for object from namespace param
    marker_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)    #Publisher for publishing markerarray
    first_run = True                                #Variable to enable multiple detections of same image
    marker = Marker()                               #Marker initialisaion
    markerArray = MarkerArray()                     #MarkerArray Initialisation

    
    marker.header.frame_id = "map"                  #Publishing markers under map frame id
    marker.type = marker.TEXT_VIEW_FACING           #Choosing text as the marker type
    marker.action = marker.ADD                      #Add the chosen marker

    #The following set of definitions are for defining the size and color of the marker text
    marker.scale.x = 0.7                            
    marker.scale.y = 0.7
    marker.scale.z = 0.7
    marker.color.a = 1.0
    marker.color.r = 0.2
    marker.color.g = 0.2
    marker.color.b = 0.5



    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', object_name, rospy.Time()) #Listen for tf between map and object
            

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            #print("Exception")
            continue


        #The following definitions assign the coordinates of object's tf frame to the marker
        marker.pose.position.x = trans.transform.translation.x 
        marker.pose.position.y = trans.transform.translation.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = trans.transform.rotation.x
        marker.pose.orientation.y = trans.transform.rotation.y
        marker.pose.orientation.z = trans.transform.rotation.z
        marker.pose.orientation.w = trans.transform.rotation.w
        marker.text = text_name
        
        if first_run:           #Only works for the first time an object is detected
                last_x = trans.transform.translation.x  #Storing the current coordinates
                last_y = trans.transform.translation.y  #Storing the current coordinates
                first_run = False
                markerArray.markers.append(marker)      #Append the Marker to the MarkerArray

        #The following condition is to check if the object is detected at a different location or the same location
        if (abs(last_x) - abs(trans.transform.translation.x)) > 2:  #To enable multiple detection based on location
        
            #if (abs(last_y) - abs(trans.transform.translation.y)) > 0.5:

            markerArray.markers.append(marker)          #Append the new coordinate to the MarkerArray
            last_x = trans.transform.translation.x      #If different location stre current coordinates
            
        else:
            last_x = last_x

        ids = 0
        for m in markerArray.markers:                   #For defining ids for MultipleArray
            m.id = ids
            ids += 1
        marker_pub.publish(markerArray)                 #Publish the Marker Array
        
        # print(trans)

        rate.sleep()
