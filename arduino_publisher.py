#!/usr/bin/env python

#Program Spec: Initiate a node, subscribe to /glove data, whenever I receive a point from glove data, substitute it for the marker end point and publish the marker to /visualization_marker

#I am playing with symbolic links

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


msg = Marker()
image = Marker()
start = Point()
end = Point()


def callback(data):
    global end

    end.x = data.x/100.0
    end.y = data.y/100.0
    end.z = data.z/100.0



def main():
    #init publisher
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.init_node('glove_publisher')
    rospy.Subscriber("glove_data", Point, callback)
    rate = rospy.Rate(100)  #Publish rate

    global msg
    global start
    global end

    #prep Arrow message
    msg.header.frame_id = "magnetometer_frame"
    msg.ns = "space"
    ''' #Set by default
    msg.id = 0 #Must be unique for each message
    msg.type = Marker.ARROW
    msg.action = Marker.ADD
    '''
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.1   #Shaft radius
    msg.scale.y = 0.5   #head radius
    msg.scale.z = 0.5   #head length
    msg.color.a = 1.0
    msg.color.r = 1.0

    start.x = start.y = start.z = 0.0
    end.x = end.y = end.z = 2.0

    #Assign the memory addresses, not just the values, to the message
    msg.points.append(start)
    msg.points.append(end)

    #Prep Ring Mesh Message
    image.header.frame_id = "magnetometer_frame"
    image.ns = "space"
    image.id = 1
    image.type = Marker.MESH_RESOURCE
    image.action = Marker.ADD
    image.pose.orientation.x = 0.00004
    image.pose.orientation.y = 0.9
    image.pose.orientation.z = 0.00004
    image.pose.orientation.w = 0.000000002
    image.scale.x = 0.05
    image.scale.y = 0.05
    image.scale.z = 0.05
    image.color.a = 1.0
    image.color.g = 1.0
    image.mesh_resource = "file:///home/id314/catkin_ws/src/glove_visualizer/resources/ring2.stl"


    #loop
    while not rospy.is_shutdown():
        #Stamp Message
        msg.header.stamp = rospy.Time.now()
        image.header.stamp = rospy.Time.now()

        #publish message
        pub.publish(msg)
        pub.publish(image)
        rate.sleep()













if __name__ == '__main__':
    main()
