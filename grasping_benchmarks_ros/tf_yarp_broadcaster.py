#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

# As test can use rosrun tf tf_echo camera world

import rospy 
import yarp 
import tf2_ros
import numpy as np 
import geometry_msgs.msg

if __name__ == "__main__":

    # Init node
    rospy.init_node('tf_yarp_broadcaster')

    # Create Broadcaster 
    br = tf2_ros.TransformBroadcaster()

    # Initialize yarp network 
    yarp.Network.init()

    # Yarp stuffs 
    robot_pose_port = yarp.BufferedPortVector()
    robot_pose_port.open("/robot_pose")
    
    # Correct code - uncomment when working with iCub
    """
    yarp.Network.connect("/realsense-holder-publisher/pose:o", "/robot_pose")
    yarp_pose = yarp.Vector(7)
    np_pose = np.zeros(7)

    yarp_pose = robot_pose_port.read()

    for i,data in enumerate(yarp_pose):
        np_pose[i] = data 

    q = axis_angle_to_quaternion(np_pose[3:])
    """ 

    # Create geometry message 
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot"
    t.child_frame_id = "camera"
    
    # Correct code
    """
    t.transform.translation.x = np_pose[0]
    t.transform.translation.y = np_pose[1]
    t.transform.translation.z = np_pose[2]
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    """ 

    # Test code
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 1.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 0.0

    # Run at 1 Hz
    r = rospy.Rate(1) 
    while not rospy.is_shutdown():
        br.sendTransform(t)
        r.sleep()

