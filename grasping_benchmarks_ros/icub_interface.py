#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import yarp
import numpy
import time
import rospy
from geometry_msgs.msg import Transform
from grasping_benchmarks_ros.srv import ICubGrasp
from std_msgs.msg import Bool

def rh_grasp(request):

    laterality = 'right'
    robot_name = 'icubSim'

    props = yarp.Property()
    props.put('device', 'cartesiancontrollerclient')
    props.put('local', '/example/' + laterality + '_arm')
    props.put('remote', '/' + robot_name + '/cartesianController/' + laterality + '_arm')
    cart_driver = yarp.PolyDriver(props)
    cart = cart_driver.viewICartesianControl()
    cart.setTrajTime(2.0)

    position_yarp = yarp.Vector(3)
    for i in range(3):
        position_yarp[i] = request.translation[i]

    quat = request.rotation

    axis_angle_yarp = yarp.Vector(4)    
    axis_angle_yarp[0] = quat[0] / m.sqrt(1 - quat[3] * quat[3])
    axis_angle_yarp[1] = quat[1] / m.sqrt(1 - quat[3] * quat[3])
    axis_angle_yarp[2] = quat[2] / m.sqrt(1 - quat[3] * quat[3])
    axis_angle_yarp[3] = 2 * m.acos(quat[3])
    
    cart.goToPoseSync(position_yarp, axis_angle_yarp)

    return bool(1)


if __name__ == '__main__':

    rospy.init_node('icub_interface')
    yarp.Network.init()
    grasp_srv = ICubGrasp
    grasp_transform = Transform

    grasp_transform.translation.x = 0.0
    grasp_transform.translation.y = 0.0
    grasp_transform.translation.z = 0.0
    grasp_transform.rotation.x = 1.0
    grasp_transform.rotation.y = 0.0
    grasp_transform.rotation.z = 0.0
    grasp_transform.rotation.w = 0.0

    grasp_srv.request = grasp_transform
    icub_service = rospy.Service('/icub_grasp_server/rh_grasp', grasp_srv, rh_grasp)

    rospy.spin()
    
