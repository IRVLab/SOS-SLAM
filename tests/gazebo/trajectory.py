#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import print_function
import rospy
import sys
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3

if __name__ == '__main__':
    print('Apply programmed perturbation to vehicle', rospy.get_namespace())
    rospy.init_node('set_body_wrench')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
    except rospy.ROSException:
        print('Service not available! Closing node...')
        sys.exit(-1)

    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    except rospy.ServiceException as e:
        print('Service call failed, error=', e)
        sys.exit(-1)

    wrench = Wrench()
    wrench.torque = Vector3(0, 0, 0)

    rate = rospy.Rate(1)
    while True:
        directions = [[1,0,0], [0,1,0], [-1,0,0], [0,-1,0]]
        for d in directions:
            for t in range(-10,11,1):
                wrench.force = Vector3(t/10*d[0],t/10*d[1],t/10*d[2])
                apply_wrench('data_rig_link', 'world', Point(0, 0, 0), wrench, rospy.Time().now(), rospy.Duration(1))
                rate.sleep()
