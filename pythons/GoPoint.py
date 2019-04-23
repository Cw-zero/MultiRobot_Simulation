#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self,robot_id):
        # rospy.init_node('nav_test', anonymous=False)
        self.move_base = robot_id + "/move_base"
        self.map = robot_id + "_tf/map"
        self.goal_sent = False
        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient(self.move_base, MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        
        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], pos['z']), 
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        
        # Start moving
        self.move_base.send_goal(goal)
        
        # Allow TurtleBot up to 300 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
        
    def Point_Navigation(self,a_dict):
        # try:
        success = False
        # coordinate = tuple_x[1:3]
        tmp_pos = a_dict["pose"]
        tmp_ori = a_dict["orientation"]
        
        # Customize the following values so they are appropriate for your location
        position = {'x': tmp_pos[0], 'y' : tmp_pos[1], 'z' : tmp_pos[2]}
        quaternion = {'r1' : tmp_ori[0], 'r2' : tmp_ori[1], 'r3' : tmp_ori[2], 'r4' : tmp_ori[3]}
        
        # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = self.goto(position, quaternion)
        
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")
            
        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)
        return success
        
        # except rospy.ROSInterruptException:
        #     rospy.loginfo("Ctrl-C caught. Quitting")


