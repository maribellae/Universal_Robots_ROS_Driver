#!/usr/bin/env python
import sys
import time
import rospy
from robotiq_2f_gripper_msgs.msg import RobotiqGripperCommand
from robotiq_2f_gripper_msgs.action import CommandRobotiqGripper
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


class GripperPosControl:
    def __init__(self):
      
        rospy.init_node("gripper") #ur3_joints
        action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripper) #CommandRobotiqGripperAction
      
        # Wait until grippers are ready to take command
        robotiq_client.wait_for_server()
        rospy.logwarn("Client test: Starting sending goals")
      
        ## ROS Subscriber Topic
        # Subscribe to the ar_pose_marker topic to get the image width and height

        self.ur_sub_gripper = rospy.Subscriber('gripper',RobotiqGripperCommand, self.callback_gripper) #CommandRobotiqGripperGoal
        
        ## ROS parameters
        self.rate = rospy.Rate(1) # Hz
       
        ######self.Position_control()
        
    def callback_gripper(self, msg):      #обработка смс 
        self.robotiq_client.send_goal(msg) 
   
    '''def Position_control(self):
        """ Control the robot"""
        while not rospy.is_shutdown():
            self.rate.sleep()
        rospy.signal_shutdown("We are done here!")'''

if __name__ == '__main__':

    try:
        client = GripperControl()
        ######client.Position_control()
    except rospy.ROSInterruptException:
        pass


