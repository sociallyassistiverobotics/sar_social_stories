# Jacqueline Kory Westlund
# May 2016
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Personal Robots Group
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy # ROS
from sar_opal_msgs.msg import OpalCommand # ROS msgs to talk to game
from sar_opal_msgs.msg import OpalAction # ROS msgs for game actions 
from sar_robot_command_msgs.msg import RobotCommand # ROS msgs for robot cmd
from sar_robot_command_msgs.msg import RobotState # ROS msgs for robot state

class ss_ros():
    # ROS node
    # set up rostopics we publish: commands to the game (on a tablet or on a
    # PC/touchscreen), and commands to the robot
    game_pub = rospy.Publisher('opal_tablet_command', OpalCommand,
            queue_size = 10)
    robot_pub = rospy.Publisher('robot_command', RobotCommand, queue_size = 10)


    def __init__(self, ros_node):
        """ Initialize ROS """
        # we get a reference to the main ros node so we can do callbacks
        # to publish messages, and subscribe to stuff
        self.ros_node = ros_node

        # subscribe to other ros nodes
        # TODO could we put list of nodes to subscribe to in config file?
        # subscribe to messages from opal game
        rospy.Subscriber('opal_tablet_action', OpalAction, self.on_opal_action_msg)
        # subscribe to messages about the robot's state
        rospy.Subscriber('robot_state', RobotState, self.on_robot_state_msg)


    def send_opal_message(self, command):
        """ Publish opal command message """
        print("sending opal command: " + command)
        msg = OpalCommand()
        msg.command = command
        # TODO opal command messages often take properties, add these!
        # TODO use sar_opal_sender as examples of how to add properties
        self.game_pub.publish(msg)
        rospy.loginfo(msg)

    
    def send_opal_message_and_wait(self, command, timeout):
        """ Publish opal command message and wait for a response """
        print("TODO send opal command and wait")
      

    def send_robot_command(self, command, timeout):
        """ Publish robot command message """
        print("sending robot command: " + command)
        msg = RobotCommand()
        msg.command = command
        # TODO add header
        # TODO robot command messages may take properties, add these!
        # TODO use robot_command_sender as examples of how to add properties
        self.robot_pub.publish(msg)
        rospy.loginfo(msg)


    def send_robot_command_and_wait(self, command):
        """ Publish robot command message and wait for a response """
        print("TODO send robot command and wait")


    def on_opal_action_msg(data):
        """ Called when we receive OpalAction messages """
        print("TODO received OpalAction message!")


    def on_robot_state_msg(data):
        """ Called when we receive RobotState messages """
        print("TODO received RobotState message!")
        # when we get robot state messages, set a flag indicating whether the
        # robot is in motion or playing sound or not
        #self.flags.robot_is_playing_sound = data.is_playing_sound
        #self.flags.robot_doing_action = data.doing_action
