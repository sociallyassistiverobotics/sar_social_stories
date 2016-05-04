#!/usr/bin/env python

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

import sys # exit and argv
import rospy # ROS

# The SAR Social Stories game node orchestrates the game: what the robot is
# told to do, what is loaded on the tablet, what to do in response to stuff
# that happens on the tablet or from other sensors. 
#
# Uses ROS to send messages to a SAR Opal tablet via a rosbridge_server
# websocket connection. Uses ROS to exchange messages with all other relevant
# nodes (such as the node that translates robot commands to specific robot
# platforms). 

class ss_game_node():
    """ Social stories main game node """
    # set up ROS node globally 
    # TODO if running on network where DNS does not resolve local
    # hostnames, get the public IP address of this machine and
    # export to the environment variable $ROS_IP to set the public
    # address of this node, so the user doesn't have to remember 
    # to do this before starting the node.
    ros_node = rospy.init_node('social_story_game', anonymous=True)

 def __init__(self):
        """ Initialize anything that needs initialization """
        # TODO initialize stuff

        # setup ROS node publisher and subscriber
        self.ros_ss = ss_ros(self.ros_node)


if __name__ == '__main__':
    
    # start running the game
    try:
        # TODO start game!

    # if roscore isn't running or shuts down unexpectedly
    except rospy.ROSInterruptException: 
        print ('ROS node shutdown')
        pass

    # enter main loop, then exit
    sys.exit(app.exec_())



