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
import argparse # to parse command line arguments
from ss_logger import ss_logger # for logging data
from ss_personalization_manager import ss_personalization_manager
from ss_session_manager import ss_session_manager
from ss_game_manager import ss_game_manager
from ss_ros import ss_ros

# The SAR Social Stories game node orchestrates the game: what the robot is
# told to do, what is loaded on the tablet, what to do in response to stuff
# that happens on the tablet or from other sensors. 
#
# This node sends ROS messages to a SAR Opal tablet via a rosbridge_server
# websocket connection, and uses ROS to exchange messages with other relevant
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


    def ss_game_node():
        # Parse python arguments 
        # The game node requires the session number and participant ID be provided
        # so the appropriate game scripts can be loaded.
        parser = argparse.ArgumentParser(
                formatter_class=argparse.RawDescriptionHelpFormatter,
                description='Start the SAR Social Stories game node, which '
                + 'orchestrates the game: loads scripts, uses ROS to tell the robot'
                + 'and tablet what to do.\nRequires roscore to be running and ' 
                + 'requires rosbridge_server for communication with the SAR opal ' 
                + 'tablet (where game content is shown).')
        parser.add_argument('session', dest='session', action='store', 
               nargs=1, type=int, default=-1, help='Indicate which session this is'
               + ' so the appropriate game scripts can be loaded.')
        parser.add_argument('participant', dest='participant', 
               action='store', nargs=1, type=str, default='DEMO', help='Indicate '
               + ' which participant this is so the appropriate game scripts can '
               + 'be loaded.')

        # parse the args we got, and print them out
        args = parser.parse_args()
        print(args)

        # give the session number and participant ID to the game launcher
        # where they will be used to load appropriate game scripts
        #
        # if the session number doesn't make sense, or we've specified that
        # this is a test or demo, run demo
        if args.session < 0 or args.participant == 'DEMO':
            self.launch_game(-1, 'DEMO')
        # otherwise, launch the game for the provided session and ID
        else 
            self.launch_game(args.session, args.participant)


    def launch_game(session, participant):
        """ Load game based on the current session and participant """
        # set up logger
        self.logger = ss_logger(session, participant)

        # get list of session scripts from session manager
        self.game_manager = ss_session_manager(self.logger)
        session_scripts = self.session_manager.get_scripts(session)

        # get list of story scripts from personalization manager
        self.personalization_manager = ss_personalization_manager(self.logger)
        stories = self.personalization_manager.get_story_scripts(session,
                                                                 participant)

        # start game
        self.game_manager = ss_game_manager(self.logger, self.ros_ss,
                                            session_scripts, stories)
        self.game_manager.start()


    if __name__ == '__main__':
        # try launching the game
        try:
            ss_game_node()

        # if roscore isn't running or shuts down unexpectedly
        except rospy.ROSInterruptException: 
            print ('ROS node shutdown')
            pass

