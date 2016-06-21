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
import json # for reading config file
import rospy # ROS
import argparse # to parse command line arguments
import signal # catching SIGINT signal
import logging # log messages
from ss_script_handler import ss_script_handler # plays back script lines
from ss_ros import ss_ros

class ss_game_node():
    """ The SAR social stories main game node orchestrates the game: what the
    robot is told to do, what is loaded on the tablet, what to do in response
    to stuff that happens on the tablet or from other sensors.

    This node sends ROS messages to a SAR Opal game via a rosbridge_server
    websocket connection, and uses ROS to exchange messages with other relevant
    nodes (such as the node that translates robot commands to specific robot
    platforms). """

    # set up ROS node globally 
    # TODO if running on network where DNS does not resolve local
    # hostnames, get the public IP address of this machine and
    # export to the environment variable $ROS_IP to set the public
    # address of this node, so the user doesn't have to remember 
    # to do this before starting the node.
    ros_node = rospy.init_node('social_story_game', anonymous=True)
            # we could set the ROS log level here if we want
            # the rest of our logging is set up in the log config file
            #log_level=rospy.DEBUG)

    def __init__(self):
        """ Initialize anything that needs initialization """
        # set up logger
        self.logger = logging.getLogger(__name__)
        # configure logging
        try:
            config_file = "ss_log_config.json"
            with open(config_file) as json_file:
                json_data = json.load(json_file)
                logging.config.dictConfig(json_data)
                self.logger.debug("==============================\n" +
                    "STARTING\nLogger configuration:\n %s", json_data)
        except Exception as e:
            # could not read config file -- use basic configuration
            logging.basicConfig(filename="ss.log", 
                    level=logging.DEBUG)
            self.logger.exception("ERROR! Could not read your json log " 
                + "config file \"" + config_file + "\". Does the file "
                + "exist? Is it valid json?\n\nUsing default log setup to "
                + "log to \"ss.log\". Will not be logging to rosout!")


    def parse_arguments_and_launch(self):
        # Parse python arguments 
        # The game node requires the session number and participant ID be
        # provided so the appropriate game scripts can be loaded.
        parser = argparse.ArgumentParser(
                formatter_class=argparse.RawDescriptionHelpFormatter,
                description='Start the SAR Social Stories game node, which '
                + 'orchestrates the game: loads scripts, uses ROS to tell the '
                + 'robot and tablet what to do.\nRequires roscore to be running' 
                + ' and requires rosbridge_server for communication with the ' 
                + 'SAR opal tablet (where game content is shown).')
        parser.add_argument('session', action='store', 
               nargs='?', type=int, default=-1, help='Indicate which session this'
               + ' is so the appropriate game scripts can be loaded.')
        parser.add_argument('participant', 
               action='store', nargs='?', type=str, default='DEMO', help=
               'Indicate which participant this is so the appropriate game '
               + 'scripts can be loaded.')

        # parse the args we got, and print them out
        args = parser.parse_args()
        self.logger.debug("Args received: %s", args)

        # give the session number and participant ID to the game launcher
        # where they will be used to load appropriate game scripts
        #
        # if the session number doesn't make sense, or we've specified that
        # this is a demo, run demo
        if args.session < 0 or args.participant == 'DEMO':
            self.launch_game(-1, 'DEMO')
        # otherwise, launch the game for the provided session and ID
        else:
            self.launch_game(args.session, args.participant)


    def launch_game(self, session, participant):
        """ Load game based on the current session and participant """
        # log session and participant ID
        self.logger.info("==============================\nSOCIAL STORIES " +
            "GAME\nSession: %s, Participant ID: %s", session, participant)

        # setup ROS node publisher and subscriber
        self.ros_ss = ss_ros(self.ros_node)

        # read config file to get relative file path to game scripts
        try:
            config_file = "ss_config.demo.json" if participant == "DEMO" \
                    else "ss_config.json"
            with open(config_file) as json_file:
                json_data = json.load(json_file)
                self.logger.debug("Reading game config file...: %s", json_data)
                if ("script_path" in json_data):
                    self.script_path = json_data["script_path"]
                else:
                    self.logger.error("Could not read relative path to game "
                        + "scripts! Expected option \"script_path\" to be in "
                        + "the config file. Exiting because we need the "
                        + "scripts to run the game.")
                    return
                if ("story_script_path" in json_data):
                    self.story_script_path = json_data["story_script_path"]
                else:
                    self.logger.error("Could not read path to story scripts! "
                        + "Expected option \"story_script_path\" to be in "
                        + "config file. Assuming story scripts are in the main"
                        + " game script directory and not a sub-directory.")
                    self.story_script_path = None
                if ("session_script_path" in json_data):
                    self.session_script_path = json_data["session_script_path"]
                else:
                    self.logger.error("Could not read path to session scripts! "
                        + "Expected option \"session_script_path\" to be in config"
                        + "config file. Assuming session scripts are in the main"
                        + "game script directory and not a sub-directory.")
                    self.session_script_path = None
        except Exception as e:
            self.logger.exception("Could not read your json config file \"" 
                + config_file + "\". Does the file exist? Is it valid json?"
                + " Exiting because we need the config file to run the game.")
            return

        # start game
        try: 
            self.script_handler = ss_script_handler(self.ros_ss, session,
                participant, self.script_path, self.story_script_path,
                self.session_script_path)
        except IOError as e:
            self.logger.exception("Did not load the session script... exiting "
                + "because we need the session script to run the game.")
        else:
            # flag to indicate whether we should exit
            self.stop = False

            # set up signal handler to catch SIGINT (e.g., ctrl-c)
            signal.signal(signal.SIGINT, self.signal_handler)

            while (not self.stop):
                try:
                    self.script_handler.iterate_once()

                except StopIteration:
                    self.logger.info("Finished script!")
                    break


    def signal_handler(self, sig, frame):
        """ Handle signals caught """
        if sig == signal.SIGINT:
            self.logger.info("Got keyboard interrupt! Exiting.")
            self.stop = True
            exit("Interrupted by user.")


if __name__ == '__main__':
    # try launching the game
    try:
        game_node = ss_game_node()
        game_node.parse_arguments_and_launch()

    # if roscore isn't running or shuts down unexpectedly
    except rospy.ROSInterruptException: 
        self.logger.exception('ROS node shutdown')
        pass

