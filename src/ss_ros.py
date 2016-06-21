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
import datetime # for header times and timeouts
import time # for sleep
import logging # log messages
from sar_opal_msgs.msg import OpalCommand # ROS msgs to talk to game
from sar_opal_msgs.msg import OpalAction # ROS msgs for game actions 
from sar_robot_command_msgs.msg import RobotCommand # ROS msgs for robot cmd
from sar_robot_command_msgs.msg import RobotState # ROS msgs for robot state
from std_msgs.msg import Header # standard ROS msg header

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

        # set up logger
        self.logger = logging.getLogger(__name__)
        self.logger.info("Subscribing to topics: opal_tablet_action, " +
            "robot_state")

        # subscribe to messages from opal game
        rospy.Subscriber('opal_tablet_action', OpalAction, 
                self.on_opal_action_msg)
        # subscribe to messages about the robot's state
        rospy.Subscriber('robot_state', RobotState, self.on_robot_state_msg)


    def send_opal_command(self, command, properties=None):
        """ Publish opal command message """
        self.logger.info("Sending opal command: " + command)
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Add appropriate command and properties if there are any.
        # We check properties for each command individually, since some
        # require properties, and if there are none, we shouldn't send 
        # the message. We assume any properties provided are in the 
        # correct format for the command.
        if "RESET" in command:
            msg.command = OpalCommand.RESET
        elif "DISABLE_TOUCH" in command:
            msg.command = OpalCommand.DISABLE_TOUCH
        elif "ENABLE_TOUCH" in command:
            msg.command = OpalCommand.ENABLE_TOUCH
        elif "SIDEKICK_DO" in command:
            msg.command = OpalCommand.SIDEKICK_DO
            # properties: a string with the name of action to do
        elif "SIDEKICK_SAY" in command:
            msg.command = OpalCommand.SIDEKICK_SAY
            # properties: a string with the name of audio file to play
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a "
                    + "SIDEKICK_SAY command! Not sending empty command.")
                return
        elif "LOAD_OBJECT" in command:
            msg.command = OpalCommand.LOAD_OBJECT
            # properties: JSON defining what object to load
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a "
                    + "LOAD_OBJECT command! Not sending empty command.")
                return
        elif "CLEAR" in command:
            msg.command = OpalCommand.CLEAR
            # properties: optionally, string defining what objects to
            # remove
            if properties:
                msg.properties = properties
        elif "MOVE_OBJECT" in command:
            msg.command = OpalCommand.MOVE_OBJECT
            # properties: JSON defining what object to move where
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a "
                    + "MOVE_OBJECT command! Not sending empty command.")
                return
        elif "HIGHLIGHT_OBJECT" in command:
            msg.command = OpalCommand.HIGHLIGHT_OBJECT
            # properties: a string with name of the object to highlight
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a "
                    + "HIGHLIGHT_OBJECT command! Not sending empty command.")
                return
        elif "REQUEST_KEYFRAME" in command:
            msg.command = OpalCommand.REQUEST_KEYFRAME
        elif "FADE_SCREEN" in command:
            msg.command = OpalCommand.FADE_SCREEN
        elif "UNFADE_SCREEN" in command:
            msg.command = OpalCommand.UNFADE_SCREEN
        elif "NEXT_PAGE" in command:
            msg.command = OpalCommand.NEXT_PAGE
        elif "PREV_PAGE" in command:
            msg.command = OpalCommand.PREV_PAGE
        elif "EXIT" in command:
            msg.command = OpalCommand.EXIT
        elif "SET_CORRECT" in command:
            msg.command = OpalCommand.SET_CORRECT
            # properties: JSON listing names of objects that are
            # correct or incorrect
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a "
                    + "SET_CORRECT command! Not sending empty command.")
                return
        elif "SHOW_CORRECT" in command:
            msg.command = OpalCommand.SHOW_CORRECT
        elif "HIDE_CORRECT" in command:
            msg.command = OpalCommand.HIDE_CORRECT
        elif "SETUP_STORY_SCENE" in command:
            msg.command = OpalCommand.SETUP_STORY_SCENE
            # properties: JSON listing scene attributes
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a "
                    + "SETUP_STORY_SCENE command! Not sending empty command.")
                return
        else:
            self.logger.warning("Not sending invalid OpalCommand: ", command)
            return
        # send message
        self.game_pub.publish(msg)
        self.logger.debug(msg)

    
    def send_opal_command_and_wait(self, command, response, timeout,
            properties=None):
        """ Publish opal command message and wait for a response """
        self.send_opal_command(command, properties)
        self.wait_for_response(response, timeout) 


    def send_robot_command(self, command, properties=None):
        """ Publish robot command message """
        self.logger.info("Sending robot command: " + str(command))
        # build message
        msg = RobotCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # add appropriate command
        if "SLEEP" in command:
            msg.command = RobotCommand.SLEEP 
        elif "WAKEUP" in command:
            msg.command = RobotCommand.WAKEUP
        elif "DO" in command:
            msg.command = RobotCommand.DO
            # DO commands take properties in the form of a string
            # contianing text to say and/or actions to do. We assume
            # these are provided in the correct string format.
            if properties:
                msg.properties = properties
            else:
                self.logger.warning("Did not get properties for a DO command! "
                        + "Not sending empty command.")
                return
        # send message
        self.robot_pub.publish(msg)
        rospy.loginfo(msg)


    def send_robot_command_and_wait(self, command, response, timeout,
            properties=None):
        """ Publish robot command message and wait for a response """
        # timeout should be a datetime.timedelta object
        self.send_robot_command(command, properties)
        self.wait_for_response(response, timeout) 


    def on_opal_action_msg(self, data):
        """ Called when we receive OpalAction messages """
        self.logger.info("Received OpalAction message: ACTION="
                + data.action + ", MESSAGE=" + data.message)

        # Currently, we are only using OpalAction messages to get
        # responses from the user. So we only care whether the action
        # was a PRESS and whether it was on an object that is used as
        # a YES or NO button or is a CORRECT or INCORRECT response 
        # object. When we do get one of these messages, set the 
        # relevant flag.
        if "tap" in data.action:
            # objectName, position
            pass
        elif "press" in data.action:
            # objectName, position
            # check if YES or NO was in the message
            if "YES" in data.message or "NO" in data.message:
                self.yes_no_response_received = True
                self.response_received = data.message
            # check if CORRECT was in the message
            if "CORRECT" in data.message:
                self.correct_incorrect_response_received = True
                self.response_received = data.message
            pass
        elif "release" in data.action:
            # no object
            pass
        elif "pancomplete" in data.action:
            # no object
            pass
        elif "pan" in data.action:
            # objectName, position
            pass
        elif "collideEnd" in data.action:
            # objectName, position, objectTwoName, positionTwo
            pass
        elif "collide" in data.action:
            # objectName, position, objectTwoName, positionTwo
            pass


    def on_robot_state_msg(self, data):
        """ Called when we receive RobotState messages """
        # When we get robot state messages, set a flag indicating
        # whether the robot is in motion or playing sound or not
        self.robot_speaking = data.is_playing_sound
        self.robot_doing_action = data.doing_action
        self.logger.info("Received RobotState message: doing_action="
                + str(data.doing_action) + ", playing_sound="
                + str(data.is_playing_sound))
        # TODO set flags for any other fields that we add to
        # RobotState messages later


    def wait_for_response(self, response, timeout):
        """ Wait for particular user or robot responses for the 
        specified amount of time.
        """
        # check what response to wait for, set that response received
        # flag to false
        # valid responses are:
        # YES_NO, CORRECT_INCORRECT, ROBOT_NOT_SPEAKING
        if "YES_NO" in response:
            self.yes_no_response_received = False
            self.waiting_for_yes_no = True
            self.waiting_for_correct_incorrect = False
            self.waiting_for_robot_speaking = False
        elif "CORRECT" in response:
            self.correct_incorrect_response_received = False
            self.waiting_for_yes_no = False
            self.waiting_for_correct_incorrect = True
            self.waiting_for_robot_speaking = False
        elif "ROBOT_NOT_SPEAKING" in response:
            self.robot_speaking = False
            self.waiting_for_yes_no = False
            self.waiting_for_correct_incorrect = False
            self.waiting_for_robot_speaking = True
        else:
            self.logger.warning("Told to wait for "
                    + response + " but that isn't one of the allowed "
                    + "responses to wait for!")
            return

        self.logger.info("waiting for " + response + "...")
        start_time = datetime.datetime.now()
        while datetime.datetime.now() - start_time < timeout:
            time.sleep(0.1)
            # check periodically whether we've received the response we
            # were waiting for, and if so, we're done waiting 
            if (self.waiting_for_yes_no and self.yes_no_response_received) \
                    or (self.waiting_for_correct_incorrect and \
                        self.correct_incorrect_response_received) \
                    or (self.waiting_for_robot_speaking \
                    and not self.robot_speaking):
                self.logger.info("Got response! "
                        + self.response_received)
                self.waiting_for_yes_no = False
                self.waiting_for_correct_incorrect = False
                self.waiting_for_robot_speaking = False
                return self.response_received
        # if we don't get the response we were waiting for, we're done
        # waiting and timed out
        self.waiting_for_yes_no = False
        self.waiting_for_correct_incorrect = False
        self.waiting_for_robot_speaking = False
        self.logger.info("Timed out! Moving on...")
        return "TIMEOUT"

