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
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
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
import Queue # for queuing messages for the main game loop
from sar_opal_msgs.msg import OpalCommand # ROS msgs to talk to game
from sar_opal_msgs.msg import OpalAction # ROS msgs for game actions
from sar_robot_command_msgs.msg import RobotCommand # ROS msgs for robot cmd
from sar_robot_command_msgs.msg import RobotState # ROS msgs for robot state
from std_msgs.msg import Header # standard ROS msg header
from sar_game_command_msgs.msg import GameState # ROS msgs for game state
from sar_game_command_msgs.msg import GameCommand # ROS msgs for game commands

class ss_ros():
    """ ROS node: set up rostopics we publish, subscribe to rostopics
    we care about, functions to send and receive messages.
    """
    # Set up rostopics we publish: commands to the game (on a tablet or on a
    # PC/touchscreen), commands to the robot, and game state messages.
    _game_pub = rospy.Publisher('/sar/opal_command', OpalCommand,
            queue_size = 10)
    _robot_pub = rospy.Publisher('/sar/robot_command', RobotCommand,
            queue_size = 10)
    _state_pub = rospy.Publisher('/sar/game_state', GameState, queue_size = 10)


    def __init__(self, queue):
        """ Initialize ROS """
        # We get a reference to the main game node's queue so we can
        # give it messages.
        self._game_node_queue = queue

        # Set up logger
        self._logger = logging.getLogger(__name__)
        self._logger.info("Subscribing to topics: /sar/opal_action, " +
            "/sar/robot_state, /sar/game_command")

        # Initialize the flags we use to track responses from the robot
        # and from the user.
        self._robot_doing_action = False
        self._robot_speaking = False
        self._response_received = None

        # Subscribe to messages from opal game.
        rospy.Subscriber('/sar/opal_action', OpalAction,
                self.on_opal_action_msg)
        # Subscribe to messages about the robot's state.
        rospy.Subscriber('/sar/robot_state', RobotState, self.on_robot_state_msg)
        # Subscribe to game commands (commands we are sent to start, pause,
        # and stop the game).
        rospy.Subscriber('/sar/game_command', GameCommand,
                self.on_game_command_msg)


    def send_opal_command(self, command, properties=None, response=None,
            timeout=None):
        """ Publish opal command message. Optionally, wait for a
        response.
        """
        self._logger.info("Sending opal command: " + command)
        # Build message.
        msg = OpalCommand()
        # Add header.
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
            # Properties: a string with the name of action to do.
        elif "SIDEKICK_SAY" in command:
            msg.command = OpalCommand.SIDEKICK_SAY
            # Properties: a string with the name of audio file to play.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a "
                    + "SIDEKICK_SAY command! Not sending empty command.")
                return
        elif "LOAD_OBJECT" in command:
            msg.command = OpalCommand.LOAD_OBJECT
            # Properties: JSON defining what object to load.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a "
                    + "LOAD_OBJECT command! Not sending empty command.")
                return
        elif "CLEAR" in command:
            msg.command = OpalCommand.CLEAR
            # Properties: optionally, string defining what objects to
            # remove.
            if properties:
                msg.properties = properties
        elif "MOVE_OBJECT" in command:
            msg.command = OpalCommand.MOVE_OBJECT
            # Properties: JSON defining what object to move where.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a "
                    + "MOVE_OBJECT command! Not sending empty command.")
                return
        elif "HIGHLIGHT" in command:
            msg.command = OpalCommand.HIGHLIGHT_OBJECT
            # Properties: a string with name of the object to highlight.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a "
                    + "HIGHLIGHT_OBJECT command! Adding null properties.")
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
            # Properties: JSON listing names of objects that are
            # correct or incorrect.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a "
                    + "SET_CORRECT command! Not sending empty command.")
                return
        elif "SHOW_CORRECT" in command:
            msg.command = OpalCommand.SHOW_CORRECT
        elif "HIDE_CORRECT" in command:
            msg.command = OpalCommand.HIDE_CORRECT
        elif "SETUP_STORY_SCENE" in command:
            msg.command = OpalCommand.SETUP_STORY_SCENE
            # Properties: JSON listing scene attributes.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a "
                    + "SETUP_STORY_SCENE command! Not sending empty command.")
                return
        else:
            self._logger.warning("Not sending invalid OpalCommand: ", command)
            return
        # Send message.
        self._game_pub.publish(msg)
        self._logger.debug(msg)

        # If we got a response to wait for and a timeout value, wait
        # for a response.
        if response and timeout:
            self.wait_for_response(response, timeout)


    def send_robot_command(self, command, properties=None, response=None,
            timeout=None):
        """ Publish robot command message and optionally wait for a
        response.
        """
        self._logger.info("Sending robot command: " + str(command))
        # Build message.
        msg = RobotCommand()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Add appropriate command.
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
                self._logger.warning("Did not get properties for a DO command! "
                        + "Not sending empty command.")
                return
        msg.interrupt = False
        # Send message.
        self._robot_pub.publish(msg)
        self._logger.debug(msg)

        # If we got a response to wait for and a timeout value, wait
        # for a response.
        # Timeout should be a datetime.timedelta object.
        if response and timeout:
            self.wait_for_response(response, timeout)


    def send_game_state(self, state, performance=None):
        """ Publish a game state message. """
        self._logger.info("Sending game state: " + str(state))
        # Build message.
        msg = GameState()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Add constant indicating which game we are.
        msg.game = GameState.STORYTELLING
        # Add appropriate state.
        if "START" in state:
            msg.state = GameState.START
        if "IN_PROGRESS" in state:
            msg.state = GameState.IN_PROGRESS
        if "PAUSED" in state:
            msg.state = GameState.PAUSED
        if "TIMEOUT" in state:
            msg.state = GameState.USER_TIMEOUT
        if "READY" in state:
            msg.state = GameState.READY
        if "END" in state:
            msg.state = GameState.END
            if performance is not None:
                msg.performance = performance
            else:
                self._logger.warning("Expected to send performance "
                    + "metric with GameState.END message, but did not "
                    + "receive a performance metric!")
        # Send message.
        self._state_pub.publish(msg)
        self._logger.debug(msg)


    def on_game_command_msg(self, data):
        """ Called when we receive GameCommand messages """
        self._logger.info("Received GameCommand message: GAME=" +
                str(data.game) + ", COMMAND=" + str(data.command))
        # If the game field doesn't list the constant referring to this
        # game, we can ignore the message.
        if GameCommand.STORYTELLING is not data.game:
            self._logger.info("Not for us... ignoring.")
            return

        # We need to act based on the game commands - queue up the
        # command received so the main game loop can take action.
        if data.command is GameCommand.START:
            # START commands may include the "level" field that can
            # specify the level the game should start at. Pass this on
            # if it exists.
            self._game_node_queue.put("START" +
                ("\t" + str(data.level) if data.level else ""))
        elif data.command is GameCommand.PAUSE:
            self._game_node_queue.put("PAUSE")
        elif data.command is GameCommand.CONTINUE:
            self._game_node_queue.put("CONTINUE")
        elif data.command is GameCommand.END:
            self._game_node_queue.put("END")
        elif data.command is GameCommand.WAIT_FOR_RESPONSE:
            self._game_node_queue.put("WAIT_FOR_RESPONSE")
        elif data.command is GameCommand.SKIP_RESPONSE:
            self._game_node_queue.put("SKIP_RESPONSE")


    def on_opal_action_msg(self, data):
        """ Called when we receive OpalAction messages """
        self._logger.info("Received OpalAction message: ACTION="
                + data.action + ", MESSAGE=" + data.message)

        # Currently, we are only using OpalAction messages to get
        # responses from the user. So we only care whether the action
        # was a PRESS and whether it was on an object that is used as
        # a START button or is a CORRECT or INCORRECT response
        # object. When we do get one of these messages, set the
        # relevant flag.
        if "tap" in data.action:
            # objectName, position
            pass
        elif "press" in data.action:
            # objectName, position
            # Check if START was in the message.
            if "START" in data.message:
                self.start_response_received = True
                self._response_received = data.message
            # Check if CORRECT was in the message.
            if "CORRECT" in data.message:
                self._correct_incorrect_response_received = True
                self._response_received = data.message
            pass
        elif "release" in data.action:
            # No object
            pass
        elif "pancomplete" in data.action:
            # No object
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
        # whether the robot is in motion or playing sound or not.
        self._robot_speaking = data.is_playing_sound
        self._robot_doing_action = data.doing_action
        self._logger.info("Received RobotState message: doing_action="
                + str(data.doing_action) + ", playing_sound="
                + str(data.is_playing_sound))
        # TODO Set flags for any other fields that we add to
        # RobotState messages later.


    def wait_for_response(self, response, timeout):
        """ Wait for particular user or robot responses for the
        specified amount of time.
        """
        # Check what response to wait for, set that response received
        # flag to false.
        # Valid responses to wait for are:
        # START, CORRECT_INCORRECT, ROBOT_NOT_SPEAKING
        if "START" in response:
            self.start_response_received = False
            self._waiting_for_start = True
            self._waiting_for_correct_incorrect = False
            self._waiting_for_robot_speaking = False
        elif "CORRECT" in response:
            self._correct_incorrect_response_received = False
            self._waiting_for_start = False
            self._waiting_for_correct_incorrect = True
            self._waiting_for_robot_speaking = False
        elif "ROBOT_NOT_SPEAKING" in response:
            self._robot_speaking = True
            self._waiting_for_start = False
            self._waiting_for_correct_incorrect = False
            self._waiting_for_robot_speaking = True
        else:
            self._logger.warning("Told to wait for " + response + " but that " +
                "isn't one of the allowed responses to wait for!")
            return

        self._logger.info("waiting for " + response + "...")
        start_time = datetime.datetime.now()
        while datetime.datetime.now() - start_time < timeout:
            time.sleep(0.1)
            # Check periodically whether we've received the response we
            # were waiting for, and if so, we're done waiting.
            if (self._waiting_for_start and self.start_response_received) \
                    or (self._waiting_for_correct_incorrect and \
                        self._correct_incorrect_response_received) \
                    or (self._waiting_for_robot_speaking \
                    and not self._robot_speaking \
                    and not self._robot_doing_action):
                self._logger.info("Got " + response + " response!")
                # Reset waiting flags
                self._waiting_for_start = False
                self._waiting_for_correct_incorrect = False
                self._waiting_for_robot_speaking = False
                return self._response_received
        # If we don't get the response we were waiting for, we're done
        # waiting and timed out.
        self._waiting_for_start = False
        self._waiting_for_correct_incorrect = False
        self._waiting_for_robot_speaking = False
        self._logger.info("Timed out! Moving on...")
        return "TIMEOUT"

