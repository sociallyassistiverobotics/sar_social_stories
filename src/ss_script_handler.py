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

import sys # For getting generic exception info
import datetime # For getting time deltas for timeouts
import time # For sleep
import json # For packing ros message properties
import random # For picking robot responses and shuffling answer options
import logging # Log messages
import Queue # for queuing messages for the main game loop
from SS_Errors import NoStoryFound # Custom exception when no stories found
from ss_script_parser import ss_script_parser # Parses scripts
from ss_personalization_manager import ss_personalization_manager
from ss_ros import ss_ros # Our ROS connection

class ss_script_handler():
    """ Social stories script handler parses and deals with script lines. Uses
    the script parser to get the next line in a script. We keep loading script
    lines and parsing script lines separate on the offchance that we might want
    to replace how scripts are stored and accessed (e.g., in a database versus
    in text files).
    """

    # Constants for script playback:
    # Time to pause after showing answer feedback and playing robot
    # feedback speech before moving on to the next question.
    ANSWER_FEEDBACK_PAUSE_TIME = 3
    # Time to wait for robot to finish speaking or acting before
    # moving on to the next script line (in seconds).
    WAIT_TIME = 30

    def __init__(self, ros_node, session, participant, script_path,
            story_script_path, session_script_path, database, queue):
        """ Save references to ROS connection and logger, get scripts and
        set up to read script lines
        """
        # Set up logger.
        self.logger = logging.getLogger(__name__)
        self.logger.info("Setting up script handler...")

        # Save reference to our ros node so we can publish messages.
        self.ros_node = ros_node

        # Save script paths so we can load scripts later.
        self.script_path = script_path

        if (story_script_path is None):
            self.story_script_path = ""
        else:
            self.story_script_path = story_script_path

        if (session_script_path is None):
            self.session_script_path = ""
        else:
            self.session_script_path = session_script_path

        # We get a reference to the main game node's queue so we can
        # give it messages.
        self.game_node_queue = queue

        # Set up personalization manager so we can get personalized
        # stories for this participant.
        self.personalization_manager = ss_personalization_manager(session,
                participant, database, percent_correct_to_level)

        # Set up counter for how many stories have been told this session.
        self.stories_told = 0

        # When we start, we are not currently telling a story or
        # repeating a script, or at the end of the game.
        self.doing_story = False
        self.repeating = False
        self.end_game = False

        # Set up script parser.
        self.script_parser = ss_script_parser()

        # Get session script from script parser and story scripts from
        # the personalization manager, and give to the script parser.
        try:
            self.script_parser.load_script(self.script_path
                    + self.session_script_path
                    + self.script_parser.get_session_script(session))
        except IOError:
            self.logger.exception("Script parser could not open session script!")
            # Pass exception up so whoever wanted a script handler knows
            # they didn't get a script.
            raise

        # Save start time so we can check whether we've run out of time.
        self.start_time = datetime.datetime.now()

        # Initialize total time paused.
        self.total_time_paused = datetime.timedelta(seconds=0)

        # Initialize pause start time in case someone calls the resume
        # game timer function before the pause game function.
        self.pause_start_time = None


    def iterate_once(self):
        """ Play the next commands from the script """
        try:
            # We check whether we've reached the game time limit when
            # we load new stories or when we are about to start a
            # repeating script over again.

            # Get next line from story script.
            if self.doing_story:
                self.logger.debug("Getting next line from story script.")
                line = self.story_parser.next_line()
            # If not in a story, get next line from repeating script.
            elif self.repeating:
                self.logger.debug("Getting next line from repeating script.")
                line = self.repeat_parser.next_line()
            # If not repeating, get next line from main session script.
            else:
                self.logger.debug("Getting next line from main session script.")
                line = self.script_parser.next_line()

        # We didn't read a line!
        # If we get a stop iteration exception, we're at the end of the
        # file and will stop iterating over lines.
        except StopIteration as e:
            # If we were doing a story, now we're done, go back to
            # the previous script.
            if self.doing_story:
                self.logger.info("Finished story " + str(self.stories_told + 1)
                        + " of " + str(self.max_stories) + "!")
                self.doing_story = False
                self.stories_told += 1
            # If we were repeating a script, increment counter.
            elif self.repeating:
                self.repetitions += 1
                self.logger.info("Finished repetition " + str(self.repetitions)
                    + " of " + str(self.max_repetitions) + "!")
                # If we've done enough repetitions, or if we've run out
                # of game time, go back to the main session script (set
                # the repeating flag to false).
                if (self.repetitions >= self.max_repetitions) \
                        or self.end_game \
                        or ((datetime.datetime.now() - self.start_time) \
                        - self.total_time_paused >= self.max_game_time):
                    self.logger.info("Done repeating!")
                    self.repeating = False
            # Otherwise we're at the end of the main script.
            else:
                self.logger.info("No more script lines to get!")
                # Pass on the stop iteration exception, with additional
                # information about the player's performance during the
                # game.
                e.performance = self.personalization_manager. \
                    get_emotion_performance_this_session()
                raise

        except ValueError:
            # We may get this exception if we try to get the next line
            # but the script file is closed. If that happens, something
            # probably went wrong with ending playback of a story script
            # or a repeating script. End repeating and end the current
            # story so we go back to the main session script.
            if self.doing_story:
                self.doing_story = False
            if self.repeating:
                self.repeating = False

        # Oh no got some unexpected error! Raise it again so we can
        # figure out what happened and deal with it during debugging.
        except Exception as e:
            self.logger.exception("Unexpected exception! Error: %s", e)
            raise

        # We got a line: parse it!
        else:
            # Make sure we got a line before we try parsing it. We
            # might not get a line if the file has closed or if
            # next_line has some other problem.
            if not line:
                self.logger.warning("[iterate_once] Tried to get next line, "
                    + "but got None!")
                return

            # Got a line - print for debugging.
            self.logger.debug("LINE: " + repr(line))

            # Parse line!
            # Split on tabs.
            elements = line.rstrip().split('\t')
            self.logger.debug("... " + str(len(elements)) + " elements: \n... "
                    + str(elements))

            if len(elements) < 1:
                self.logger.info("Line had no elements! Going to next line...")
                return

            # Do different stuff depending on what the first element is.
            #########################################################
            # only STORY lines have only one part to the command.
            elif len(elements) == 1:
                # For STORY lines, play back the next story for this
                # participant.
                if "STORY" in elements[0]:
                    self.logger.debug("STORY")
                    # If line indicates we need to start a story, do so.
                    self.doing_story = True
                    # Create a script parser for the filename provided,
                    # assume it is in the session_scripts directory.
                    self.story_parser = ss_script_parser()
                    try:
                        self.story_parser.load_script(self.script_path
                           + self.story_script_path
                           + self.personalization_manager.get_next_story_script())
                        self.personalization_manager.record_story_loaded()
                    except IOError:
                        self.logger.exception("Script parser could not open "
                                + "story script! Skipping STORY line.")
                        self.doing_story = False
                    except AttributeError:
                        self.logger.exception("Script parser could not open "
                                + "story script because no script was loaded! "
                                + "Skipping STORY line.")
                        self.doing_story = False
                    except NoStoryFound:
                        self.logger.exception("Script parser could not get \
                                the next story script because no script was \
                                found by the personalization manager! \
                                Skipping STORY line.")
                        self.doing_story = False

            # Line has 2+ elements, so check the other commands.
            #########################################################
            # For ROBOT lines, send command to the robot.
            elif "ROBOT" in elements[0]:
                self.logger.debug("ROBOT")
                # Play a randomly selected story intro from the list.
                if "STORY_INTRO" in elements[1]:
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.story_intros[
                            random.randint(0,len(self.story_intros)-1)])

                # Play a randomly selected story closing from the list.
                elif "STORY_CLOSING" in elements[1]:
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.story_closings[
                            random.randint(0,len(self.story_closings)-1)])

                # Send a command to the robot, with properties.
                elif len(elements) > 2:
                    self.ros_node.send_robot_command_and_wait(elements[1],
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=elements[2])

                # Send a command to the robot, without properties.
                else:
                    self.ros_node.send_robot_command(elements[1], "")

            #########################################################
            # For OPAL lines, send command to Opal game
            elif "OPAL" in elements[0]:
                self.logger.debug("OPAL")
                if "LOAD_ALL" in elements[1] and len(elements) >= 3:
                    # Load all objects listed in file -- the file is
                    # assumed to have properties for one object on each
                    # line.
                    to_load = self.read_list_from_file(
                            self.script_path + self.session_script_path +
                            elements[2])
                    for obj in to_load:
                        self.ros_node.send_opal_command("LOAD_OBJECT", obj)

                # Get the next story and load graphics into game.
                elif "LOAD_STORY" in elements[1]:
                    self.load_next_story()

                # Load answers for game.
                elif "LOAD_ANSWERS" in elements[1] and len(elements) >= 3:
                    self.load_answers(elements[2])

                # Send an opal command, with properties.
                elif len(elements) > 2:
                    self.ros_node.send_opal_command(elements[1], elements[2])

                # Send an opal command, without properties.
                else:
                    self.ros_node.send_opal_command(elements[1])

            #########################################################
            # For PAUSE lines, sleep for the specified number of
            # seconds before continuing script playback.
            elif "PAUSE" in elements[0] and len(elements) >= 2:
                self.logger.debug("PAUSE")
                try:
                    time.sleep(int(elements[1]))
                except ValueError:
                    self.logger.exception("Not pausing! PAUSE command was "
                        + "given an invalid argument (should be an int)!")

            #########################################################
            # For ADD lines, get a list of robot commands that can be
            # used in response to particular triggers from the specified
            # file and save them for later use -- all ADD lines should
            # have 3 elements.
            elif "ADD" in elements[0] and len(elements) >= 3:
                self.logger.debug("ADD")
                # Read list of responses from the specified file into the
                # appropriate variable.
                try:
                    if "INCORRECT_RESPONSES" in elements[1]:
                        self.incorrect_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.incorrect_responses)))
                    if "CORRECT_RESPONSES" in elements[1]:
                        self.correct_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.correct_responses)))

                    elif "START_RESPONSES" in elements[1]:
                        self.start_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.start_responses)))
                    elif "NO_RESPONSES" in elements[1]:
                        self.no_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.no_responses)))
                    elif "ANSWER_FEEDBACK" in elements[1]:
                        self.answer_feedback = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.answer_feedback)))
                    elif "STORY_INTROS" in elements[1]:
                        self.story_intros = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.story_intros)))
                    elif "STORY_CLOSINGS" in elements[1]:
                        self.story_closings = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.story_closings)))
                    elif "TIMEOUT_CLOSINGS" in elements[1]:
                        self.timeout_closings = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("Got "
                                + str(len(self.timeout_closings)))
                    elif "MAX_STORIES_REACHED" in elements[1]:
                        self.max_stories_reached = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.debug("... Got "
                                + str(len(self.max_stories_reached)))
                except IOError:
                    self.logger.exception("Failed to add responses!")
                else:
                    self.logger.info("Added " + elements[1])

            #########################################################
            # For SET lines, set the specified constant.
            elif "SET" in elements[0] and len(elements) >= 3:
                self.logger.debug("SET")
                if "MAX_INCORRECT_RESPONSES" in elements[1]:
                    self.max_incorrect_responses = int(elements[2])
                    self.logger.info("Set MAX_INCORRECT_RESPONSES to " +
                            elements[2])
                elif "MAX_GAME_TIME" in elements[1]:
                    self.max_game_time = datetime.timedelta(minutes=
                            int(elements[2]))
                    self.logger.info("Set MAX_GAME_TIME to " + elements[2])
                elif "MAX_STORIES" in elements[1]:
                    self.max_stories = int(elements[2])
                    self.logger.info("Set MAX_STORIES to " + elements[2])

            #########################################################
            # For WAIT lines, wait for the specified user response,
            # or for a timeout.
            elif "WAIT" in elements[0] and len(elements) >= 3:
                self.logger.debug("WAIT")
                self.wait_for_response(elements[1], int(elements[2]))

            #########################################################
            # For REPEAT lines, repeat lines in the specified script
            # file the specified number of times.
            elif "REPEAT" in elements[0] and len(elements) >= 3:
                self.logger.debug("REPEAT")
                self.repeating = True
                self.repetitions = 0
                # Create a script parser for the filename provided,
                # assume it is in the session_scripts directory.
                self.repeat_parser = ss_script_parser()
                try:
                    self.repeat_parser.load_script(self.script_path
                            + self.session_script_path
                            + elements[2])
                except IOError:
                    self.logger.exception("Script parser could not open "
                        + "session script to repeat! Skipping REPEAT line.")
                    self.repeating = False
                    return

                # Figure out how many times we should repeat the script.
                if "MAX_STORIES" in elements[1]:
                    try:
                        self.max_repetitions = self.max_stories
                    except AttributeError:
                        self.logger.exception("Tried to set MAX_REPETITIONS to"
                                + " MAX_STORIES, but MAX_STORIES has not been "
                                + "set . Setting to 1 repetition instead.")
                        self.max_repetitions = 1
                else:
                    self.max_repetitions = int(elements[1])
                self.logger.debug("Going to repeat " + elements[2] + " " +
                        str(self.max_repetitions) + " time(s).")


    def read_list_from_file(self, filename):
        ''' Read a list of robot responses from a file, return a list
        of the lines from the file
        '''
        # Open script for reading.
        try:
            fh = open(filename, "r")
            return fh.readlines()
        except IOError as e:
            self.logger.exception("Cannot open file: " + filename)
            # Pass exception up so anyone trying to add a response list
            # from a script knows it didn't work.
            raise


    def wait_for_response(self, response_to_get, timeout):
        """ Wait for a user response or wait until the specified time
        has elapsed. If the response is incorrect, allow multiple
        attempts up to the maximum number of incorrect responses.
        """
        for i in range(0, self.max_incorrect_responses):
            self.logger.info("Waiting for user response...")
             # Save the response we were trying to get in case we need
             # to try again.
            self.last_response_to_get = response_to_get
            self.last_response_timeout = timeout
            # Wait for the specified type of response, or until the
            # specified time has elapsed.
            response = self.ros_node.wait_for_response(response_to_get,
                    datetime.timedelta(seconds=int(timeout)))

            # After waiting for a response, need to play back an
            # appropriate robot response.

            # If we didn't receive a response, then it was probably
            # because we didn't send a valid response to wait for.
            # This is different from a TIMEOUT since we didn't time
            # out -- we just didn't get a response of any kind.
            if not response:
                self.logger.info("Done waiting -- did not get valid response!")
                return False

            # If we received no user response before timing out, send a
            # TIMEOUT message and pause the game.
            elif "TIMEOUT" in response:
                # Announce we timed out.
                self.ros_node.send_game_state("TIMEOUT")
                # Pause game and wait to be told whether we should try
                # waiting again for a response or whether we should
                # skip it and move on. Queue up the pause command so the
                # main game loop can take action.
                self.game_node_queue.put("PAUSE")
                # Announce the game is pausing.
                self.ros_node.send_game_state("PAUSE")
                # Indicate that we did not get a response.
                # We don't break and let the user try again because the
                # external game monitor deals with TIMEOUT events, and
                # will tell us whether to try waiting again or to just
                # skip waiting for this response.
                return False

            # If response was INCORRECT, randomly select a robot
            # response to an incorrect user action.
            elif "INCORRECT" in response:
                try:
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.incorrect_responses[random.randint(0,
                            len(self.incorrect_responses)-1)])
                except AttributeError:
                    self.logger.exception("Could not play an incorrect "
                            + "response because none were loaded!")
                # Don't break so we allow the user a chance to respond
                # again.

            # If response was NO, randomly select a robot response to
            # the user selecting no.
            elif "NO" in response:
                try:
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.no_responses[random.randint(0,
                            len(self.no_responses)-1)])
                except AttributeError:
                    self.logger.exception("Could not play a response to "
                            + "user's NO because none were loaded!")
                # Don't break so we allow the user a chance to respond
                # again.

            # If response was CORRECT, randomly select a robot response
            # to a correct user action, highlight the correct answer,
            # and break out of response loop.
            elif "CORRECT" in response:
                try:
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.correct_responses[random.randint(0,
                            len(self.correct_responses)-1)])
                    self.ros_node.send_opal_command("SHOW_CORRECT")
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.answer_feedback[random.randint(0,
                            len(self.answer_feedback)-1)])
                # Pause after speaking before hiding correct again
                    time.sleep(self.ANSWER_FEEDBACK_PAUSE_TIME)
                    self.ros_node.send_opal_command("HIDE_CORRECT")
                except AttributeError:
                    self.logger.exception("Could not play a correct "
                            + "response or could not play robot's answer"
                            + "feedback because none were loaded!")
                # Break from the for loop so we don't give the user
                # a chance to respond again.
                break

            # If response was START, randomly select a robot response to
            # the user selecting START, and break out of response loop.
            elif "START" in response:
                    try:
                        self.ros_node.send_robot_command_and_wait("DO",
                            response="ROBOT_NOT_SPEAKING",
                            timeout=datetime.timedelta(seconds=int(
                                self.WAIT_TIME)),
                            properties=self.start_responses[random.randint(0,
                                len(self.start_responses)-1)])
                    except AttributeError:
                        self.logger.exception("Could not play response to"
                            + "user's START because none were loaded!")
                    # Break from the for loop so we don't give the user
                    # a chance to respond again.
                    break

        # We exhausted our allowed number of user responses, so have
        # the robot do something instead of waiting more.
        else:
            # If user was never correct, play robot's correct answer
            # feedback and show which answer was correct in the game.
            if "CORRECT" in response_to_get:
                try:
                    self.ros_node.send_opal_command("SHOW_CORRECT")
                    self.ros_node.send_robot_command_and_wait("DO",
                        response="ROBOT_NOT_SPEAKING",
                        timeout=datetime.timedelta(seconds=int(
                            self.WAIT_TIME)),
                        properties=self.answer_feedback[random.randint(0,
                            len(self.answer_feedback)-1)])
                    # Pause after speaking before hiding correct again.
                    time.sleep(self.ANSWER_FEEDBACK_PAUSE_TIME)
                    self.ros_node.send_opal_command("HIDE_CORRECT")
                except AttributeError:
                    self.logger.exception("Could not play robot's answer"
                            + " feedback because none were loaded!")

            # If user never selects START (which is used to ask the user
            # if they are ready to play), stop all stories and repeating
            # scripts, continue with main script so we go to the end.
            elif "START" in response_to_get:
                self.repeating = False
                self.story = False

        # We got a user response and responded to it!
        return True


    def skip_wait_for_response(self):
        """ Skip waiting for a response; treat the skipped response as
        a NO or INCORRECT response.
        """
        # If the response to wait for was CORRECT or INCORRECT,
        # randomly select a robot response to an incorrect user
        # action.
        if "CORRECT" in self.last_response_to_get:
            try:
                self.ros_node.send_robot_command_and_wait("DO",
                    response="ROBOT_NOT_SPEAKING",
                    timeout=datetime.timedelta(seconds=int(self.WAIT_TIME)),
                    properties=self.incorrect_responses[random.randint(0, \
                        len(self.incorrect_responses)-1)])
            except AttributeError:
                self.logger.exception("Could not play an incorrect "
                        + "response because none were loaded!")

        # If response to wait for was YES or NO, randomly select a
        # robot response for a NO user action.
        elif "NO" in self.last_response_to_get:
            try:
                self.ros_node.send_robot_command_and_wait("DO",
                    response="ROBOT_NOT_SPEAKING",
                    timeout=datetime.timedelta(seconds=int(self.WAIT_TIME)),
                    properties=self.no_responses[random.randint(0,
                        len(self.no_responses)-1)])
            except AttributeError:
                self.logger.exception("Could not play a response to "
                        + "user's NO because none were loaded!")


    def set_end_game(self):
        ''' End the game gracefully -- stop any stories or repeating
        scripts, go back to main session script and finish.
        '''
        # For now, we just need to set a flag indicating we should end
        # the game. When we check whether we should load another story
        # or repeat a repeating script, this flag will be used to skip
        # back to the main session script, to the end of the game.
        self.end_game = True


    def set_start_level(self, level):
        """ When the game starts, a level to start at can be provided.
        Pass this to the personalization manager to deal with, since it
        deals with picking the levels of stories to play.
        """
        self.personalization_manager.set_start_level(level)


    def pause_game_timer(self):
        ''' Track how much time we spend paused so when we check
        whether we have reached the max game time, we don't include
        time spent paused.
        '''
        self.pause_start_time = datetime.datetime.now()


    def resume_game_timer(self):
        ''' Add how much time we spent paused to our total time spent
        paused.
        '''
        # Since this function could theoretically be called before we
        # get a call to pause_game_timer, we have to check that there
        # is a pause start time, and then later, reset it so we can't
        # add the same pause length multiple times to our total pause
        # time.
        if self.pause_start_time is not None:
            self.total_time_paused += datetime.datetime.now() \
               - self.pause_start_time
        # Reset pause start time.
        self.pause_start_time = None

    def wait_for_last_response_again(self):
        """ Wait for the same response that we just waited for again,
        with the same parameters for the response and the timeout.
        """
        return self.wait_for_response(self.last_response_to_get,
            last_response_timeout)


    def load_answers(self, answer_list):
        ''' Load the answer graphics for this story '''
        # We are given a list of words that indicate what the answer
        # options are. By convention, the first word is probably the
        # correct answer; the others are incorrect answers. However,
        # we won't set this now because this convention may not hold.
        # We expect the SET_CORRECT OpalCommand to be used to set
        # which answers are correct or incorrect.
        # split the list of answers on commas.
        answers = answer_list.strip().split(',')

        # Shuffle answers to display them in a random order.
        random.shuffle(answers)

        # Load in the graphic for each answer.
        for answer in answers:
            toload = {}
            # Remove whitespace from name before using it.
            toload["name"] = answer.strip()
            toload["tag"] = "PlayObject"
            toload["slot"] = answers.index(answer) + 1
            toload["draggable"] = False
            toload["isAnswerSlot"] = True
            self.ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))


    def load_next_story(self):
        ''' Get the next story, set up the game scene with scene and
        answer slots, and load scene graphics.
        '''
        # If we've told the max number of stories, or if we've reached
        # the max game time, don't load another story even though we
        # were told to load one -- instead, play error message from
        # robot saying we have to be done now.
        if self.stories_told >= self.max_stories \
            or ((datetime.datetime.now() - self.start_time) \
            - self.total_time_paused >= self.max_game_time) or self.end_game:
            self.logger.info("We were told to load another story, but we've "
                    + "already played the maximum number of stories or we ran"
                    " out of time! Skipping and ending now.")
            self.doing_story = False
            try:
                self.ros_node.send_robot_command_and_wait("DO",
                    response="ROBOT_NOT_SPEAKING",
                    timeout=datetime.timedelta(seconds=int(self.WAIT_TIME)),
                    properties=self.max_stories_reached
                    [random.randint(0, len(self.no_responses)-1)])
            except AttributeError:
                self.logger.exception("Could not play a max stories reached "
                        + "response because none were loaded!")
            # We were either told to play another story because a
            # repeating script loads a story and the max number of
            # repetitions is greater than the max number of stories,
            # so more stories were requested than can be played, or
            # because we ran out of time and were supposed to play more
            # stories than we have time for. Either way, stop the
            # repeating script if there is one.
            self.repeating = False
            return

        # Get the details for the next story.
        try:
            scenes, in_order, num_answers = \
                self.personalization_manager.get_next_story_details()
        except NoStoryFound:
            # If no story was found, we can't load the story!
            self.logger.exception("Cannot load story - no story to load was \
                    found!")
            self.doing_story = False
            return

        # Set up the story scene in the game.
        setup = {}
        setup["numScenes"] = len(scenes)
        setup["scenesInOrder"] = in_order
        setup["numAnswers"] = num_answers
        self.ros_node.send_opal_command("SETUP_STORY_SCENE", json.dumps(setup))

        # Load the scene graphics.
        for scene in scenes:
            toload = {}
            toload["name"] = scene
            toload["tag"] = "PlayObject"
            toload["slot"] = scenes.index(scene) + 1
            if not in_order:
                toload["correctSlot"] = scenes.index(scene) + 1
            toload["draggable"] = False if in_order else True
            toload["isAnswerSlot"] = False
            self.ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))

        # Tell the personalization manager that we loaded the story so
        # it can keep track of which stories have been played.
        self.personalization_manager.record_story_loaded()
