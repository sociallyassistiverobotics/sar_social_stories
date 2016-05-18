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

import sys # for getting generic exception info
import datetime # for getting time deltas for timeouts
import json
from ss_script_parser import ss_script_parser
from ss_personalization_manager import ss_personalization_manager
from ss_ros import ss_ros
from random import randint

class ss_script_handler():
    """ Social stories script handler parses and deals with script lines. Uses 
    the script parser to get the next line in a script. We keep loading script
    lines and parsing script lines separate on the offchance that we might want
    to replace how scripts are stored and accessed (e.g., in a database versus 
    in text files). 
    """

    def __init__(self, logger, ros_node, session, participant):
        """ Save references to ROS connection and logger, get scripts and
        set up to read script lines 
        """

        # save reference to logger for logging stuff later
        self.logger = logger
        self.logger.log("Setting up script handler...")

        # save reference to our ros node so we can publish messages
        self.ros_node = ros_node

        # set up personalization manager so we can get personalized stories
        # for this participant
        self.personalization_manager = ss_personalization_manager(self.logger,
                session, participant)

        # set up script parser
        self.script_parser = ss_script_parser(self.logger)

        # get session script from script parser and story scripts from the
        # personalization manager, and give to the script parser
        try:
            self.script_parser.load_script(self.script_parser.get_session_script(
                session))
        except IOError:
            self.logger.log("Script parser could not open session script!")
            # pass exception up so whoever wanted a script handler knows they
            # didn't get a script
            raise


    def iterate_once(self):
        """ Play the next commands from the script """
        try:
            # TODO check whether we've reached the time limit
            # TODO check whether we've reached the max stories to tell
            # TODO check whether we've reached the max incorrect responses

            # get next line from repeating script
            if self.repeating:
                line = self.repeat_parser.next_line()
            # if not repeating, get next line from main session script
            else:
                line = self.script_parser.next_line()


            # got a line - print for debugging
            print("LINE: " + repr(line))
            # TODO switch prints to logger.logs if we want to log them

            # parse line!
            # split on tabs
            elements = line.split("\t")
            if len(elements) < 1:
                print("Line had no elements! Going to next line...")
                return

            # do different stuff depending on what the first element is
            #########################################################
            # only STORY lines have only one part to the command
            elif len(elements) == 1:
                # for STORY lines, play back the next story for this
                # participant
                if "STORY" in elements[0]:
                    print("STORY")
                    # TODO if line indicates we need to start a story, do so 
                    # use self.script_file
                    # open in new script parser like REPEAT lines

            # line has 2+ elements, so check the other commands
            #########################################################
            # for ROBOT lines, send command to the robot
            elif "ROBOT" in elements[0]:
                print("ROBOT")
                # play a randomly selected story intro from the list
                if "STORY_INTRO" in elements[1]:
                    self.ros_node.send_robot_command("DO", self.story_intros[
                        randint(0,len(self.story_intros)-1)])

                # play a randomly selected story closing from the list
                elif "STORY_CLOSING" in elements[1]:
                    print("TODO play a story closing")
                    self.ros_node.send_robot_command("DO", self.story_closings[
                        randint(0,len(self.story_closings)-1)])
                
                # send a command to the robot, with properties
                elif len(elements) > 2:
                    self.ros_node.send_robot_command(elements[1], elements[2])

                # send a command to the robot, without properties
                else:
                    self.ros_node.send_robot_command(elements[1], "")

            #########################################################
            # for OPAL lines, send command to Opal game
            elif "OPAL" in elements[0]:
                print("OPAL")
                if "LOAD_ALL" in elements[1] and len(elements) >= 3:
                    # load all objects listed in file -- the file is 
                    # assumed to have properties for one object on each 
                    # line
                    to_load = read_list_from_file(elements[2])
                    for obj in to_load:
                        self.ros_node.send_opal_command("LOAD_OBJECT", obj)

                elif "LOAD_STORY" in elements[1]:
                    # get the next story
                    self.script_file, scenes, in_order, num_answers = \
                        self.personalization_manager.get_next_story()

                    # set up the story scene in the game
                    setup = {}
                    setup["numScenes"] = len(scenes)
                    setup["scenesInOrder"] = in_order
                    setup["numAnswers"] = num_answers
                    self.ros_node.send_opal_command("SETUP_STORY_SCENE",
                           json.dumps(setup))

                    # load the scene graphics
                    for scene in scenes:
                        toload = {}
                        toload["name"] = scene
                        toload["tag"] = "PlayObject"
                        toload["slot"] = scenes.index(scene) + 1
                        if not in_order:
                            toload["correctSlot"] = scenes.index(scene) + 1
                        toload["draggable"] = False if in_order else True
                        toload["isAnswerSlot"] = False
                        self.ros_node.send_opal_command("LOAD_OBJECT", toload)

                # send an opal command, with properties
                elif len(elements) > 2:
                    self.ros_node.send_opal_command(elements[1], elements[2])

                # send an opal command, without properties
                else:
                    self.ros_node.send_opal_command(elements[1])
            
            #########################################################
            # for ADD lines, get a list of robot commands that can be 
            # used in response to particular triggers from the specified
            # file and save them for later use -- all ADD lines should 
            # have 3 elements
            elif "ADD" in elements[0] and len(elements) >= 3:
                # read list of responses from the specified file into the 
                # appropriate variable
                try:
                    if "CORRECT_RESPONSES" in elements[1]:
                        self.correct_responses = self.read_list_from_file(
                                elements[2])
                    elif "INCORRECT_RESPONSES" in elements[1]:
                        self.incorrect_responses = self.read_list_from_file(
                                elements[2])
                    elif "YES_RESPONSES" in elements[1]:
                        self.yes_responses = self.read_list_from_file(
                                elements[2])
                    elif "NO_RESPONSES" in elements[1]:
                        self.no_responses = self.read_list_from_file(
                                elements[2])
                    elif "STORY_INTROS" in elements[1]:
                        self.story_intros = self.read_list_from_file(
                                elements[2])
                    elif "STORY_CLOSINGS" in elements[1]:
                        self.story_closings = self.read_list_from_file(
                                elements[2])
                    elif "TIMEOUT_CLOSINGS" in elements[1]:
                        self.timeout_closings = self.read_list_from_file(
                                elements[2])
                except IOError:
                    self.logger.log("Failed to add responses!")

                else:
                    self.logger.log("Added " + elements[1])

            #########################################################
            # for SET lines, set the specified constant
            elif "SET" in elements[0] and len(elements) >= 3:
                if "MAX_INCORRECT_RESPONSES" in elements[1]:
                    self.max_incorrect_responses = elements[2]
                    self.logger.log("Set MAX_INCORRECT_RESPONSES to " + 
                            elements[2])
                elif "MAX_GAME_TIME" in elements[1]:
                    self.max_game_time = datetime.timedelta(seconds=elements[2])
                    self.logger.log("Set MAX_GAME_TIME to " + elements[2])
                elif "MAX_STORIES" in elements[1]:
                    self.max_stories = elements[2]
                    self.logger.log("Set MAX_STORIES to " + elements[2])

            #########################################################
            # for WAIT lines, wait for the specified user response, or timeout
            # if no response is received
            elif "WAIT" in elements[0] and len(elements) >= 3:
                # wait for a user response or wait until the specified time has
                # elapsed. If the response is incorrect, allow multiple 
                # attempts up to the maximum number of incorrect responses
                for i in range(0, self.max_incorrect_responses):
                    self.logger.log("Waiting for user response...")
                    # wait for the specified type of response, or until the 
                    # specified time has elapsed
                    response = self.ros_node.wait_for_response(elements[1],
                            datetime.timedelta(seconds=elements[2]))
                    
                    # after waiting for a response, need to play back an
                    # appropriate robot response 
                    # if response was CORRECT, randomly select a robot response
                    # to a correct user action, and break out of response loop
                    if "CORRECT" in response:
                        self.ros_node.send_robot_command("DO", 
                                self.correct_responses[randint(0,
                                    len(self.correct_responses)-1)])
                        break

                    # if response was YES, randomly select a robot response to
                    # the user selecting yes, and break out of response loop 
                    elif "YES" in response:
                        self.ros_node.send_robot_command("DO", 
                                self.yes_responses[randint(0,
                                    len(self.yes_responses)-1)])
                        break

                    # if we received no user response before timing out, 
                    # treat as either NO or INCORRECT

                    # if response was INCORRECT, randomly select a robot 
                    # response to an incorrect user action
                    elif ("INCORRECT" in response) or ("TIMEOUT" in response 
                            and "CORRECT" in elements[1]):
                        self.ros_node.send_robot_command("DO", 
                                self.incorrect_responses[randint(0,
                                    len(self.incorrect_responses)-1)])

                    # if response was NO, randomly select a robot response to
                    # the user selecting no 
                    elif "NO" in response or ("TIMEOUT" in response 
                            and "YES_NO" in elements[1]):
                        self.ros_node.send_robot_command("DO", 
                                self.no_responses[randint(0,
                                    len(self.no_responses)-1)])

                # we exhausted our allowed number of user responses, so have
                # the robot do something
                else:
                    # TODO if user was never correct, play robot's correct 
                    # response feedback? if user never selected yes, ??
                    pass

            #########################################################
            # for REPEAT lines, repeat lines in the specified script file the 
            # specified number of times
            elif "REPEAT" in elements[0] and len(elements) >= 3:
                self.repeating = True
                self.repetitions = 0
                # create a script parser for the filename provided, assume it
                # is in the session_scripts directory 
                self.repeat_parser = ss_script_parser(self.logger)
                try:
                    self.repeat_parser.load_script("../session_scripts/" +
                        elements[2])
                except IOError:
                    self.logger.log("Script parser could not open script to "
                            + "repeat! Skipping REPEAT line.")
                    self.repeating = False
                    return

                # figure out how many times we should repeat the script
                if "MAX_STORIES" in elements[1]:
                    try:
                        self.max_repetitions = self.max_stories
                    except NameError:
                        self.logger.log("Tried to set MAX_REPETITIONS to " + 
                                "MAX_STORIES, but MAX_STORIES has not been set"
                                ". Setting to 1 repetition instead.")
                        self.max_repetitions = 1
                    else:
                        self.max_repetitions = elements[1]
                    self.logger.log("Going to repeat " + elements[2] + " " +
                            self.max_repetitions + " times.")


        # if we get a stop iteration exception, we're at the end of the file
        # and will stop iterating over lines
        except StopIteration:
            # if we were repeating a script, increment counter 
            if self.repeating:
                self.logger.log("Finished repetition " + self.repetitions + "!")
                self.repetitions += 1
                # if we've done enough repetitions, go back to the main session
                # script (set the repeating flag to false)
                if self.repetitions >= self.max_repetitions:
                    self.repeating = False

            else:
                self.logger.log("No more script lines to get!")
                # pass on the stop iteration exception
                raise

        # oh no got some unexpected error! raise it again so we can figure out
        # what happened and deal with it during debugging
        except:
            self.logger.log("Unexpected exception! Error:")
            print(sys.exc_info()[0])
            raise

    
    def read_list_from_file(self, filename):
        ''' Read a list of robot responses from a file, return a list of the
        lines from the file 
        '''
        # open script for reading
        try:
            fh = open(script, "r")
            return fh.readlines()
        except IOError:
            self.logger.log("Cannot open file: " + filename)
            # pass exception up so anyone trying to add a response list from
            # a script knows it didn't work
            raise

