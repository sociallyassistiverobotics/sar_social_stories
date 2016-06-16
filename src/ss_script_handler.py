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
import json # for packing ros message properties
import random # for picking robot responses and shuffling answer options
from ss_script_parser import ss_script_parser
from ss_personalization_manager import ss_personalization_manager
from ss_ros import ss_ros

class ss_script_handler():
    """ Social stories script handler parses and deals with script lines. Uses 
    the script parser to get the next line in a script. We keep loading script
    lines and parsing script lines separate on the offchance that we might want
    to replace how scripts are stored and accessed (e.g., in a database versus 
    in text files). 
    """

    def __init__(self, logger, ros_node, session, participant, script_path,
            story_script_path, session_script_path):
        """ Save references to ROS connection and logger, get scripts and
        set up to read script lines 
        """
        # save reference to logger for logging stuff later
        self.logger = logger
        self.logger.log("Setting up script handler...")

        # save reference to our ros node so we can publish messages
        self.ros_node = ros_node

        # save script paths so we can load scripts later
        self.script_path = script_path

        if (story_script_path is None):
            self.story_script_path = "" 
        else:
            self.story_script_path = story_script_path 

        if (session_script_path is None):
            self.session_script_path = ""
        else:
            self.session_script_path = session_script_path 

        # set up personalization manager so we can get personalized 
        # stories for this participant
        self.personalization_manager = ss_personalization_manager(self.logger,
                session, participant)

        # set up counter for how many stories have been told this session
        self.stories_told = 0

        # when we start, we are not currently telling a story or 
        # repeating a script
        self.doing_story = False
        self.repeating = False

        # set up script parser
        self.script_parser = ss_script_parser(self.logger)

        # get session script from script parser and story scripts from
        # the personalization manager, and give to the script parser
        try:
            self.script_parser.load_script(self.script_path
                    + self.session_script_path
                    + self.script_parser.get_session_script(session))
        except IOError:
            self.logger.log("Script parser could not open session script!")
            # pass exception up so whoever wanted a script handler knows
            # they didn't get a script
            raise

        # save start time so we can check whether we've run out of time
        self.start_time = datetime.datetime.now()


    def iterate_once(self):
        """ Play the next commands from the script """
        try:
            # we check whether we've reached the game time limit when
            # we load new stories or when we are about to start a
            # repeating script over again

            # get next line from story script
            if self.doing_story:
                self.logger.log("Getting next line from story script...")
                line = self.story_parser.next_line()
            # if not in a story, get next line from repeating script
            elif self.repeating:
                self.logger.log("Getting next line from repeating script...")
                line = self.repeat_parser.next_line()
            # if not repeating, get next line from main session script
            else:
                self.logger.log("Getting next line from main session script...")
                line = self.script_parser.next_line()
            
        # we didn't read a line!
        # if we get a stop iteration exception, we're at the end of the
        # file and will stop iterating over lines
        except StopIteration:
            # if we were doing a story, now we're done, go back to
            # the previous script
            if self.doing_story:
                self.logger.log("Finished story " + str(self.stories_told + 1) +
                        " of " + str(self.max_stories) + "!")
                self.doing_story = False
                self.stories_told += 1
            # if we were repeating a script, increment counter
            elif self.repeating:
                self.repetitions += 1
                self.logger.log("Finished repetition " + str(self.repetitions)
                    + " of " + str(self.max_repetitions) + "!")
                self.logger.log("repetitions >= max_repetitions ? " +
                        str(self.repetitions >= self.max_repetitions))
                print(self.repetitions)
                print(self.max_repetitions)
                # if we've done enough repetitions, or if we've run out
                # of game time, go back to the main session script (set
                # the repeating flag to false)
                if (self.repetitions >= self.max_repetitions) \
                        or (datetime.datetime.now() - self.start_time \
                        >= self.max_game_time):
                    self.logger.log("Done repeating!")
                    self.repeating = False
            # otherwise we're at the end of the main script
            else:
                self.logger.log("No more script lines to get!")
                # pass on the stop iteration exception
                raise

        except ValueError:
            # We may get this exception if we try to get the next line
            # but the script file is closed. If that happens, something
            # probably went wrong with ending playback of a story script
            # or a repeating script. End repeating and end the current
            # story so we go back to the main session script.
            # TODO may need to handle this differently!
            if self.doing_story:
                self.doing_story = False
            if self.repeating:
                self.repeating = False

        # oh no got some unexpected error! raise it again so we can
        # figure out what happened and deal with it during debugging
        except Exception as e:
            self.logger.log("Unexpected exception! Error:")
            self.logger.log(e)
            self.logger.log(sys.exc_info()[0])
            raise

        # we got a line: parse it!
        else:
            # Make sure we got a line before we try parsing it. We 
            # might not get a line if the file has closed or if
            # next_line has some other problem.
            if not line:
                self.logger.log("[iterate_once] Tried to get next line, but got None!")
                return

            # got a line - print for debugging
            self.logger.log("LINE: " + repr(line))
            # TODO switch prints to logger.logs if we want to log them

            # parse line!
            # split on tabs
            elements = line.rstrip().split('\t')
            print("... " + str(len(elements)) + " elements: \n... " + str(elements))

            if len(elements) < 1:
                self.logger.log("Line had no elements! Going to next line...")
                return

            # do different stuff depending on what the first element is
            #########################################################
            # only STORY lines have only one part to the command
            elif len(elements) == 1:
                # for STORY lines, play back the next story for this
                # participant
                if "STORY" in elements[0]:
                    print("STORY")
                    # if line indicates we need to start a story, do so 
                    self.doing_story = True
                    # create a script parser for the filename provided,
                    # assume it is in the session_scripts directory 
                    self.story_parser = ss_script_parser(self.logger)
                    try:
                        self.story_parser.load_script(self.script_path
                           + self.story_script_path
                           + self.personalization_manager.get_next_story_script())
                    except IOError:
                        self.logger.log("Script parser could not open story "
                                + "script! Skipping STORY line.")
                        self.doing_story = False
                    except AttributeError:
                        self.logger.log("Script parser could not open story "
                                + "script because no script was loaded! "
                                + "Skipping STORY line.")
                        self.doing_story = False

            # line has 2+ elements, so check the other commands
            #########################################################
            # for ROBOT lines, send command to the robot
            elif "ROBOT" in elements[0]:
                print("ROBOT")
                # play a randomly selected story intro from the list
                if "STORY_INTRO" in elements[1]:
                    self.ros_node.send_robot_command("DO", self.story_intros[
                        random.randint(0,len(self.story_intros)-1)])

                # play a randomly selected story closing from the list
                elif "STORY_CLOSING" in elements[1]:
                    self.ros_node.send_robot_command("DO", self.story_closings[
                        random.randint(0,len(self.story_closings)-1)])
                
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
                    to_load = self.read_list_from_file(
                            self.script_path + self.session_script_path +
                            elements[2])
                    for obj in to_load:
                        self.ros_node.send_opal_command("LOAD_OBJECT", obj)

                # get the next story and load graphics into game
                elif "LOAD_STORY" in elements[1]:
                    self.load_next_story()

                # load answers for game
                elif "LOAD_ANSWERS" in elements[1] and len(elements) >= 3:
                    self.load_answers(elements[2])

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
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.correct_responses)))
                    elif "INCORRECT_RESPONSES" in elements[1]:
                        self.incorrect_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.incorrect_responses)))
                    elif "YES_RESPONSES" in elements[1]:
                        self.yes_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.yes_responses)))
                    elif "NO_RESPONSES" in elements[1]:
                        self.no_responses = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.no_responses)))
                    elif "ANSWER_FEEDBACK" in elements[1]:
                        self.answer_feedback = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.answer_feedback)))
                    elif "STORY_INTROS" in elements[1]:
                        self.story_intros = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.story_intros)))
                    elif "STORY_CLOSINGS" in elements[1]:
                        self.story_closings = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.story_closings)))
                    elif "TIMEOUT_CLOSINGS" in elements[1]:
                        self.timeout_closings = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.timeout_closings)))
                    elif "MAX_STORIES_REACHED" in elements[1]:
                        self.max_stories_reached = self.read_list_from_file(
                                self.script_path + self.session_script_path +
                                elements[2])
                        self.logger.log("Got " + str(len(self.max_stories_reached)))
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
                    self.max_game_time = datetime.timedelta(minutes=
                            int(elements[2]))
                    self.logger.log("Set MAX_GAME_TIME to " + elements[2])
                elif "MAX_STORIES" in elements[1]:
                    self.max_stories = elements[2]
                    self.logger.log("Set MAX_STORIES to " + elements[2])

            #########################################################
            # for WAIT lines, wait for the specified user response, or timeout
            # if no response is received
            elif "WAIT" in elements[0] and len(elements) >= 3:
                self.wait_for_response(elements[1], int(elements[2]))

            #########################################################
            # for REPEAT lines, repeat lines in the specified script 
            # file the specified number of times
            elif "REPEAT" in elements[0] and len(elements) >= 3:
                self.repeating = True
                self.repetitions = 0
                # create a script parser for the filename provided, 
                # assume it is in the session_scripts directory 
                self.repeat_parser = ss_script_parser(self.logger)
                try:
                    self.repeat_parser.load_script(self.script_path
                            + self.session_script_path
                            + elements[2])
                except IOError:
                    self.logger.log("Script parser could not open session "
                        + "script to repeat! Skipping REPEAT line.")
                    self.repeating = False
                    return

                # figure out how many times we should repeat the script
                if "MAX_STORIES" in elements[1]:
                    try:
                        self.max_repetitions = self.max_stories
                    except AttributeError:
                        self.logger.log("Tried to set MAX_REPETITIONS to " + 
                                "MAX_STORIES, but MAX_STORIES has not been set"
                                ". Setting to 1 repetition instead.")
                        self.max_repetitions = 1
                else:
                    self.max_repetitions = elements[1]
                self.logger.log("Going to repeat " + elements[2] + " " +
                        self.max_repetitions + " time(s).")


    def read_list_from_file(self, filename):
        ''' Read a list of robot responses from a file, return a list
        of the lines from the file 
        '''
        # open script for reading
        try:
            fh = open(filename, "r")
            return fh.readlines()
        except IOError as e:
            self.logger.log("Cannot open file: " + filename)
            self.logger.log(e)
            # pass exception up so anyone trying to add a response list
            # from a script knows it didn't work
            raise


    def wait_for_response(self, response_to_get, timeout):
        ''' Wait for a user response or wait until the specified time 
        has elapsed. If the response is incorrect, allow multiple 
        attempts up to the maximum number of incorrect responses.
        '''
        for i in range(0, len(self.max_incorrect_responses)):
            self.logger.log("Waiting for user response...")
            # wait for the specified type of response, or until the 
            # specified time has elapsed
            response = self.ros_node.wait_for_response(response_to_get,
                    datetime.timedelta(seconds=int(timeout)))
            
            # after waiting for a response, need to play back an
            # appropriate robot response 
            # if response was CORRECT, randomly select a robot response
            # to a correct user action, highlight the correct answer, 
            # and break out of response loop
            if "CORRECT" in response:
                try:
                    self.ros_node.send_robot_command_and_wait("DO", 
                            "ROBOT_NOT_SPEAKING", 
                            datetime.timedelta(seconds=int(10)),
                            self.correct_responses[random.randint(0,
                                len(self.correct_responses)-1)])
                    self.ros_node.send_opal_command("SHOW_CORRECT")
                    self.ros_node.send_robot_command_and_wait("DO",
                            "ROBOT_NOT_SPEAKING", 
                            datetime.timedelta(seconds=int(10)),
                            self.answer_feedback[random.randint(0,
                                len(self.answer_feedback)-1)])
                    self.ros_node.send_opal_command("HIDE_CORRECT")
                except AttributeError:
                    self.logger.log("Could not play a correct " 
                            + "response or could not play robot's answer"
                            + "feedback because none were loaded!")
                break

            # if response was YES, randomly select a robot response to
            # the user selecting yes, and break out of response loop 
            elif "YES" in response:
                    try:
                        self.ros_node.send_robot_command("DO", 
                                self.yes_responses[random.randint(0,
                                    len(self.yes_responses)-1)])
                    except AttributeError:
                        self.logger.log("Could not play response to user's"
                                + "YES because none were loaded!")
                    break

            # if we received no user response before timing out, treat
            # as either NO or INCORRECT

            # if response was INCORRECT, randomly select a robot 
            # response to an incorrect user action
            elif ("INCORRECT" in response) or ("TIMEOUT" in response 
                    and "CORRECT" in response_to_get):
                try:
                    self.ros_node.send_robot_command("DO", 
                            self.incorrect_responses[random.randint(0,
                                len(self.incorrect_responses)-1)])
                except AttributeError:
                    self.logger.log("Could not play an incorrect " 
                            + "response because none were loaded!")

            # if response was NO, randomly select a robot response to
            # the user selecting no 
            elif "NO" in response or ("TIMEOUT" in response 
                    and "YES_NO" in response_to_get):
                try:
                    self.ros_node.send_robot_command("DO", 
                            self.no_responses[random.randint(0,
                                len(self.no_responses)-1)])
                except AttributeError:
                    self.logger.log("Could not play a response to " 
                            + "user's NO because none were loaded!")

        # we exhausted our allowed number of user responses, so have 
        # the robot do something
        else:
            # if user was never correct, play robot's correct answer
            # feedback and show which answer was correct in the game
            if "CORRECT" in response_to_get:
                try:
                    self.ros_node.send_opal_command("SHOW_CORRECT")
                    self.ros_node.send_robot_command_and_wait("DO",
                            "ROBOT_NOT_SPEAKING", 
                            datetime.timedelta(seconds=int(10)),
                            self.answer_feedback[random.randint(0,
                                len(self.answer_feedback)-1)])
                    self.ros_node.send_opal_command("HIDE_CORRECT")
                except AttributeError:
                    self.logger.log("Could not play robot's answer"
                            + " feedback because none were loaded!")
            
            # if user never selects YES (which is used to ask the user 
            # if they are ready to play), stop all stories and repeating
            # scripts, continue with main script so we go to the end
            elif "YES" in response_to_get:
                self.repeating = False
                self.story = False


    def load_answers(self, answer_list):
        ''' Load the answer graphics for this story '''
        # we are given a list of words that indicate what the answer
        # options are. By convention, the first word is probably the 
        # correct answer; the others are incorrect answers. However,
        # we won't set this now because this convention may not hold.
        # We expect the SET_CORRECT OpalCommand to be used to set
        # which answers are correct or incorrect.
        # split the list of answers on commas
        answers = answer_list.strip().split(',')

        # shuffle answers to display them in a random order
        random.shuffle(answers)

        # load in the graphic for each answer
        for answer in answers:
            toload = {}
            toload["name"] = answer.strip() # remove whitespace from name
            toload["tag"] = "PlayObject"
            toload["slot"] = answers.index(answer) + 1
            toload["draggable"] = False
            toload["isAnswerSlot"] = True
            self.ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))


    def load_next_story(self):
        ''' Get the next story, set up the game scene with scene and 
        answer slots, and load scene graphics.
        '''
        # if we've told the max number of stories, or if we've reached
        # the max game time, don't load another story even though we 
        # were told to load one -- instead, play error message from 
        # robot saying we have to be done now
        if self.stories_told >= self.max_stories or \
            datetime.datetime.now() - self.start_time >= self.max_game_time:
            self.logger.log("We were told to load another story, but we've "
                    + "already played the maximum number of stories or we ran"
                    " out of time! Skipping and ending now.")
            self.doing_story = False
            try:
                self.ros_node.send_robot_command("DO", self.max_stories_reached
                        [random.randint(0, len(self.no_responses)-1)])
            except AttributeError:
                self.logger.log("Could not play a max stories reached response"
                        + " because none were loaded!")
            # We were either told to play another story because a
            # repeating script loads a story and the max number of 
            # repetitions is greater than the max number of stories,
            # so more stories were requested than can be played, or 
            # because we ran out of time and were supposed to play more
            # stories than we have time for. Either way, stop the 
            # repeating script if there is one.
            self.repeating = False
            return

        # get the next story
        scenes, in_order, num_answers = \
            self.personalization_manager.get_next_story_details()

        # set up the story scene in the game
        setup = {}
        setup["numScenes"] = len(scenes)
        setup["scenesInOrder"] = in_order
        setup["numAnswers"] = num_answers
        self.ros_node.send_opal_command("SETUP_STORY_SCENE", json.dumps(setup))

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
            self.ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))

