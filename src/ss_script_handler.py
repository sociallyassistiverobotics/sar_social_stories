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
from ss_script_parser import ss_script_parser
from ss_personalization_manager import ss_personalization_manager
from ss_ros import ss_ros

class ss_script_handler():
    """ Social stories script handler parses and deals with script lines. Uses 
    the script parser to get the next line in a script. We keep loading script
    lines and parsing script lines separate on the offchance that we might want
    to replace how scripts are stored and accessed (e.g., in a database versus 
    in text files). """

    def __init__(self, logger, ros_node, session, participant):
        """ Save references to ROS connection and logger, get scripts and
        set up to read script lines """

        # save reference to logger for logging stuff later
        self.logger = logger
        self.logger.log("Setting up script handler...")

        # save reference to our ros node so we can publish messages
        self.ros_node = ros_node

        # set up personalization manager so we can get personalized stories
        # for each participant
        self.personalization_manager = ss_personalization_manager(self.logger)

        # get story scripts from personalization manager
        self.personalization_manager.get_story_scripts(session, participant)
        
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
        """ play the next commands from the script """
        try:
            # get next line
            line = self.script_parser.next_line()

            # got a line, what do we do with it?
            print("LINE: " + repr(line))

            # TODO sometimes we need to wait for a response
            #self.ros_node.send_opal_command_and_wait(6, 
                    #datetime.timedelta(seconds=10))

            # TODO if line indicates we need to start a story, load next story 
            # in its own script parser?

        except StopIteration:
            self.logger.log("No more script lines to get!")
            # pass on the stop iteration exception
            raise

        except:
            self.logger.log("Unexpected exception! Error:")
            print(sys.exc_info()[0])
            raise

        
