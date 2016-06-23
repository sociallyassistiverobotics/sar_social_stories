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
import logging # log messages

class ss_personalization_manager():
    """ Determine personalization for a participant, given their past 
    performance and the current session """ 


    def __init__(self, participant, session):
        """ Initialize stuff """
        # set up logger
        self.logger = logging.getLogger(__name__)

        self.logger.info("TODO initialize personalization manager - "
            + "using DEMO setup")

        # save participant and session so we can use them to determine 
        # which stories to present and to load any saved files
        self.participant = participant
        self.session = session

        # TODO load any saved files for this participant
        # TODO check personalization, determine which stories to present


    def get_next_story_script(self):
        """ Determine which story should be heard next """
        self.logger.debug("TODO get story scripts - using DEMO set")
        # TODO get name of next script file
        script_file = "demo-story-1.txt"
        return script_file


    def get_next_story_details(self):
        """ Determine the number of scenes, whether they are shown in
        order, and the number of answer options for the next story.
        """
        # TODO get list of scene graphics names
		# demo set:
        scenes = ["scenes/CR1-scene1.png", "scenes/CR1-scene2.png", 
				"scenes/CR1-scene3.png", "scenes/CR1-scene4.png"]

        # TODO determine whether the scenes are shown in order or not
		# demo is in order
        in_order = True

        # TODO determine how many answer options there are per question 
        # at this story level
		# demo has 4 scenes
        num_answers = 4

        self.logger.debug("TODO get story details - using DEMO set:\nScenes: "
                + str(scenes) + "\nIn order: " + str(in_order) 
                + "\nNum answers: " + str(num_answers))

        # return the story information
        return scenes, in_order, num_answers


    def get_joint_attention_level(self):
        """ Determine what level of joint attention scaffolding to provide 
        each time it is required """
        self.logger.debug("TODO determine joint attention level")
