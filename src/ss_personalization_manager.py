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
from ss_db_manager import ss_db_manager

class ss_personalization_manager():
    """ Determine personalization for a participant, given their past 
    performance and the current session """ 


    def __init__(self, participant, session):
        """ Initialize stuff """
        # Set up logger
        self.logger = logging.getLogger(__name__)

        self.logger.info("TODO initialize personalization manager - "
            + "using DEMO setup")

        # Save participant and session so we can use them to get their
        # past performance from the database, which we will use to 
        # determine which stories to present and what level to use. 
        self.participant = participant
        self.session = session

        # Get database manager.
        self.db_man = ss_db_manager()

        # Get the level for this session.
        self.level = self.get_level_for_session()

        # We can't get a queue of stories, because we don't know how
        # many we would need to queue up. Instead, get a list of the
        # emotions that the participant needs the most practice with
        # that should be present in the stories this session.
        self.emotion_list = self.get_emotion_list()


    def get_level_for_session(self):
        """ Determine which level the stories told in this session
        should be at.
        """
        # Data for this participant's last session is in the database.
        # Use their past performance to decide whether to level up.
        #TODO
        # need last time's level, number of questions correct last time
        # if 75%-80% correct, level up
        self.logger.warn("TODO get level")


    def get_emotion_list(self):
        """ Determine which stories will be told this session, based
        on which stories have already been heard.
        """
        # Data for this participant's last session is in the database.
        # Use this participant's past performance to determine which
        # emotions they need more practice with. We can use this info
        # to load appropriate stories this session.
        #TODO
        # need emotion questions incorrect for the past session(s)
        # need list of all emotions
        self.logger.warn("TODO get emotion list")


    def get_next_story_script(self):
        """ Determine which story should be heard next. """
        if (self.session == -1):
            self.logger.debug("Using DEMO script.")
            return "demo-story-1.txt"
        else:
            # TODO Return name of the next script file, based on which
            # emotions should be present in the stories, which stories
            # have already been heard, and how much review to do.
            # need: list of stories heard, emotion list
            self.logger.warn("TODO return next script file name!")


    def get_next_story_details(self):
        """ Determine the number of scenes, whether they are shown in
        order, and the number of answer options for the next story.
        """
        # If this is a demo session, load a demo scene.
        if (self.session == -1):
            # Demo set:
            scenes = ["scenes/CR1-scene1.png", "scenes/CR1-scene2.png", 
                    "scenes/CR1-scene3.png", "scenes/CR1-scene4.png"]

            # Demo story has scenes in order.
            in_order = True

            # Demo has 4 scenes.
            num_answers = 4

            self.logger.debug("DEMO story:\nScenes: " + str(scenes)
                    + "\nIn order: " + str(in_order)
                    + "\nNum answers: " + str(num_answers))
        else:
            # TODO Get list of scene graphics names.
            # TODO Determine whether the scenes are shown in order.
            # TODO Determine how many answer options there are per
            # question at this story level.
            self.logger.warn("TODO get story details")

        # Return the story information.
        return scenes, in_order, num_answers


    def get_joint_attention_level(self):
        """ Determine what level of joint attention scaffolding to provide 
        each time it is required.
        """
        self.logger.debug("TODO determine joint attention level")
