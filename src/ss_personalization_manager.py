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
from SS_Errors import NoStoryFound

class ss_personalization_manager():
    """ Determine personalization for a participant, given their past
    performance and the current session """

    def __init__(self, participant, session, database):
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
        self.db_man = ss_db_manager(database)

        # Get the level for this session.
        self.level = self.get_level_for_session()

        # In each session, alternate between telling new stories and
        # telling previously told stories (if possible -- obviously in
        # the earlier sessions, there is less review available). Start
        # with a new story.
        self.tell_new_story = True

        # We don't have a current story yet
        self.current_story = None

        # We can't get a queue of stories, because we don't know how
        # many we would need to queue up. Instead, get a list of the
        # emotions that the participant needs the most practice with
        # that should be present in the stories this session.
        self.emotion_list = self.get_emotion_list()
        # Need emotion questions incorrect for the past session(s).
        self.emotion_list = self.db_man.get_most_recent_incorrect_emotions(
            self.participant, self.session)


    def get_level_for_session(self):
        """ Determine which level the stories told in this session
        should be at.
        """
        # Data for this participant's last session is in the database.
        # Use their past performance to decide whether to level up.
        # need last time's level, number of questions correct last time
        level = self.db_man.get_most_recent_level(self.participant,
            self.session)
        # If there is no previous data, start at level 1.
        if (level is None):
            return 1
        # If participant got 75%-80% questions correct last time, level up
        #TODO total performance or just last time's performance?
        if(self.db_man.get_most_recent_percent_correct_responses(
            self.participant, self.session) > 0.75):
            self.logger.info("Participant got >75% questions correct last \
                time, so we can level up! Level will be " + str(level+1) + ".")
            return level + 1
        else:
            self.logger.info("Participant got <75% questions correct last \
                time, so we don't level up. Level will be " + str(level) + ".")
            return level


    def get_next_story_script(self):
        """ Determine which story should be heard next. We have 40
        stories. Alternate telling new stories and telling review
        stories. Earlier sessions will use more new stories since there
        isn't much to review. Return name of the next script file, based
        on which emotions should be present in the stories, which
        stories have already been heard, whether we should tell a review
        story, and the current level.
        """
        # If this is a demo session, use the demo script.
        if (self.session == -1):
            self.logger.debug("Using DEMO script.")
            return "demo-story-1.txt"

        # If we should tell a new story, get the next new story that
        # has one of the emotions to practice in it. If there aren't
        # any stories with one of those emotions, just get the next new
        # story.
        elif self.tell_new_story:
            self.logger.warn("TODO return next script file name!")
            story = self.db_man.get_next_new_story(self.participant,
                self.session, self.emotion_list)

        # If there are no more new stories to tell, or if we need to
        # tell a review story next, get a review story that has one of
        # the emotions to practice in it. If there aren't any with
        # those emotions, get the oldest, least played review story.
        if (story is None) or not self.tell_new_story:
            story = self.db_man.get_next_review_story(self.participant,
                self.session, self.emotion_list)

        # If there are no review stories available, get a new story
        # instead (this may happen if we are supposed to tell a review
        # story but haven't told very many stories yet).
        if (story is None):
            story = self.db_man.get_next_new_story(self.participant,
                self.session, self.emotion_list)

        # If we still don't have a story, then for some reason there
        # are no new stories and no review stories we can tell. This is
        # a problem.
        if (story is None):
            self.logger.error("We were supposed to get the next story \
                but could not find a new or a review story that we \
                can play.")
            raise NoStoryFound("Could not find new or review story to play.",
                    self.participant, self.session)

        # Toggle flag for telling new versus telling previously heard
        # stories (since we alternate).
        self.tell_new_story = not self.tell_new_story

        # Save current story so we can provide story details later.
        self.current_story = story

        # Return name of story script: story name + level + file extension.
        return story + str(self.level) + ".txt"


    def get_next_story_details(self):
        """ Determine the number of scenes, whether they are shown in
        order, and the number of answer options for the next story.
        """
        # If this is a demo session, load a demo scene.
        if (self.session == -1):
            # Demo set:
            graphic_names = ["scenes/CR1-scene1.png", "scenes/CR1-scene2.png",
                    "scenes/CR1-scene3.png", "scenes/CR1-scene4.png"]

            # Demo story has scenes in order.
            in_order = True

            # Demo has 4 scenes.
            num_answers = 4

            self.logger.debug("DEMO story:\nScenes: " + str(scenes)
                    + "\nIn order: " + str(in_order)
                    + "\nNum answers: " + str(num_answers))

        # If the current story isn't set, throw exception.
        elif (self.current_story is None):
           self.logger.error("We were asked for story details, but we \
                haven't picked the next story yet!")
           raise NoStoryFound("No current story is set.", self.participant,
               self.session)

        # Otherwise, we have the current story.
        else:
            # Get story information from the database: scene graphics
            # names, whether the scenes are shown in order, how many
            # answer options there are per question at this level.
            graphic_names = self.db_man.get_graphics(self.current_story,
                self.level)
            in_order, num_answers = self.db_man.get_level_info(self.level)

        # Return the story information.
        return graphic_names, in_order, num_answers


    def record_story_loaded(self):
        """ Record that we loaded a story, and that this participant is
        playing this story.
        """
        #TODO call database and give info for stories_played table
        self.db_man.record_story_played(self.participant, self.session,
            self.level, self.current_story)


    def record_user_response(self):
        """ Record that the participant responded to one of the story
        questions.
        """
        self.db_man.record_response() #TODO fill in necessary arguments


    def get_joint_attention_level(self):
        """ Determine what level of joint attention scaffolding to provide
        each time it is required.
        """
        self.logger.debug("TODO determine joint attention level")
