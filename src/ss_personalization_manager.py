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

    def __init__(self, session, participant, database,
            percent_correct_to_level):
        """ Initialize stuff """
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Initializing personalization manager...")

        # Save participant and session so we can use them to get their
        # past performance from the database, which we will use to
        # determine which stories to present and what level to use.
        self._participant = participant
        self._session = session

        # Store the percent of questions a player has to get right in
        # the previous session in order to level up in this session, so
        # we can use it to determine whether a player will level up.
        self._percent_correct_to_level = percent_correct_to_level
        # Get database manager, but don't require the database for a
        # DEMO session!
        if (self._session != -1):
            self._db_man = ss_db_manager(database)

        # Get the level for this session.
        self._level = self.get_level_for_session()

        # In each session, alternate between telling new stories and
        # telling previously told stories (if possible -- obviously in
        # the earlier sessions, there is less review available). Start
        # with a new story.
        self._tell_new_story = True

        # We don't have a current story yet.
        self._current_story = None

        # We can't get a queue of stories, because we don't know how
        # many we would need to queue up. Instead, get a list of the
        # emotions that the participant needs the most practice with
        # that should be present in the stories this session. These
        # will be the emotions gotten incorrect in the past session.
        # Skip this if this is a demo session.
        if (self._session != -1):
            self._emotion_list = self._db_man.get_most_recent_incorrect_emotions(
                self._participant, self._session)


    def get_level_for_session(self):
        """ Determine which level the stories told in this session
        should be at.
        """
        # Use level 1 if this is a demo session.
        if (self._session == -1):
            return 1

        # Data for this participant's last session is in the database.
        # Use their past performance to decide whether to level up.
        # need last time's level, number of questions correct last time
        level = self._db_man.get_most_recent_level(self._participant,
            self._session)
        # If there is no previous data, start at level 1.
        if (level is None):
            return 1
        # If participant got 75%-80% questions correct last time, level
        # up. If no responses were found or not enough were answered
        # correctly, do not level up.
        #TODO total performance or just last time's performance?
        past_performance = self._db_man.get_percent_correct_responses(
            self._participant, (self._session - 1))
        if past_performance is None:
            self._logger.info("Participant did not answer any questions last "
                + "time, so we will not level up. Level will be " + str(level)
                + ".")
            return level
        elif (past_performance > percent_correct_to_level):
            self._logger.info("Participant got more than " +
                (percent_correct_to_level*100) + "% questions correct last "
                + "time, so we can level up! Level will be " + str(level+1)
                + ".")
            return level + 1
        else:
            self._logger.info("Participant got less than " +
                (percent_correct_to_level*100) + "% questions correct last "
                + "time, so we don't level up. Level will be " + str(level)
                + ".")
            return level


    def get_performance_this_session(self):
        """ Get the user's performance on all questions asked this
        session, by question type, and format as a json object.
        """
        # Only get the user's performance if this isn't a DEMO session.
        if (self._session != -1):
            # Get the user's performance on the emotion questions, on
            # the theory of mind questions, and on the order questions.
            return self._db_man.get_percent_correct_responses(
                self._participant, self._session, "emotion"), \
                self._db_man.get_percent_correct_responses(self._participant,
                self._session, "ToM"), \
                self._db_man.get_percent_correct_responses( \
                self._participant, self._session, "order")
        else:
            return None


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
        if (self._session == -1):
            self._logger.debug("Using DEMO script.")
            return "demo-story-1.txt"

        # If we should tell a new story, get the next new story that
        # has one of the emotions to practice in it. If there aren't
        # any stories with one of those emotions, just get the next new
        # story.
        elif self._tell_new_story:
            story = self._db_man.get_next_new_story(self._participant,
                self._session, self._emotion_list)

        # If there are no more new stories to tell, or if we need to
        # tell a review story next, get a review story that has one of
        # the emotions to practice in it. If there aren't any with
        # those emotions, get the oldest, least played review story.
        if (story is None) or not self._tell_new_story:
            story = self._db_man.get_next_review_story(self._participant,
                self._session, self._emotion_list)

        # If there are no review stories available, get a new story
        # instead (this may happen if we are supposed to tell a review
        # story but haven't told very many stories yet).
        if (story is None):
            story = self._db_man.get_next_new_story(self._participant,
                self._session, self._emotion_list)

        # If we still don't have a story, then for some reason there
        # are no new stories and no review stories we can tell. This is
        # a problem.
        if (story is None):
            self._logger.error("We were supposed to get the next story \
                but could not find a new or a review story that we \
                can play.")
            raise NoStoryFound("Could not find new or review story to play.",
                    self._participant, self._session)

        # Toggle flag for telling new versus telling previously heard
        # stories (since we alternate).
        self._tell_new_story = not self._tell_new_story

        # Save current story so we can provide story details later.
        self._current_story = story

        # Return name of story script: story name + level + file extension.
        return (story + "-" + str(self._level) + ".txt").lower()


    def get_next_story_details(self):
        """ Determine the number of scenes, whether they are shown in
        order, and the number of answer options for the next story.
        """
        # If this is a demo session, load a demo scene.
        if (self._session == -1):
            # Demo set:
            graphic_names = ["scenes/CR1-B-a.png", "scenes/CR1-B-b.png",
                    "scenes/CR1-B-c.png", "scenes/CR1-B-d.png"]

            # Demo story has scenes in order.
            in_order = True

            # Demo has 4 scenes.
            num_answers = 4

            self._logger.debug("DEMO story:\nScenes: " + str(graphic_names)
                    + "\nIn order: " + str(in_order)
                    + "\nNum answers: " + str(num_answers))

        # If the current story isn't set, throw exception.
        elif (self._current_story is None):
           self._logger.error("We were asked for story details, but we \
                haven't picked the next story yet!")
           raise NoStoryFound("No current story is set.", self._participant,
               self._session)

        # Otherwise, we have the current story.
        else:
            # Get story information from the database: scene graphics
            # names, whether the scenes are shown in order, how many
            # answer options there are per question at this level.
            graphic_names = self._db_man.get_graphics(self._current_story,
                self._level)
            num_answers, in_order = self._db_man.get_level_info(self._level)

        # Return the story information.
        return graphic_names, in_order, num_answers


    def record_story_loaded(self):
        """ Record that we loaded a story, and that this participant is
        playing this story.
        """
        # Skip if this is a demo session; otherwise record.
        if (self._session != -1):
            self._db_man.record_story_played(self._participant, self._session,
                self._level, self._current_story)


    def record_user_response(self, question_num, question_type, response):
        """ Record that the participant responded to one of the story
        questions.
        """
        # Skip if this is a demo session; otherwise record.
        if (self._session != -1):
            self._db_man.record_response(self._participant, self._session,
                self._level, self._current_story, question_num, question_type,
                response)


    def set_start_level(self, level):
        """ When the game starts, a level to start at can be provided.
        We're going to ignore this and use our internal database and
        personalization algorithm to determine leveling, but we will
        print out an error if the level we pick is different from the
        level we are told to start at.
        """
        if (level != self._level):
            self._logger.warning("We were told to play at level " + str(level)
                + " but our internal personalization algorithm says we should "
                + "play at level " + str(self._level) + ". We will be playing "
                + "at level " + str(self._level))


    def get_joint_attention_level(self):
        """ Determine what level of joint attention scaffolding to provide
        each time it is required.
        """
        self._logger.debug("TODO determine joint attention level")
