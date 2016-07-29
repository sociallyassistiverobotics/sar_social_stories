# Jacqueline Kory Westlund
# July 2016
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
import sqlite3 # store game info and personalization

class ss_db_manager():
    """ Interface to database for storing personalization information. """

    def __init__(self, database):
        """ Initialize database connection. """
        # Set up logger
        self.logger = logging.getLogger(__name__)

        # Get connection to database.
        try:
            self.conn = sqlite3.connect(database)
            self.cursor = conn.cursor()
        except:
            self.logger.exception("Could not connect to database: " +
                database)
            raise


    def get_most_recent_level(self, participant, current_session):
        """ Get the level at which the participant played during the
        previous session.
        """
        # If there wasn't a previous session, we can't get its level.
        if (current_session < 1):
            return None

        try:
            return self.cursor.execute("""SELECT level_id FROM stories_played
                WHERE participant=(?) and session=(?) LIMIT 1""",
                participant, (current_session-1))
        except:
            self.logger.exception("Could not find level of previous session"
                + " for " + participant + " for session " + current_session
                + " in the database!")
            return None


    def get_most_recent_percent_correct_responses(self, participant,
            current_session):
        """ Get the percentage of the participant's question responses
        from the previous session that were correct.
        """
        #TODO
        pass


    def get_most_recent_incorrect_emotions(self, participant, current_session):
        """ Get a list of the target emotions for all questions from
        the last session where the participant responded incorrectly.
        """
        #TODO
        pass


    def get_next_new_story(self, participant, current_session, emotions):
        """ Get the next unplayed story from the story table with at
        least one of the listed emotions present in the story. If no
        unplayed story has the desired emotions or if there are no
        desired emotions, return the name of the next unplayed story.
        If there are no more unplayed stories, return None.
        """
        #TODO
        pass


    def get_next_review_story(self, participant, current_session, emotions):
        """ Get a review story with at least one of the listed emotions
        present in the story that wasn't played in the current session.
        If no played stories have the desired emotions, return the
        name of the story heard least recently and least often. If there
        are no stories we can review, return None.
        """
        #TODO
        pass


    def get_level_info(self, level):
        """ Get information about stories at this level: whether the
        scenes are presented in order or not, and how many answer
        options are shown when questions are asked.
        """
        #TODO
        #return in_order, num_answers
        pass


    def get_graphics(self, story, level):
        """ Get the list of names of graphics for the scenes in a story
        at the specified level.
        """
        #TODO
        pass


    def record_story_played(participant, session, level, story):
        """ Insert the participant ID, session number, story level,
        current date and time, and a reference to the current story
        into the stories_played table.
        """
        #TODO
        pass


    def record_response(): #TODO fill in necessary arguments
        """ Insert a user response into the responses table: we need
        the question ID, stories_played ID, and the actual response.
        """
        #TODO
        pass
