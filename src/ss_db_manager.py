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
            # Pass on exception for now.
            raise


    def get_most_recent_level(self, participant, current_session):
        """ Get the level at which the participant played during the
        previous session (return None if no previous session).
        """
        # If there wasn't a previous session, we can't get its level.
        if (current_session < 1):
            return None

        try:
            result = self.cursor.execute("""
                SELECT level_id
                FROM stories_played
                    WHERE participant=(?)
                    AND session=(?)
                    ORDER BY time DESC
                    LIMIT 1""",
                    (participant, (current_session-1))).fetchone()
            if result is None:
                self.logger.warn("Could not find level of previous session"
                    + " for " + participant + " for session " + current_session
                    + " in the database!")
                return None
            else:
                # Database gives us a tuple, so return the first element.
                return result[0]
        except Exception as e:
            self.logger.exception("Could not find level of previous session"
                    + " for " + participant + " for session " + current_session
                    + " in the database!")
            # Pass on exception for now.
            raise


    def get_most_recent_percent_correct_responses(self, participant,
            current_session):
        """ Get the percentage of the participant's question responses
        from the previous session that were correct.
        """
        try:
            result = self.cursor.execute("""
                SELECT ...
                FROM ...
                WHERE ... """, (participant, (current_session-1)))
            #TODO fill in query!
            if result is None:
                self.logger.warn("Could not find any responses for "
                    + participant + " for session " + (current_session-1)
                    + " in the database!")
                return None
            else:
                # TODO Return percent responses correct.
                pass
        except Exception as e:
            self.logger.exception("Could not find any responses for "
                + participant + " for session " + (current_session-1)
                + " in the database!")
            # Pass on exception for now.
            raise


    def get_most_recent_incorrect_emotions(self, participant, current_session):
        """ Get a list of the target emotions for all questions from
        the last session where the participant responded incorrectly.
        """
        # May not be able to use ORDER BY in the subquery - if this is
        # a problem, fix later.
        try:
            result = self.cursor.execute("""
                SELECT DISTINCT responses.response, questions.target_response
                FROM responses
                JOIN questions
                   ON questions.id = responses.questions_id
                   WHERE questions.target_response <> responses.response
                   AND responses.stories_played_id = (
                       SELECT id
                       FROM stories_played
                       WHERE participant = (?)
                          AND session = (?)
                       ORDER BY time DESC
                       LIMIT 1)
                """, (participant, current_session))

            if result is None:
                self.logger.warn("Could not find any incorrect responses for "
                    + participant + " for session " + (current_session-1)
                    + " in the database!")
                return None
            else:
                # Database gives us a list of tuples, so convert to a
                # a list before returning.
                return [emotion[0] for emotion in result]
        except Exception as e:
            self.logger.exception("Could not find any incorrect responses for "
                + participant + " for session " + (current_session-1)
                + " in the database!")
            # Pass on exception for now.
            raise


    def get_next_new_story(self, participant, current_session, emotions):
        """ Get the next unplayed story from the story table with at
        least one of the listed emotions present in the story. If no
        unplayed story has the desired emotions or if there are no
        desired emotions, return the name of the next unplayed story.
        If there are no more unplayed stories, return None.
        """
        try:
            result = self.cursor.execute("""
                SELECT ...
                FROM ...
                WHERE ... """, (participant, current_session))
            #TODO fill in query!
            if result is None:
                self.logger.warn("Could not find any unplayed stories for "
                    + participant + " for session " + current_session +
                    " with emotions " + emotions + " in the database!")
                return None
            else:
                # TODO Return the name of a new story to play.
                pass
        except Exception as e:
            self.logger.exception("Could not find any unplayed stories for "
                    + participant + " for session " + current_session +
                    " with emotions " + emotions + " in the database!")
            # Pass on exception for now.
            raise



    def get_next_review_story(self, participant, current_session, emotions):
        """ Get a review story with at least one of the listed emotions
        present in the story that wasn't played in the current session.
        If no played stories have the desired emotions, return the
        name of the story heard least recently and least often. If there
        are no stories we can review, return None.
        """
        try:
            result = self.cursor.execute("""
                SELECT ...
                FROM ...
                WHERE ... """, (participant, current_session))
            #TODO fill in query!
            if result is None:
                self.logger.warn("Could not find any stories to review for "
                    + participant + " for session " + current_session +
                    + " with emotions " + emotions + " in the database!")
                return None
            else:
                # TODO Return a story to review.
                pass
        except Exception as e:
            self.logger.exception("Could not find any stories to review for "
                    + participant + " for session " + current_session +
                    + " with emotions " + emotions + " in the database!")
            # Pass on exception for now.
            raise


    def get_level_info(self, level):
        """ Get information about stories at this level: whether the
        scenes are presented in order or not, and how many answer
        options are shown when questions are asked.
        """
        try:
            result = self.cursor.execute("""
                SELECT num_answers, in_order
                FROM levels
                WHERE level=(?)
                """, (level,))
            if result is None:
                self.logger.warn("Could not find info for level " + level
                        + " in the database!")
                return None
            else:
                # Database gives us a tuple, so return first element as
                # the number of answers and convert the second element
                # to a boolean.
                return result[0], (True if result[1] == 1 else False)
        except Exception as e:
            self.logger.exception("Could not find info for level " + level
                    + " in the database!")
            # Pass on exception for now.
            raise


    def get_graphics(self, story, level):
        """ Get the list of names of graphics for the scenes in a story
        at the specified level.
        """
        try:
            result = self.cursor.execute("""
                SELECT graphic
                FROM graphics
                WHERE level_id=(?)
                AND story_id=(
                    SELECT id
                    FROM stories
                    WHERE story_name=(?))
                """, (level, story)).fetchall()
            if result is None:
                self.logger.warn("Could not find graphics for story " + story
                    + " at level " + level + " in the database!")
                return None
            else:
                # Database gives us a list of tuples of graphic names,
                # so make this into a list of graphic names.
                return [name[0] for name in result]
        except Exception as e:
            self.logger.exception("Could not find graphics for story " + story
                    + " at level " + level + " in the database!")
            # Pass on exception for now.
            raise


    def record_story_played(participant, session, level, story):
        """ Insert the participant ID, session number, story level,
        current date and time, and a reference to the current story
        into the stories_played table.
        """
        try:
            self.cursor.execute("""
                INSERT INTO stories_played (participant, session,
                    level_id, story_id)
                VALUES (
                (?),
                (?),
                (SELECT id
                    FROM stories
                    WHERE story_name=(?)),
                (SELECT level
                    FROM levels
                    WHERE level=(?)))
                """, (participant, session, story, level))
        except Exception as e:
            self.logger.exception("Could not insert record into stories_played"
                + " table in database! Tried to insert: participant=" +
                participant + ", session=" + session + ", level=" + level +
                ", story=" + story)
            # Pass on exception for now.
            raise


    def record_response(participant, session, level, story, question_num,
            question_type, response):
        """ Insert a user response into the responses table: we need
        the question ID, stories_played ID, and the actual response.
        """
        try:
            self.cursor.execute("""
                INSERT INTO responses (stories_played_id, questions_id,
                    response
                VALUES (
                (SELECT id from stories_played
                    WHERE participant=(?)
                    AND session=(?)
                    AND level_id=(
                        SELECT level
                        FROM levels
                        WHERE level=(?))
                    AND story_id=(
                        SELECT id
                            FROM stories
                            WHERE story_name=(?))
                    ORDER BY time DESC
                    LIMIT 1),
                (SELECT id
                    FROM questions
                    WHERE question_num=(?)
                    AND question_type=(?)
                    AND level=(
                        SELECT level
                        FROM levels
                        WHERE level=(?))
                    AND story_id=(
                        SELECT id
                        FROM stories
                        WHERE story_name=(?))),
                (?))
                """, (participant, session, level, story, question_num,
                    question_type, level, story, response))
        except Exception as e:
            self.logger.exception("Could not insert record into questions"
                + " table in database! Tried to insert: participant=" +
                participant + ", session=" + session + ", level=" + level +
                ", story=" + story + ", question_num=" + question_num +
                ", question_type=" + question_type + ", response=" + response)
            # Pass on exception for now.
            raise
