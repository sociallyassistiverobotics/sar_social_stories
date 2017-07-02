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
        self._logger = logging.getLogger(__name__)

        # Get connection to database.
        try:
            self._conn = sqlite3.connect(database)
            self._cursor = self._conn.cursor()
        except:
            self._logger.exception("Could not connect to database: " +
                database)
            # Pass on exception for now.
            raise

    def __del__(self):
        """ Destructor. """
        # Close database connection.
        self._cursor.close()
        self._conn.close()


    def get_most_recent_level(self, participant, current_session):
        """ Get the level at which the participant played during the
        previous session (return None if no previous session).
        """
        # If there wasn't a previous session, we can't get its level.
        if (current_session < 1):
            return None

        try:
            result = self._cursor.execute("""
                SELECT level
                FROM stories_played
                    WHERE participant = (?)
                    AND session = (?)
                    ORDER BY time DESC
                    LIMIT 1""",
                    (participant, (current_session-1))).fetchone()
            if result is None:
                self._logger.warn("Could not find level of previous session"
                    + " for " + participant + " for session "
                    + str(current_session) + " in the database!")
                return None
            else:
                # Database gives us a tuple, so return the first element.
                return result[0]
        except Exception as e:
            self._logger.exception("Failed when trying to find the level of "
                "previous session" + " for " + participant + " for session "
                + str(current_session) + " in the database!")
            # Pass on exception for now.
            raise


    def get_percent_correct_responses(self, participant, session,
            question_type=None):
        """ Get the percentage of the participant's questions responses
        from the specified session that were correct. Unless
        question_type is specified, this function will look at ALL
        question types. If specified, only that single type of question
        will be counted (e.g., only emotion questions).
        """
        try:
            # Get the number of correct responses (i.e., the questions
            # from the participant's last session where their response
            # was equal to the target response).
            # The responses table can be empty if no responses from the
            # participant have been recorded yet. The questions table
            # should never be empty (filled when stories are imported).
            total_correct = self._cursor.execute("""
                SELECT COUNT(responses.response)
                FROM responses
                JOIN questions
                    ON questions.id = responses.questions_id
                JOIN stories_played
                    ON responses.stories_played_id = stories_played.id
                WHERE questions.target_response = responses.response
                    AND stories_played.participant = (?)
                    AND stories_played.session = (?)
                """
                # Only filter by question type if one was provided.
                + ("" if (question_type is None) else \
                    " AND questions.question_type = (?)"),
                # Provide the question_type if we need to filter by it.
                ((participant, session) if (question_type is None) else \
                    (participant, session, question_type))
                ).fetchone()
                #TODO helper function, "correct" as parameter like question type

            # The participant may not have responded to any questions
            # correctly.
            if total_correct is None:
                self._logger.warn("Could not find any correct responses for "
                    + participant + " for session " + str(session)
                    + " in the database!")
                total_correct = 0

            # Get the total number of responses made by the participant
            # in the last session.
            total_responses = self._cursor.execute("""
                SELECT COUNT(responses.response)
                FROM responses
                JOIN questions
                    ON questions.id = responses.questions_id
                    WHERE responses.stories_played_id in (
                        SELECT id
                        FROM stories_played
                        WHERE participant = (?)
                           AND session = (?)
                        ORDER BY time DESC)
                    """
                    # Only filter by question type if one was provided.
                    + ("" if (question_type is None) else \
                        " AND questions.question_type = (?)"),
                 # Provide the question_type if we need to filter by it.
                ((participant, session) if (question_type is None) else \
                    (participant, session, question_type))
                ).fetchone()

            # The participant may not have responded to any questions.
            if total_responses is None or total_responses[0] == 0:
                self._logger.warn("Could not find any responses for "
                    + participant + " for session " + str(session)
                    + " in the database!")
                total_responses = 0
                return None
            else:
                # Return percent responses correct (database gave us
                # these values in tuples).
                return float(total_correct[0]) / total_responses[0]
        except Exception as e:
            self._logger.exception("Failed when trying to find responses for "
                + participant + " for session " + str(session)
                + " in the database!")
            # Pass on exception for now.
            raise


    def get_most_recent_incorrect_emotions(self, participant, current_session):
        """ Get a list of the target emotions for all questions from
        the last session where the participant responded incorrectly.
        Return an empty list if none are found.
        """
        # May not be able to use ORDER BY in the subquery - if this is
        # a problem, fix later.
        # The responses table may be empty if no responses have been
        # recorded yet.
        try:
            result = self._cursor.execute("""
                SELECT DISTINCT responses.response, questions.target_response
                FROM responses
                JOIN questions
                    ON questions.id = responses.questions_id
                JOIN stories_played
                    ON responses.stories_played_id = stories_played.id
                WHERE questions.target_response <> responses.response
                    AND stories_played.participant = (?)
                    AND stories_played.session = (?)
                """, (participant, current_session)).fetchall()
            # The user may not have responded incorrectly to any
            # questions, in which case we get no results from the query.
            if result is None or result == []:
                self._logger.warn("Could not find any incorrect responses for "
                    + participant + " for session " + str(current_session-1)
                    + " in the database!")
                return []
            else:
                # Database gives us a list of tuples, so convert to a
                # a list before returning.
                return [emotion[0] for emotion in result]
        except Exception as e:
            self._logger.exception("Failed when trying to find incorrect "
                "responses for " + participant + " for session " +
                str(current_session-1) + " in the database!")
            # Pass on exception for now.
            raise


    def get_next_new_story(self, participant, emotions, level):
        """ Get the next unplayed story for the desired level from the
        story table with at least one of the listed emotions present in
        the story. If no unplayed story has the desired emotions or if
        there are no desired emotions, return the name of the next
        unplayed story.  If there are no more unplayed stories, return
        None.
        """
        try:
            # Parameters are the list of emotions, participant, session.
            # We have to put all the parameters into the same list so
            # we can give the query just one list. We can't supply a
            # list and a couple other things, because SQLite doesn't
            # know how to deal with a list parameter. So we add
            # correct number of ?'s into the query for the number of
            # emotions and supply a list with a matching number of
            # parameters.
            params = list(emotions)
            params.append(participant)
            params.append(level)
            params.append(participant)

            # The stories and questions tables should not be empty.
            # The stories_played table may be empty.
            # The first half of the query looks for a story with the
            # specified emotions; the second half looks for any unplayed
            # story, not caring about emotions.
            # 0 for the first query and 1 for the second query lets us
            # order the results by anything found from the first query (the
            # first half of the union) before anything from the second half.
            query1 = """
                SELECT stories.story_name, stories.id, 0 AS found_emotion
                FROM stories
                JOIN questions
                    ON questions.story_id = stories.id
                LEFT JOIN stories_played
                    ON stories_played.story_id = stories.id
                WHERE questions.target_response IN (%s)
                    AND (stories_played.participant <> (?)
                    OR stories_played.participant IS NULL)
                    AND questions.level = (?)
                """ % ",".join("?"*len(emotions))

            query2 = """
                SELECT stories.story_name, stories.id, 1 AS found_emotion
                FROM stories
                LEFT JOIN stories_played
                    ON stories_played.story_id = stories.id
                WHERE stories_played.participant <> (?)
                    OR stories_played.participant IS NULL
                """

            query = query1 + " UNION " + query2 + """
                ORDER BY found_emotion, stories.id
                LIMIT 1 """

            result = self._cursor.execute(query, params).fetchone()

            if result is None or result == []:
                self._logger.warn("Could not find any unplayed stories for "
                + participant + " in the database!")
                return None

            # We either found an unplayed story with the right emotions
            # or didn't, and found an unplayed story without them.
            # Return the name of a new story to play. The DB gives
            # us the name of the story in a tuple.
            self._logger.info("Found a story to play: " + str(result[0]))
            return result[0]

        except Exception as e:
            self._logger.exception("Failed when trying to find unplayed "
                "stories for " + participant + " with emotions " +
                str(emotions) + " in the database!")
            # Pass on exception for now.
            raise


    def get_next_review_story(self, participant, current_session, emotions,
            level):
        """ Get a review story with at least one of the listed emotions
        present in the story that wasn't played in the current session.
        If no played stories have the desired emotions, return the
        name of the story heard least recently and least often. If there
        are no stories we can review, return None.
        """
        try:
            # Parameters are the list of emotions, participant, session.
            # We have to put all the parameters into the same list so
            # we can give the query just one list. We can't supply a
            # list and a couple other things, because SQLite doesn't
            # know how to deal with a list parameter. So we add
            # correct number of ?'s into the query for the number of
            # emotions and supply a list with a matching number of
            # parameters.
            params = list(emotions)
            params.append(participant)
            params.append(current_session)
            params.append(level)

            # This gives us a randomly picked story from a list of
            # stories played not this session with at least one of the
            # desired emotions.
            # The stories and questions tables should not be empty.
            # The stories_played table may be empty.
            result = self._cursor.execute("""
                SELECT DISTINCT stories.story_name
                FROM stories
                JOIN questions
                    ON questions.story_id = stories.id
                JOIN stories_played
                    ON stories_played.story_id = stories.id
                WHERE questions.target_response IN (%s)
                AND stories_played.participant = (?)
                AND stories_played.session <> (?)
                AND questions.level = (?)
                ORDER BY RANDOM()
                LIMIT 1
                """ % ",".join("?"*len(emotions)), params).fetchone()

            if result is None:
                self._logger.warn("Could not find any stories to review for "
                    + participant + " for session " + str(current_session)
                    + " with emotions " + str(emotions) + " in the database!"
                    + " Looking for a story without those emotions...")

                # If no stories have the desired emotions to review,
                # find and return the story heard least recently and
                # least often. Sort results by stories heard least
                # often, then by the least recent. If there is a tie,
                # pick the least recent.
                result = self._cursor.execute("""
                    SELECT stories.story_name
                    FROM stories_played
                    JOIN stories
                        ON stories_played.story_id = stories.id
                    WHERE stories_played.participant = (?)
                    AND stories_played.session <> (?)
                    GROUP BY stories_played.story_id
                    ORDER BY count(stories_played.story_id) ASC,
                        max(stories_played.time) ASC
                    LIMIT 1
                    """, (participant, current_session)).fetchone()

                if result is None or result == []:
                    self._logger.warn("Could not find any review stories for "
                    + participant + " for session " + str(current_session)
                    + " in the database!")
                    return None

            # We found a review story with the right emotions, or
            # we didn't find any with the right emotions, so we
            # found the story told least often and least recently.
            # Return the name of a review story to play. The DB
            # gives us the name of the story in a tuple.
            return result[0]

        except Exception as e:
            self._logger.exception("Failed when trying to find stories to "
                "review for " + participant + " for session " +
                str(current_session) + " with emotions " + str(emotions) +
                " in the database!")
            # Pass on exception for now.
            raise


    def get_level_info(self, level):
        """ Get information about stories at this level: whether the
        scenes are presented in order or not, and how many answer
        options are shown when questions are asked.
        """
        try:
            result = self._cursor.execute("""
                SELECT num_answers, in_order
                FROM levels
                WHERE level = (?)
                """, (level,)).fetchone()
            if result is None:
                self._logger.warn("Could not find info for level " + str(level)
                        + " in the database!")
                return None
            else:
                # Database gives us a tuple, so return first element as
                # the number of answers and convert the second element
                # to a boolean.
                return result[0], (True if result[1] == 1 else False)
        except Exception as e:
            self._logger.exception("Failed when trying to find info for level "
                    + str(level) + " in the database!")
            # Pass on exception for now.
            raise


    def get_graphics(self, story, level):
        """ Get the list of names of graphics for the scenes in a story
        at the specified level.
        """
        try:
            result = self._cursor.execute("""
                SELECT graphic
                FROM graphics
                WHERE level = (?)
                AND story_id = (
                    SELECT id
                    FROM stories
                    WHERE story_name = (?))
                """, (level, story)).fetchall()
            if result is None or result == []:
                self._logger.warn("Could not find graphics for story " + story
                    + " at level " + str(level) + " in the database!")
                return None
            else:
                # Database gives us a list of tuples of graphic names,
                # so make this into a list of graphic names.
                return [name[0] for name in result]
        except Exception as e:
            self._logger.exception("Failed when trying to find graphics for "
                "story " + story + " at level " + str(level) +
                " in the database!")
            # Pass on exception for now.
            raise


    def record_story_played(self, participant, session, level, story):
        """ Insert the participant ID, session number, story level,
        current date and time, and a reference to the current story
        into the stories_played table.
        """
        try:
            self._cursor.execute("""
                INSERT INTO stories_played (participant, session,
                    story_id, level)
                VALUES (
                (?),
                (?),
                (SELECT id
                    FROM stories
                    WHERE story_name = (?)),
                (?))
                """, (participant, session, story, level))
            # Commit after recording the story.
            self._conn.commit()
            self._logger.debug("Recorded story played: participant=" +
                participant + ", session=" + str(session) + ", level=" +
                str(level) + ", story=" + story)
        except Exception as e:
            self._logger.exception("Could not insert record into stories_played"
                + " table in database! Tried to insert: participant=" +
                participant + ", session=" + str(session) + ", level=" +
                str(level) + ", story=" + story)
            # Pass on exception for now.
            raise


    def record_response(self, participant, session, level, story, question_num,
            question_type, response):
        """ Insert a user response into the responses table: we need
        the question ID, stories_played ID, and the actual response.
        """
        try:
            self._cursor.execute("""
                INSERT INTO responses (stories_played_id, questions_id,
                    response)
                VALUES (
                (SELECT id from stories_played
                    WHERE participant = (?)
                    AND session = (?)
                    AND level = (?)
                    AND story_id = (
                        SELECT id
                            FROM stories
                            WHERE story_name = (?))
                    ORDER BY time DESC
                    LIMIT 1),
                (SELECT id
                    FROM questions
                    WHERE question_num = (?)
                    AND question_type = (?)
                    AND level = (?)
                    AND story_id = (
                        SELECT id
                        FROM stories
                        WHERE story_name = (?))),
                (?))
                """, (participant, session, level, story, question_num,
                    question_type, level, story, response))
            # Commit after recording the response.
            self._conn.commit()
        except Exception as e:
            self._logger.exception("Could not insert record into questions"
                + " table in database! Tried to insert: participant=" +
                participant + ", session=" + str(session) + ", level=" +
                str(level) + ", story=" + story + ", question_num=" +
                str(question_num) + ", question_type=" + question_type +
                ", response=" + response)
            # Pass on exception for now.
            raise
