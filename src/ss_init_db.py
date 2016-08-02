#!/usr/bin/env python

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

import sqlite3 # store game info and personalization

def ss_init_db():
    """ Initalize database with tables for tracking question responses
    in the social stories game.
    """
    # Get connection to database.
    # TODO Make the name of the database an argument to this script!
    conn = sqlite3.connect("socialstories.db")
    cursor = conn.cursor()

    # Create tables.
    # The STORIES table holds the story names.
    cursor.execute(''' CREATE TABLE stories (
            id       integer     PRIMARY KEY,
            story_name  text        NOT NULL   UNIQUE
            )''')

    # The LEVELS table lists the number of answer options shown for
    # questions asked for stories at a level and whether the graphics
    # for the story are shown in order or not at that level.
    cursor.execute(''' CREATE TABLE levels (
            level       integer     PRIMARY KEY,
            num_answers integer     NOT NULL,
            in_order    integer     NOT NULL
            )''')

    # The GRAPHICS table lists the filenames of the graphics to load
    # for each scene in a story. Since stories at lower levels have
    # fewer scenes, sometimes the graphic shown at scene 1 for a lower
    # level story is not the saem graphic shown at scene 1 for a higher
    # level story ... thus the need to list all this separately for
    # each level.
    cursor.execute(''' CREATE TABLE graphics (
            story_id    integer     NOT NULL,
            level_id    integer     NOT NULL,
            scene_num   integer     NOT NULL,
            graphic     text        NOT NULL,
            FOREIGN KEY(story_id) REFERENCES stories(id),
            FOREIGN KEY(level_id) REFERENCES levels(level)
            )''')

    # The QUESTIONS table lists meta-information about questions that
    # are asked about a story. This includes the type of the question
    # (e.g., whether it is about emotions, story order, or theory of
    # mind) and the number of the question (e.g., question 2 about
    # emotions for this story). More questions are asked about higher
    # level stories, so we list all of this by level.
    cursor.execute('''CREATE TABLE questions (
            id              integer PRIMARY KEY,
            story_id        integer NOT NULL,
            question_num    integer NOT NULL,
            question_type   text    NOT NULL,
            target_response text    NOT NULL,
            level           integer NOT NULL,
            FOREIGN KEY(story_id) REFERENCES stories(id),
            FOREIGN KEY(level) REFERENCES levels(level)
            )''')

    # The RESPONSES_IN_QUESTION table lists the answer options that
    # will be shown for a given question.
    cursor.execute('''CREATE TABLE responses_in_question (
            questions_id    integer NOT NULL,
            response        text    NOT NULL,
            FOREIGN KEY(questions_id) REFERENCES questions(id)
            )''')

    # The RESPONSES table will track user responses to questions, which
    # we can use to determine whether they got questions correct or
    # incorrect.
    cursor.execute('''CREATE TABLE responses (
            id                  integer PRIMARY KEY,
            stories_played_id   integer NOT NULL,
            questions_id        integer NOT NULL,
            response            text,
            FOREIGN KEY(stories_played_id) REFERENCES stories_played(id),
            FOREIGN KEY(questions_id) REFERENCES questions(id)
            )''')

    # The STORIES_PLAYED table tracks which stories a participant has
    # played and when. This can be used to determine which stories are
    # new for that participant or to select personalized stories.
    cursor.execute('''CREATE TABLE stories_played (
            id          integer     PRIMARY KEY,
            date        date        NOT NULL    default current_date,
            time        timestamp   NOT NULL    default current_timestamp,
            participant text        NOT NULL,
            session     integer     NOT NULL,
            level_id    integer     NOT NULL,
            story_id    text        NOT NULL,
            FOREIGN KEY(story_id) REFERENCES stories(id),
            FOREIGN KEY(level_id) REFERENCES levels(level)
            )''')

    conn.commit()
    conn.close()


if __name__ == '__main__':
    ss_init_db()
