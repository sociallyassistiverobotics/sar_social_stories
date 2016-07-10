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
    # get connection to database
    conn = sqlite3.connect("socialstories.db")
    cursor = conn.cursor()

    # create tables
    cursor.execute('''CREATE TABLE stories_played (
            id          integer     PRIMARY KEY,
            date        date        NOT NULL    default current_date,
            time        timestamp   NOT NULL    default current_timestamp,
            participant text        NOT NULL,
            session     integer     NOT NULL,
            level       integer     NOT NULL,
            story       text        NOT NULL
            )''')

    cursor.execute('''CREATE TABLE questions (
            stories_played_id   integer NOT NULL,
            id                  integer PRIMARY KEY,
            question_num        integer NOT NULL,
            question_type       text    NOT NULL,
            target_emotion      text    NOT NULL,
            response            text,
            FOREIGN KEY(stories_played_id) REFERENCES stories_played(id)
            )''')

    cursor.execute('''CREATE TABLE emotions_in_question (
            questions_id    integer NOT NULL,
            emotion         text    NOT NULL,
            FOREIGN KEY(questions_id) REFERENCES questions(id)
            )''')
    
    conn.commit()
    conn.close()


if __name__ == '__main__':
    ss_init_db()
