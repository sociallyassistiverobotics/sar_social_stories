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

def ss_fill_db():
    """ Insert initial data about stories, story questions, emotions in
    the questions, and story graphics into tables.
    """
    # get connection to database
    conn = sqlite3.connect("socialstories.db")
    cursor = conn.cursor()
    
    # Initialize stories table.
    # story_name = The story's unique tag string.
    cursor.execute('''INSERT INTO stories (story_name) VALUES
        (""),
        ''') # TODO fill in values

    # Initialize graphics table.
    # TODO

    # Initialize questions table.
    # question_num = Number of question in the story (1,2,3).
    # question_type = Emotion, order, or midway ToM question. 
    # target_response = Correct answer (emotion or scene name).
    
    # TODO fill in command -- example: INSERT INTO questions (stories_id, question_num, question_type, target_response) VALUES ((SELECT id from stories WHERE type="test1"), "1", "emotion", "angry");

    # Initialize emotions_in_question table.
    # TODO


    # Initialize levels table.
    # level = The level number.
    # num_scenes = The number of scenes in the story at this level.
    # in_order = Whether the scenes for stories at that level are shown
    # in order (1=True) or out of order (0=False).
    cursor.execute('''INSERT INTO levels (level, num_scenes, in_order) VALUES
        ("1", "1", "1"),
        ("2", "2", "1"),
        ("3", "3", "1"),
        ("4", "4", "1"),
        ("5", "4", "0"),
        ("6", "4", "0"),
        ("7", "4", "0"),
        ("8", "4", "0"),
        ("9", "4", "0"),
        ("10", "4", "0"),
        ("11", "4", "0"),
        ("12", "4", "0"),
        ''')

    conn.commit()
    conn.close()


if __name__ == '__main__':
    ss_init_db()
