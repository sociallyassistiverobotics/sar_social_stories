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
import argparse # to parse command line arguments
import pyexcel # for reading in .ods spreadsheets
from collections import OrderedDict # spreadsheets read into OrderedDicts

def gen_stories_from_ods():
    """ Using the story info and scripts in the .ods spreadsheets,
    generate story script .txt files and fill the database with initial
    information about the stories, questions asked about stories, and
    graphics loaded for stories.
    """
    # Parse python arguments: A list of .ods files should be provided,
    # which will each be parsed for stories.
    parser = argparse.ArgumentParser(
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description="""Read .ods spreadsheets containing story info for the
            SAR Social Stories game stories. Generate game scripts that will be
            used to load graphics and tell the robot how to read aloud the
            story. Add meta-information about the stories and the questions to
            ask about each story to the database.""")
    parser.add_argument('-d', '--database', dest='db',
           action='store', nargs='?', type=str, default='socialstories.db',
           help= 'The database filename for storing story and question info.')
    parser.add_argument('ods_files', action='store',
           nargs='+', type=str, help="""A list of .ods spreadsheets containing
           stories for the SAR Social Stories game.""")

    # Parse the args we got, and print them out.
    args = parser.parse_args()
    print("Args received: " + str(args))

    # Get connection to database.
    conn = sqlite3.connect(args.db)
    cursor = conn.cursor()

    # Fill levels table since it doesn't depend on the spreadsheets.
    fill_levels_table(conn)

    # For each spreadsheet file, go through sheets and read in stories.
    for ods in args.ods_files:
        print("Processing file: " + ods)

        # Open file and get data.
        book = pyexcel.get_book(file_name=ods, name_columns_by_row=0)
        
        # Print out general info about the spreadsheet.
        print("Found " + str(book.number_of_sheets()) + " sheets.")

        # Add list of story names to story table (from sheet names).
        insert_to_stories_table(conn, book.sheet_names())

        # For each sheet, read in story, generate scripts, input info to db
        for sheet in book:
            print("Processing sheet: " + sheet.name)
            sheet.name_columns_by_row(0)
            print("Has columns: " + str(sheet.colnames))
            # TODO parse sheet

    # Close database connection.
    conn.close()


def insert_to_stories_table(conn, story_names):    
    """ Add a story to the stories table. """
    # story_name = The story's unique tag string.
    cursor = conn.cursor()
    for name in story_names:
        try:
            cursor.execute("INSERT INTO stories (story_name) VALUES (?)",
                (name,)) 
        except sqlite3.IntegrityError as e:
            print("Error adding story " + name + " to db! It may already "
                "exist. Exception: " + str(e))
    conn.commit()


def insert_to_graphics_table(conn, story_name, level, scene, graphic):
    """ Add a list of graphics names to the graphics table."""
    # story_id = The id from the stories table for this story.
    # level_id = The level number from the levels table for this level.
    # scene = Scene number (1,2,3,4).
    # graphic = Filename of graphic to load for this scene.
    cursor = conn.cursor()
    cursor.execute('''INSERT INTO graphics
            (stories_id, level_id, scene, graphic) VALUES (
            (SELECT id from stories WHERE type=(?)),
            (SELECT level from levels WHERE type=(?)),
            (?),
            (?))''',
            (story_name,), (level,), (scene,), (graphic,))
    conn.commit()


def insert_to_questions_table(conn):
    """ Add a question to the questions table."""
    # question_num = Number of question in the story (1,2,3).
    # question_type = Emotion, order, or midway ToM question. 
    # target_response = Correct answer (emotion or scene name).
    cursor = conn.cursor()
    # TODO fill in command -- example: INSERT INTO questions (stories_id, question_num, question_type, target_response) VALUES ((SELECT id from stories WHERE type="test1"), "1", "emotion", "angry");
    conn.commit()


def insert_to_emotions_table(conn):
    """ Add a question-emotion pair to the emotions_in_question table. """
    # question_id = The id of the question in the questions table.
    # emotion = Emotion string.
    cursor = conn.cursor()
    #cursor.execute('''INSERT INTO emotions_in_question (question_id, emotion) VALUES''', question, emotion) #TODO
    conn.commit()


def fill_levels_table(conn):
    """ Initialize levels table. """
    # level = The level number.
    # num_scenes = The number of scenes in the story at this level.
    # in_order = Whether the scenes for stories at that level are shown
    # in order (1=True) or out of order (0=False).
    try:
        cursor = conn.cursor()
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
            ("12", "4", "0")
            ''')
        conn.commit()
    except sqlite3.IntegrityError as e:
        print("Error adding levels to db! They may already exist. Exception: "
                + str(e))


if __name__ == '__main__':
    gen_stories_from_ods()
