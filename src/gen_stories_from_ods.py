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
import re # regex for parsing data in spreadsheet cells

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

    # Reset any tables that shouldn't have duplicate data.
    cursor.execute("DELETE FROM questions")
    cursor.execute("DELETE FROM responses_in_question")
    cursor.execute("VACUUM")

    # Fill levels table since it doesn't depend on the spreadsheets.
    fill_levels_table(cursor)

    # For each spreadsheet file, go through sheets and read in stories.
    for ods in args.ods_files:
        print("Processing file: " + ods)

        # Open file and get data.
        book = pyexcel.get_book(file_name=ods, name_columns_by_row=0)
        
        # Print out general info about the spreadsheet.
        print("Found " + str(book.number_of_sheets()) + " sheets.")

        # Add list of story names to story table (from sheet names).
        insert_to_stories_table(cursor, book.sheet_names())

        # For each sheet, read in story, generate scripts, input info
        # about stories, questions, and emotions to DB.
        for sheet in book:
            print("Processing sheet: " + sheet.name)
            sheet.name_columns_by_row(0)
            print("Has columns: " + str(sheet.colnames))
           
            sheet_dict = sheet.to_dict()
            graphics_list = [""] * 4
            scene_keys = [""] * 4
            # Add each question to the DB. Loop through columns.
            for key in sheet_dict.keys():
                # For each question column, get question text without
                # the answer list.
                if "question" in key.lower() and not "correct" in key.lower():
                    print("Adding question: " + key)
                    # Get the number of the question, if it has one
                    try:
                        question_num = re.findall(r'\d+', key)[0]
                    except:
                        # If there is no number in the question's label,
                        # there is probably only one such question, so
                        # label it question 1.
                        question_num = 1
                    # Get the type of question
                    if "midway" in key.lower():
                        question_type = "ToM"
                    elif "order" in key.lower():
                        question_type = "order"
                    else:
                        question_type = "emotion"

                    # Find responses column for the question by looping
                    # through the keys and finding the one that matches
                    # the question we're on.
                    responses = None
                    for k in sheet_dict.keys():
                        if key.lower() in k.lower() and "correct" in k.lower():
                            responses = k
                            break

                    if (responses is None):
                        print("Error! Did not find question responses.")
                        return

                    for level in range(0,10):
                        # Skip empty cells.
                        if (sheet_dict[responses][level] == "") \
                                or (sheet_dict[responses][level] == "-") \
                                or (sheet_dict[responses][level] == ["-"]):
                            print("Skipping empty cell")
                            continue

                        # Add question to questions table at this level.
                        insert_to_questions_table(cursor, sheet.name, level+1,
                            question_num, question_type,
                            # Target response is the first in the list
                            # of response options
                            sheet_dict[responses][level].split(',')[0].strip())

                        # Add responses to emotions_in_question table
                        # at this level
                        insert_to_responses_table(cursor, sheet.name, level+1,
                            question_num, question_type,
                            sheet_dict[responses][level].split(','))

                # If this is a Scene column, collect the graphics
                # descriptions at level 10 (0-index).
                if "scene" in key.lower() and "graphic" in key.lower():
                    # Get the number of the scene
                    try:
                        scene_num = int(re.findall(r'\d+', key)[0])
                        graphics_list[scene_num-1] = sheet_dict[key][9].lower()
                        scene_keys[scene_num-1] = key

                        # TODO some descriptions are different at different
                        # levels, so we have to list the graphics in the ods
                        # spreadsheets instead of automatically figuring
                        # them out...
                        #insert_to_graphics_table(cursor, sheet.name, level,
                        #    scene_num + 1, sheet[level,key].lower())

                    except Exception as e:
                        # If there is no number in the scene's label,
                        # that's a problem.
                        print("Error! No scene number found!\n" + str(e))
                        return

            # Add graphics filenames to DB.
            # At each level, the graphic to load in the scene1 slot is
            # the index of the level 10 graphic description that matches
            # the description in that level's scene1 slot.
            # If no match, print an error!
            # Graphics file names:
            #     [env][story_num]-[background_type]-[scene_num].png
            #     e.g., LR1-B-1.png or CF1-P-4.png
            # Levels 1-5: tag P for plain background.
            # Levels 6-10: tag B for complex background.
            for level in range(0,10):
                for key in scene_keys:
                    try:
                        # Find index of graphic description text at the
                        # level for graphic number; check scene_keys to
                        # get index of scene to load graphic into (both
                        # are 1-indexed).
                        insert_to_graphics_table(cursor, sheet.name, level,
                            scene_keys.index(key)+1,
                            graphics_list.index(sheet[level,key].lower())+1)
                    except:
                        print("Error! Could not find scene number matching "
                            + "the graphics description! " + sheet.name +
                            " " + str(level) + " " + str(key))
                        return

            # For each level, generate story.
            # Rows are 0-indexed but levels are 1-indexed.
            for level in range(0,10):
                # Use story to generate game script for robot
                generate_script_for_story(sheet.name, level+1,
                        sheet[level, "Story"])
                #TODO also give question info to story gen function!

            # Commit after each story
            conn.commit()

    # Close database connection.
    conn.close()


def insert_to_stories_table(cursor, story_names):    
    """ Add a story to the stories table. """
    # story_name = The story's unique tag string.
    for name in story_names:
        try:
            cursor.execute("INSERT INTO stories (story_name) VALUES (?)",
                (name,)) 
        except sqlite3.IntegrityError as e:
            print("Error adding story " + name + " to DB! It may already "
                "exist. Exception: " + str(e))


def insert_to_graphics_table(cursor, story_name, level, scene, graphic_num):
    """ Add a list of graphics names to the graphics table."""
    # story_id = The id from the stories table for this story.
    # level_id = The level number from the levels table for this level.
    # scene = Scene number (1,2,3,4) to load this graphic into.
    # graphic_num = Number of graphic to load for the scene (1,2,3,4).
    print("ADD GRAPHIC: " + story_name + "-" + str(level) + " scene" +
            str(scene) + " graphic" + graphic_num)
    graphic_name = story_name.replace("Story-","") + "-" + \
        ("P" if level < 6 else "B") + "-" + str(graphic_num) + ".png"
    cursor.execute('''INSERT INTO graphics
            (stories_id, level_id, scene, graphic) VALUES (
            (SELECT id FROM stories WHERE type=(?)),
            (SELECT level FROM levels WHERE type=(?)),
            (?),
            (?))''',
            (story_name, level, scene, graphic_name))


def insert_to_questions_table(cursor, story, level, question_num, question_type,
        target_response):
    """ Add a question to the questions table."""
    # story = Story this question belongs to
    # level = Level of the story (some levels have more questions)
    # question_num = Number of question in the story (1,2,3).
    # question_type = Emotion, order, or midway ToM question. 
    # target_response = Correct answer (emotion or scene name).
    print("ADD QUESTION: " + story + "-" + str(level) + " " + question_type +
        " " + str(question_num) + ": " + target_response)
    cursor.execute('''INSERT INTO questions (stories_id, level, question_num,
        question_type, target_response) VALUES (
        (SELECT id FROM stories WHERE story_name=(?)),
        (SELECT level FROM levels WHERE level=(?)),
        (?),
        (?),
        (?))''',
        (story, level, question_num, question_type, target_response))


def insert_to_responses_table(cursor, story, level, question_num, question_type,
        responses):
    """ Add a question-response pair to the responses table. """
    # question_id = The id of the question in the questions table.
    # emotion = Emotion string.
    print("ADD RESPONSES: " + story + "-" + str(level) + " " + question_type +
        " " + str(question_num) + ": " + str(responses))
    for response in responses:
        resp = response.strip().replace(" ", "")
        if (resp == ""):
            continue
        cursor.execute('''INSERT INTO responses_in_question (questions_id,
            response) VALUES (
            (SELECT id FROM questions WHERE level=(?) and question_num=(?) and
            question_type=(?) and stories_id IN (SELECT id FROM stories WHERE
            story_name=(?))),
            (?))''',
            (level, question_num, question_type, story, resp))


def fill_levels_table(cursor):
    """ Initialize levels table. """
    # level = The level number.
    # num_scenes = The number of scenes in the story at this level.
    # in_order = Whether the scenes for stories at that level are shown
    # in order (1=True) or out of order (0=False).
    try:
        cursor.execute('''INSERT INTO levels (level, num_scenes, in_order)
            VALUES
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
    except sqlite3.IntegrityError as e:
        print("Error adding levels to DB! They may already exist. Exception: "
                + str(e))


def generate_script_for_story(story_name, level, story):
    """ Using the provided story text, generate a game script with the
    instructions for loading and playing the story with a robot.
    """
    #TODO
    print("TODO: Generate game script for story: " + story_name + "-" +
        str(level))


if __name__ == '__main__':
    gen_stories_from_ods()
