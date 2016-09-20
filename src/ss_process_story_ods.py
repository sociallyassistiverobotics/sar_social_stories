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

def ss_process_story_ods():
    """ Using the story info and scripts in the .ods spreadsheets,
    generate story script .txt files and fill the database with initial
    information about the stories, questions asked about stories, and
    graphics loaded for stories.
    """
    # Parse python arguments: A list of .ods files should be provided,
    # which will each be parsed for stories.
    parser = argparse.ArgumentParser(
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description="Read .ods spreadsheets containing story info for the"
            " SAR Social Stories game stories. Generate game scripts that will"
            " be used to load graphics and tell the robot how to read aloud"
            " the story. Add meta-information about the stories and the"
            " questions to ask about each story to the database.\nThis script"
            " will clear any existing data from the tables, so you should run"
            " it with all the spreadsheets at once -- if you run it again"
            " later, some or all of the previously imported data may be"
            " deleted.")
    parser.add_argument('-d', '--database', dest='db',
           action='store', nargs='?', type=str, default='socialstories.db',
           help= "The database filename for storing story and question info. "
           + "Defaults to \"socialstories.db\".")
    parser.add_argument('ods_files', action='store',
           nargs='+', type=str, help="""A list of .ods spreadsheets containing
           stories for the SAR Social Stories game.""")
    parser.add_argument('-o', '--output_dir', dest='out_dir', action='store',
            nargs='?', type=str, default="", help="""The output directory where
            generated story scripts will be saved. Defaults to the current
            directory.""")

    # Parse the args we got, and print them out.
    args = parser.parse_args()
    print("Args received: " + str(args))

    # Get connection to database.
    conn = sqlite3.connect(args.db)
    cursor = conn.cursor()

    # Reset any tables that shouldn't have duplicate data.
    cursor.execute("DELETE FROM questions")
    cursor.execute("DELETE FROM responses_in_question")
    cursor.execute("DELETE FROM graphics")
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
            question_list = {}
            midway_question_list = {}
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
                        insert_to_questions_table(cursor, sheet.name.lower(),
                            level+1, question_num, question_type,
                            # Target response is the first in the list
                            # of response options
                            sheet_dict[responses][level].split(',')[0]
                                .strip().lower())

                        # Add responses to emotions_in_question table
                        # at this level.
                        insert_to_responses_table(cursor, sheet.name.lower(),
                            level+1, question_num, question_type,
                            sheet_dict[responses][level].split(','))

                        # Make dict of level: question text, responses.
                        if level not in question_list.keys():
                            question_list[level] = []
                        if level not in midway_question_list.keys():
                            midway_question_list[level] = []
                        # Save theory of mind questions separately
                        # because they are asked midway through the
                        # story.
                        if "ToM" in question_type:
                            midway_question_list[level].append(
                                [sheet_dict[key][level],
                                sheet_dict[responses][level].split(',')])
                        else:
                            question_list[level].append(
                                [sheet_dict[key][level],
                                sheet_dict[responses][level].split(',')])

                # If this is a Scene column, add the graphics filenames
                # to the DB.
                if "scene" in key.lower() and "graphic" in key.lower():
                    # Get the number of the scene
                    try:
                        scene_num = int(re.findall(r'\d+', key)[0])
                    except Exception as e:
                        # If there is no number in the scene's label,
                        # that's a problem.
                        print("Error! No scene number found!\n" + str(e))
                        return
                    else:
                        # Graphics may have different descriptions at
                        # different levels, so we cannot automatically
                        # figure out which description matches which
                        # graphics tag. Instead, there are columns that
                        # list the tag of the graphic to load for that
                        # scene, for that level, for that story.
                        for level in range(0,10):
                            # Skip empty cells.
                            if (sheet[level,key] == "") \
                                or (sheet[level,key] == "-") \
                                or (sheet[level,key] == ["-"]):
                                print("Skipping empty cell")
                                continue
                            # Graphics scene numbers are 1-indexed.
                            insert_to_graphics_table(cursor,
                                sheet.name.lower(), level + 1, scene_num,
                                sheet[level,key].lower())

            # For each level, generate story.
            # Rows are 0-indexed but levels are 1-indexed.
            for level in range(0,10):
                # Use story to generate game script for robot
                generate_script_for_story(args.out_dir, sheet.name.lower(),
                        level+1, sheet[level, "Story"], question_list[level],
                        midway_question_list[level])

            # Commit after each story.
            conn.commit()

    # Close database connection.
    conn.close()


def insert_to_stories_table(cursor, story_names):
    """ Add a story to the stories table. """
    # story_name = The story's unique tag string.
    for name in story_names:
        try:
            cursor.execute("""
                INSERT INTO stories (story_name)
                VALUES (?)
                """, (name.lower(),))
        except sqlite3.IntegrityError as e:
            print("Error adding story " + name + " to DB! It may already "
                "exist. Exception: " + str(e))


def insert_to_graphics_table(cursor, story_name, level, scene, graphic_tag):
    """ Add a list of graphics names to the graphics table."""
    # story_id = The id from the stories table for this story.
    # level_id = The level number from the levels table for this level.
    # scene = Scene number (1,2,3,4) to load this graphic into.
    # graphic_tag = Tag of graphic to load (lowercase letter).
    print("ADD GRAPHIC: " + story_name + "-" + str(level) + " scene" +
            str(scene) + " " + graphic_tag)
    # Graphics file names:
    #     [env][story_num]-[background_type]-[tag].png
    #     e.g., LR1-B-a.png or CF1-P-f.png
    # Levels 1-5: tag P for plain background.
    # Levels 6-10: tag B for complex background.
    graphic_name = story_name.replace("Story-","") + "-" + \
        ("P" if level < 6 else "B") + "-" + graphic_tag + ".png"
    cursor.execute("""
        INSERT INTO graphics (story_id, level_id, scene_num, graphic)
        VALUES (
            (SELECT id FROM stories WHERE story_name=(?)),
            (SELECT level FROM levels WHERE level=(?)),
            (?),
            (?))
        """, (story_name, level, scene, graphic_name))


def insert_to_questions_table(cursor, story, level, question_num,
        question_type, target_response):
    """ Add a question to the questions table."""
    # story = Story this question belongs to
    # level = Level of the story (some levels have more questions)
    # question_num = Number of question in the story (1,2,3).
    # question_type = Emotion, order, or midway ToM question.
    # target_response = Correct answer (emotion or scene name).
    print("ADD QUESTION: " + story + "-" + str(level) + " " + question_type +
        " " + str(question_num) + ": " + target_response)
    cursor.execute("""
        INSERT INTO questions (story_id, level, question_num, question_type,
            target_response)
        VALUES (
            (SELECT id
                FROM stories
                WHERE story_name=(?)),
            (SELECT level
                FROM levels
                WHERE level=(?)),
            (?),
            (?),
            (?))
        """, (story, level, question_num, question_type, target_response))


def insert_to_responses_table(cursor, story, level, question_num,
        question_type, responses):
    """ Add a question-response pair to the responses table. """
    # question_id = The id of the question in the questions table.
    # emotion = Emotion string.
    print("ADD RESPONSES: " + story + "-" + str(level) + " " + question_type +
        " " + str(question_num) + ": " + str(responses))
    for response in responses:
        resp = response.strip().replace(" ", "")
        if (resp == ""):
            continue
        cursor.execute("""
            INSERT INTO responses_in_question (questions_id, response)
            VALUES (
                (SELECT id
                    FROM questions
                    WHERE level=(?)
                    AND question_num=(?)
                    AND question_type=(?)
                    AND story_id
                        IN (
                        SELECT id
                        FROM stories
                        WHERE story_name=(?))),
                (?))
            """, (level, question_num, question_type, story, resp))


def fill_levels_table(cursor):
    """ Initialize levels table. """
    # level = The level number.
    # num_answers = The number of answer options for questions asked
    # about the story this level.
    # in_order = Whether the scenes for stories at that level are shown
    # in order (1=True) or out of order (0=False).
    try:
        cursor.execute("""
            INSERT INTO levels (level, num_answers, in_order)
            VALUES
            ("1", "3", "1"),
            ("2", "3", "1"),
            ("3", "3", "1"),
            ("4", "3", "1"),
            ("5", "3", "0"),
            ("6", "4", "0"),
            ("7", "4", "0"),
            ("8", "4", "0"),
            ("9", "4", "0"),
            ("10", "5", "0"),
            ("11", "4", "0"),
            ("12", "4", "0")
            """)
    except sqlite3.IntegrityError as e:
        print("Error adding levels to DB! They may already exist. Exception: "
                + str(e))


def generate_script_for_story(output_dir, story_name, level, story, questions,
        midway_questions):
    """ Using the provided story text, generate a game script with the
    instructions for loading and playing the story with a robot.
    """
    print("Generating script for story: " + story_name + "-" + str(level))

    # Open file for game script for this story.
    with open(output_dir + story_name + "-" + str(level) + ".txt", "w+") as f:
        # Add story to script as one line for the robot to say.
        # We can't split on sentences because splitting by period may
        # make some quoted speech in the stories be split onto multiple
        # lines, since periods may be inside of the quotations...
        # We do need to split by scenes, using "//" as a delimiter, so
        # that we can send a "highlight scene" message to highlight the
        # scene corresponding to the current story text being read.
        if "//" in story:
            # There's a scene delimiter, so there is text for more than
            # one scene here.
            story = story.split("//")

            # Add each story segment with a scene highlight command.
            counter = 0
            for s in story:
                # Highlight current scene.
                f.write("OPAL\tHIGHLIGHT\tscene" + str(counter) + "\n")
                # Add story text.
                if "*" in s:
                    # There's a question to ask partway through the
                    # story, so we need to tell half the story, ask the
                    # question, then tell the second half.
                    s = s.split("*")
                    # Add first half of story.
                    f.write("ROBOT\tDO\t" + s[0].strip() + "\n")
                    # Add midway question
                    add_question_to_script(midway_questions[0], f)
                    # Add second half of story.
                    f.write("ROBOT\tDO\t" + s[1].strip() + "\n")
                else:
                    # There's no question to ask partway through this
                    # story segment.
                    f.write("ROBOT\tDO\t" + s.strip() + "\n")
                counter += 1

        # Or, there's only text for one scene in this story.
        else:
            # Highlight current scene.
            f.write("OPAL\tHIGHLIGHT\tscene0\n")
            f.write("ROBOT\tDO\t" + story.strip() + "\n")

        # Add "The end" and a pause.
        f.write("ROBOT\tDO\tThe end.\n"
            + "PAUSE\t2\n")

        # Then add questions!
        for question in questions:
            add_question_to_script(question, f)


def find_character(words):
    """ Find the name of the character that the question is about. """
    # The name of the character that this question is about is the
    # second capitalized word in the sentence.
    first = True
    for word in words:
        if not word.islower():
            # Has at least one uppercase letter.
            if first:
                first = False
            else:
                return word.lower()


def add_question_to_script(question, outfile):
    """ Add a question to a game script. """
    # Find the character this question is about.
    # The question provided has two parts:
    #   [0] = the question text
    #   [1] = the comma-separated list of responses
    character = find_character(question[0].split())

    # Make a string so we can deal with commas.
    s = ""

    # We only want to load character faces as answers if the question is
    # an emotion or ToM question about a character.
    if "scene" not in question[1][0]:
        # Load answers line.
        outfile.write("OPAL\tLOAD_ANSWERS\t")
        for response in question[1]:
            s += "answers/" + character + "_" + response.lower().strip() \
                + ".png, "
        # Remove last comma before adding ending punctuation and
        # writing the rest of the line to the file.
        outfile.write(s[:-2] + "\n")

        # Set correct line.
        outfile.write("OPAL\tSET_CORRECT\t{\"correct\":[\"" + character + "_"
                + question[1][0].lower().strip() + "\"], \"incorrect\":[")
        # Make a string so we can deal with commas.
        s = ""
        for i in range (1, len(question[1])):
            s += "\"" + character + "_" + question[1][i].lower().strip() + "\","
        # Remove last comma before adding ending punctuation and
        # writing the rest of the line to the file.
        outfile.write(s[:-1] + "]}" + "\n")

    # For order questions, we don't need to load answers -- we will use
    # the scenes as the answer slots. So we will just need to set the
    # scenes as correct or incorrect.
    else:
        # Set correct line. Scene slots in the game are 0-indexed, but
        # in the story scripts, they are 1-indexed, so we need to
        # convert them.
        responses_0indexed = []
        for response in question[1]:
            try:
                num = re.findall(r'\d+', response)[0]
                responses_0indexed.append("scene" + str(int(num)-1))
            except:
                # If there is no number, we have a problem. Order
                # questions should always have numbered scene responses.
                print("No scene number found in question responses!")
                raise

        # Now that we have 0-indexed responses, build a line to set
        # the correct and incorrect responses.
        outfile.write("OPAL\tSET_CORRECT\t{\"correct\":[\""
            + responses_0indexed[0]
            + "\"], \"incorrect\":[")
        # Make a string so we can deal with commas.
        s = ""
        for i in range (1, len(responses_0indexed)):
            s += "\"" + responses_0indexed[i] + "\","

        # Remove last comma before adding ending punctuation and
        # writing the rest of the line to the file.
        outfile.write(s[:-1] + "]}" + "\n")

    # Robot will say the question text next.
    outfile.write("ROBOT\tDO\t" + question[0] + "\n")

    # Add wait line.
    outfile.write("WAIT\tCORRECT_INCORRECT\t10\n")

    # Robot will say the answer, but only for ToM and emotion questions.
    if "scene" not in question[1][0]:
        outfile.write("ROBOT\tDO\t" + (character[0].upper() + character[1:])
                + " felt " + question[1][0].lower() + " <"
                + question[1][0].lower() + ">.\n")

    # Add clear and pause lines.
    outfile.write("OPAL\tCLEAR\tANSWERS\n"
        + "PAUSE\t1\n")


if __name__ == '__main__':
    ss_process_story_ods()
