# Jacqueline Kory Westlund
# September 2016
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
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import unittest
import mock
import random
from mock import Mock
from ss_db_manager import ss_db_manager

class test_db_manager(unittest.TestCase):

    def setUp(self):
        self.dbm = ss_db_manager("ss_test_no_participant_data.db")
        #self.dbm2 = ss_db_manager("ss_test_with_participant_data.db")
        #TODO add second database with participant data


    def test_get_most_recent_level(self):
        # Test different participant and session values.
        # If there is no participant data, we should always get None.
        self.assertEqual(self.dbm.get_most_recent_level("P001", 1), None)
        self.assertEqual(self.dbm.get_most_recent_level("P001", -5), None)
        self.assertEqual(self.dbm.get_most_recent_level("P001", 0), None)
        self.assertEqual(self.dbm.get_most_recent_level("P001", 7), None)
        self.assertEqual(self.dbm.get_most_recent_level("P001", 0.7), None)
        self.assertEqual(self.dbm.get_most_recent_level("aaaa", 2), None)
        self.assertEqual(self.dbm.get_most_recent_level("0928u4ijos", 2), None)
        self.assertEqual(self.dbm.get_most_recent_level("", 2), None)
        self.assertEqual(self.dbm.get_most_recent_level("0xa7", 2), None)


    def test_get_percent_correct_responses(self):
        # If there is no participant data, we should always get None.
        self.assertIsNone(self.dbm.get_percent_correct_responses("p234", 2))
        self.assertIsNone(self.dbm.get_percent_correct_responses("93", 1))
        self.assertIsNone(self.dbm.get_percent_correct_responses("93", 0.1))
        self.assertIsNone(self.dbm.get_percent_correct_responses("1", 0))
        self.assertIsNone(self.dbm.get_percent_correct_responses("", 0))
        self.assertIsNone(self.dbm.get_percent_correct_responses("p234", 2,
            "order"))
        self.assertIsNone(self.dbm.get_percent_correct_responses("p234", 2,
            "emotion"))
        self.assertIsNone(self.dbm.get_percent_correct_responses("p234", 2,
            "ToM"))


    def test_get_most_recent_incorrect_emotions(self):
        # If there is no participant data, we should get an empty list.
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("p134", 4), [])
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("1", 0), [])
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("d81", -33), [])
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("d81", -1), [])
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("p134", 1), [])
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("0xa7", 4), [])
        self.assertEqual(
            self.dbm.get_most_recent_incorrect_emotions("", 4), [])


    def test_get_next_new_story(self):
        # If there is no participant data, we should get the name of an
        # unplayed story with the relevant emotions at that level. Some
        # emotions are only present at higher levels, so for these, at
        # lower levels, we expect to get whichever story is first in
        # stories table.
        #
        # Note that these tests assume that all 42 SAR stories have
        # been imported into the database, and that the ods sheets were
        # imported in alphabetical order. A more general version of the
        # tests here could merely assert that we get back a string, or
        # could provide a list of all the story names, and assert that
        # the string we get back is one of the story names. However,
        # that would not test whether the story included the correct
        # emotions for the story's level, since not all stories have
        # the same emotions present at every level.
        self.assertEqual(self.dbm.get_next_new_story("p391", ["angry"], 1),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("", ["angry"], 10),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("", ["angry"], 22),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("p391", ["sad"], 2),
            "story-cr1")
        self.assertEqual(self.dbm.get_next_new_story("33", ["sad"], 10),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("p391", ["happy"], 3),
            "story-am1")
        self.assertEqual(self.dbm.get_next_new_story("33", ["happy"], 8),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("3811", ["nervous"],
            8), "story-am2")
        self.assertEqual(self.dbm.get_next_new_story("0x7a", ["nervous"],
            3), "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("", ["excited"], 9),
            "story-fo2")
        self.assertEqual(self.dbm.get_next_new_story("P001", ["excited"],
            4), "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("002", ["guilty"], 10),
            "story-am1")
        self.assertEqual(self.dbm.get_next_new_story("0.03a", ["guilty"], 1),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("0", ["surprised"], 7),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("P01", ["surprised"],
            6), "story-sr1")
        self.assertEqual(self.dbm.get_next_new_story("0", ["afraid"], 5),
            "story-fo2")
        self.assertEqual(self.dbm.get_next_new_story("P01", ["afraid"], 10),
            "story-fo2")
        self.assertEqual(self.dbm.get_next_new_story("0", ["frustrated"],
            8), "story-cr1")
        self.assertEqual(self.dbm.get_next_new_story("P01", ["frustrated"],
            3), "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("0", ["calm"], 10),
            "story-st1")
        self.assertEqual(self.dbm.get_next_new_story("P01", ["calm"], 1),
            "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("0", ["bored"], 8),
            "story-sr2")
        self.assertEqual(self.dbm.get_next_new_story("P01", ["bored"], 3),
            "story-fo1")

        self.assertEqual(self.dbm.get_next_new_story("0", ["frustrated",
            "bored", "happy"], 8), "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("0", ["frustrated",
            "surprised", "sad"], 8), "story-fo1")
        self.assertEqual(self.dbm.get_next_new_story("p391", ["happy",
            "frustrated", "afraid", "sad"], 1), "story-fo2")
        self.assertEqual(self.dbm.get_next_new_story("p391", [""], 1),
            "story-fo1")


    def test_get_next_review_story(self):
        # If there is no participant data, there will be no stories to
        # review, since none have been recorded as played yet.
        self.assertIsNone(self.dbm.get_next_review_story("p391", 1, ["angry"],
            1))
        self.assertIsNone(self.dbm.get_next_review_story("", 2, ["angry"], 2))
        self.assertIsNone(self.dbm.get_next_review_story("", 2, ["angry"], 22))
        self.assertIsNone(self.dbm.get_next_review_story("p391", 1, ["sad"],
            2))
        self.assertIsNone(self.dbm.get_next_review_story("33", 10, ["sad"],
            10))
        self.assertIsNone(self.dbm.get_next_review_story("p391", 1, ["happy"],
            3))
        self.assertIsNone(self.dbm.get_next_review_story("33", 2, ["happy"],
            8))
        self.assertIsNone(self.dbm.get_next_review_story("3811", 13,
            ["nervous"], 8))
        self.assertIsNone(self.dbm.get_next_review_story("0x7a", -0.1,
            ["nervous"], 3))
        self.assertIsNone(self.dbm.get_next_review_story("", 55, ["excited"],
            9))
        self.assertIsNone(self.dbm.get_next_review_story("P001", -22,
            ["excited"], 4))
        self.assertIsNone(self.dbm.get_next_review_story("002", 0, ["guilty"],
            10))
        self.assertIsNone(self.dbm.get_next_review_story("0.03a", 9,
            ["guilty"], 1))
        self.assertIsNone(self.dbm.get_next_review_story("0", 8, ["surprised"],
            7))
        self.assertIsNone(self.dbm.get_next_review_story("P01", 2,
            ["surprised"], 6))
        self.assertIsNone(self.dbm.get_next_review_story("0", 5, ["afraid"],
            5))
        self.assertIsNone(self.dbm.get_next_review_story("P01", 2, ["afraid"],
            10))
        self.assertIsNone(self.dbm.get_next_review_story("0", 9,
            ["frustrated"], 8))
        self.assertIsNone(self.dbm.get_next_review_story("P01", 2,
            ["frustrated"], 3))
        self.assertIsNone(self.dbm.get_next_review_story("0", 5, ["calm"],
            10))
        self.assertIsNone(self.dbm.get_next_review_story("P01", 2, ["calm"],
            1))
        self.assertIsNone(self.dbm.get_next_review_story("0", 5, ["bored"], 8))
        self.assertIsNone(self.dbm.get_next_review_story("P01", 2, ["bored"],
            3))

        self.assertIsNone(self.dbm.get_next_review_story("0", 9, ["frustrated",
            "bored", "happy"], 8))
        self.assertIsNone(self.dbm.get_next_review_story("0", 9, ["frustrated",
            "surprised", "sad"], 8))
        self.assertIsNone(self.dbm.get_next_review_story("p391", 1, ["happy",
            "frustrated", "afraid", "sad"], 1))
        self.assertIsNone(self.dbm.get_next_review_story("p391", 2, [""], 1))


    def test_get_level_info(self):
        # If a level doesn't exist, we should get None.
        self.assertIsNone(self.dbm.get_level_info(0))
        self.assertIsNone(self.dbm.get_level_info(0.04))
        self.assertIsNone(self.dbm.get_level_info(14))
        self.assertIsNone(self.dbm.get_level_info(-5))
        self.assertIsNone(self.dbm.get_level_info(50333330))

        # If a level does exist, we get its number of answers and if it
        # should be shown in order.
        ans, order = self.dbm.get_level_info(1)
        self.assertEqual(ans, 3)
        self.assertEqual(order, 1)

        ans, order = self.dbm.get_level_info(2)
        self.assertEqual(ans, 3)
        self.assertEqual(order, 1)

        ans, order = self.dbm.get_level_info(5)
        self.assertEqual(ans, 3)
        self.assertEqual(order, 0)

        ans, order = self.dbm.get_level_info(10)
        self.assertEqual(ans, 5)
        self.assertEqual(order, 0)

        ans, order = self.dbm.get_level_info(7)
        self.assertEqual(ans, 4)
        self.assertEqual(order, 0)


    def test_get_graphics(self):
        # If the story requested or the level requested doesn't exist,
        # we should get None.
        self.assertIsNone(self.dbm.get_graphics("-3-139uadfbio", 1))
        self.assertIsNone(self.dbm.get_graphics("dioufad-story", 4))
        self.assertIsNone(self.dbm.get_graphics("story-bb3", 9))
        self.assertIsNone(self.dbm.get_graphics("STORY-FO1", 10))
        self.assertIsNone(self.dbm.get_graphics("", 10))
        self.assertIsNone(self.dbm.get_graphics("story-fo1", 11))
        self.assertIsNone(self.dbm.get_graphics("story-cl2", -5))
        self.assertIsNone(self.dbm.get_graphics("story-fo3", 50))
        self.assertIsNone(self.dbm.get_graphics("story-sp2", 0))

        # If a story exists at the level, we get a list of graphics
        # file names. Note that these tests assume that all 42 SAR
        # stories have been imported into the database.
        self.assertIsInstance(self.dbm.get_graphics("story-fo1", 1), list)
        self.assertIsInstance(self.dbm.get_graphics("story-sr2", 10), list)
        self.assertIsInstance(self.dbm.get_graphics("story-fo2", 3), list)
        self.assertIsInstance(self.dbm.get_graphics("story-ki2", 4), list)

        self.assertListEqual(self.dbm.get_graphics("story-fo1", 1),
                ["story-fo1-P-a.png"])
        self.assertListEqual(self.dbm.get_graphics("story-fo1", 2),
                ["story-fo1-P-a.png", "story-fo1-P-b.png"])
        self.assertListEqual(self.dbm.get_graphics("story-fo1", 8),
                ["story-fo1-B-a.png", "story-fo1-B-b.png",
                "story-fo1-B-c.png", "story-fo1-B-d.png"])


    def test_record_story_played(self):
        # Add a record to the stories_played table.
        # args: participant, session, level, story
        #self.dbm.record_story_played("P001", 1, 1, "story-fo1")
        # Check that it was inserted correctly.
        # Reset database: remove all the data we added.
        pass


    def test_record_response(self):
        # Add a record to the responses table.
        # args: participant, session, level, story, q_num, q_type, response
        #self.dbm.record_response("P001", 1, 1, "story-fo1", 1, "emotion",
            #"happy")
        # Check that it was inserted correctly.
        # Reset database: remove all the data we added.
        pass


    def test_exceptions(self):
        # For each function, test that we handle exceptions properly.
        m = Mock()
        self.dbm._cursor = m
        m.execute.side_effect = Exception

        with self.assertRaises(Exception):
            self.dbm.get_most_recent_level("p023", 1)

        with self.assertRaises(Exception):
            self.dbm.get_percent_correct_responses("p023", 1)
        with self.assertRaises(Exception):
            self.dbm.get_percent_correct_responses("p023", 1, "emotion")

        with self.assertRaises(Exception):
            self.dbm.get_most_recent_incorrect_emotions("p023", 1)

        with self.assertRaises(Exception):
            self.dbm.get_next_new_story("p391", ["happy", "frustrated"], 1)

        with self.assertRaises(Exception):
            self.dbm.get_next_review_story("40", 5, ["bored"], 8)

        with self.assertRaises(Exception):
            self.dbm.get_level_info(2)

        with self.assertRaises(Exception):
            self.dbm.get_graphics("story-ki2", 4)
