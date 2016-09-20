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
from ss_personalization_manager import ss_personalization_manager

class test_personalization_manager(unittest.TestCase):
    # TODO test for different participants, sessions, including DEMO!

    def setup_demo(self):
        # Set up to test a demo session, with no participant data in
        # the database yet.
        #args: session, participant, database, percent_correct_to_level
        self.pm = ss_personalization_manager(-1, "DEMO",
                "ss_test_no_participant_data.db", 0.75)

        # Mock the database manager for a participant who has no data
        # in the database yet.
        m = Mock()
        self.pm._db_man = m
        m.get_most_recent_level.return_value = None
        m.get_percent_correct_responses.return_value = None
        m.get_most_recent_incorrect_emotions.return_value = []
        m.get_next_new_story.return_value = "story-fo1"
        m.get_next_review_story.return_value = None
        m.get_level_info.return_value = (3, 1)
        m.get_graphics.return_value = ["story-fo1-P-a.png"]


    def setup_no_participant_data(self, participant, session):
        # Set up test for a participant on their first session, with no
        # participant data in the database yet.

        #args: session, participant, database, percent_correct_to_level
        self.pm = ss_personalization_manager(session, participant,
                "ss_test_no_participant_data.db", 0.75)

        # Mock the database manager for a participant who has no data
        # in the database yet.
        m = Mock()
        self.pm._db_man = m
        m.get_most_recent_level.return_value = None
        m.get_percent_correct_responses.return_value = None
        m.get_most_recent_incorrect_emotions.return_value = []
        m.get_next_new_story.return_value = "story-fo1"
        m.get_next_review_story.return_value = None
        m.get_level_info.return_value = (3, 1)
        m.get_graphics.return_value = ["story-fo1-P-a.png"]
        return m


    def test_get_level_for_session(self):
        # Test demo session.
        self.setup_demo()
        self.assertEqual(self.pm.get_level_for_session(), 1)

        # Test a participant with no data on their first session.
        dbm = self.setup_no_participant_data("P001", 1)

        # Mock past play and performance data so we can test different
        # values.

        # Last level = exception
        #dbm.get_most_recent_level.return_value = Exception
        #with self.assertRaises(Exception):
            #self.pm.get_level_for_session()
            # TODO add exception handling to get_level_for_session?

        # Last level = None (never played before)
        dbm.get_most_recent_level.return_value = None
        self.assertEqual(self.pm.get_level_for_session(), 1)

        # Last level = 1
        dbm.get_most_recent_level.return_value = 1
        # No questions answered, play at level 1.
        dbm.get_percent_correct_responses.return_value = None
        self.assertEqual(self.pm.get_level_for_session(), 1)
        # All questions correct, play at level 2.
        dbm.get_percent_correct_responses.return_value = 1
        self.assertEqual(self.pm.get_level_for_session(), 2)
        # 75% questions correct, play at level 2.
        dbm.get_percent_correct_responses.return_value = 0.75
        self.assertEqual(self.pm.get_level_for_session(), 2)
        # 74% questions correct, play at level 1.
        dbm.get_percent_correct_responses.return_value = 0.74
        self.assertEqual(self.pm.get_level_for_session(), 1)
        # -3% questions correct, play at # level 1.
        dbm.get_percent_correct_responses.return_value = -0.03
        self.assertEqual(self.pm.get_level_for_session(), 1)

        # Last level = 4
        dbm.get_most_recent_level.return_value = 4
        # No questions answered ever, play at level 1.
        dbm.get_percent_correct_responses.return_value = None
        self.assertEqual(self.pm.get_level_for_session(), 4)
        # All questions correct, play at level 5.
        dbm.get_percent_correct_responses.return_value = 1
        self.assertEqual(self.pm.get_level_for_session(), 5)
        # 75% questions correct, play at level 5.
        dbm.get_percent_correct_responses.return_value = 0.75
        self.assertEqual(self.pm.get_level_for_session(), 5)
        # 74% questions correct, play at level 4.
        dbm.get_percent_correct_responses.return_value = 0.74
        self.assertEqual(self.pm.get_level_for_session(), 4)
        # -3% questions correct, play at # level 4.
        dbm.get_percent_correct_responses.return_value = -0.03
        self.assertEqual(self.pm.get_level_for_session(), 4)

        # Last level = 10
        dbm.get_most_recent_level.return_value = 10
        # No questions answered ever, play at level 10.
        dbm.get_percent_correct_responses.return_value = None
        self.assertEqual(self.pm.get_level_for_session(), 10)
        # All questions correct, play at level 10.
        dbm.get_percent_correct_responses.return_value = 1
        self.assertEqual(self.pm.get_level_for_session(), 10)
        # 75% questions correct, play at level 10.
        dbm.get_percent_correct_responses.return_value = 0.75
        self.assertEqual(self.pm.get_level_for_session(), 10)
        # 74% questions correct, play at level 10.
        dbm.get_percent_correct_responses.return_value = 0.74
        self.assertEqual(self.pm.get_level_for_session(), 10)
        # -3% questions correct, play at # level 10.
        dbm.get_percent_correct_responses.return_value = -0.03
        self.assertEqual(self.pm.get_level_for_session(), 10)



    def test_get_performance_this_session(self):
        pass


    def test_get_performance_this_session(self):
        pass


    def test_get_next_story_script(self):
        pass


    def test_get_next_story_details(self):
        pass


    def test_record_story_loaded(self):
        pass


    def test_record_user_response(self):
        pass


    def test_set_start_level(self):
        pass


    def test_get_joint_attention_level(self):
        pass



