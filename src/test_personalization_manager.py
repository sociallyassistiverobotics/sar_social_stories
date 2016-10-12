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
from mock import Mock, patch
from ss_personalization_manager import ss_personalization_manager
from SS_Errors import NoStoryFound

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
        m.get_graphics.return_value = ["FO1-a-p.png"]


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
        m.get_graphics.return_value = ["FO1-a-p.png"]
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

        # Last level played was...
        last_level = [1,4,10]
        # Percent questions correct were... None = no questions
        # answered, 1 = all questions correct, 0.75 = 75% correct, etc.
        percent_questions_correct = [None, 1, 0.75, 0.74, -0.03]
        # Then we expect to play at level...
        expected_level = [
            (1, 2, 2, 1, 1),
            (4, 5, 5, 4, 4),
            (10, 10, 10, 10, 10)
            ]

        # Test at each level.
        for i in range(0, len(last_level)):
            dbm.get_most_recent_level.return_value = last_level[i]

            # Test for each percentage of questions correct.
            for j in range(0, len(percent_questions_correct)):
                dbm.get_percent_correct_responses.return_value = \
                    percent_questions_correct[j]
                self.assertEqual(self.pm.get_level_for_session(),
                    expected_level[i][j])


    def test_get_performance_this_session(self):
        # Test demo session.
        self.setup_demo()
        self.assertEqual(self.pm.get_performance_this_session(), None)

        # Test a participant with no data on their first session.
        dbm = self.setup_no_participant_data("P001", 1)

        # Mock past play and performance data so we can test different
        # values. Returns (emotion, ToM, order) performance.
        performance_data = [
                # No questions answered.
                (None, None, None),
                # All correct.
                (1, 1, 1),
                # All incorrect.
                (0, 0, 0),
                # Some correct.
                (0.5, 0.75, 0.5),
                # Didn't answer some, answered others.
                (None, 0, 0),
                (0, None, 0),
                (0, 0, None),
                (1, None, None),
                (None, None, 1),
                (None, 1, None),
                (0.4, -1, 5),
                ]

        for pd in performance_data:
            dbm.get_percent_correct_responses.side_effect = [ pd[0], pd[1],
                pd[2] ]
            self.assertEqual(self.pm.get_performance_this_session(), pd)


    @patch("ss_personalization_manager.ss_personalization_manager."
        + "pick_next_story")
    def test_get_next_story_script(self, mock):
        # The get_next_story_script function is also tested in the next test,
        # when testing the pick_next_story function.
        # Test a participant with no data on their first session.
        dbm = self.setup_no_participant_data("P001", 1)
        mock.return_value = "story-cr1"
        self.assertEqual(self.pm.get_next_story_script(), "story-cr1-1.txt")
        self.assertTrue(mock.called)


    def test_pick_next_story(self):
        # Test demo session.
        self.setup_demo()
        self.assertEqual(self.pm.pick_next_story(), "demo-story-1")
        self.assertEqual(self.pm._current_story, "demo-story-1")
        self.assertEqual(self.pm.get_next_story_script(), "demo-story-1.txt")

        # Test a participant with no data on their first session.
        dbm = self.setup_no_participant_data("P001", 1)

        # Tell new story starts out True, toggles after each call to
        # pick_new_story.
        self.assertTrue(self.pm._tell_new_story)

        # Mock relevant story data.
        # Need new story, no new or review story found.
        dbm.get_next_new_story.side_effect = [ None, None ]
        dbm.get_next_review_story.side_effect = [ None ]
        with self.assertRaises(NoStoryFound):
            self.pm.pick_next_story()

        # Need new story, no new story found.
        dbm.get_next_new_story.side_effect = [ None, None ]
        dbm.get_next_review_story.side_effect = [ "story-cr1" ]
        self.assertEqual(self.pm.pick_next_story(), "story-cr1")
        self.assertEqual(self.pm._current_story, "story-cr1")
        self.assertEqual(self.pm.get_next_story_script(), "story-cr1-1.txt")

        # Tell new story flag should toggle.
        self.assertFalse(self.pm._tell_new_story)

        # Need review story, review story found on 1st try.
        dbm.get_next_review_story.side_effect = [ "story-cf1" ]
        self.assertEqual(self.pm.pick_next_story(), "story-cf1")
        self.assertEqual(self.pm._current_story, "story-cf1")
        self.assertEqual(self.pm.get_next_story_script(), "story-cf1-1.txt")

        # Tell new story flag should toggle.
        self.assertTrue(self.pm._tell_new_story)

        # Need new story, new story found on 1st try.
        dbm.get_next_new_story.side_effect = [ "story-cr1", None ]
        dbm.get_next_review_story.side_effect = [ None ]
        self.assertEqual(self.pm.pick_next_story(), "story-cr1")
        self.assertEqual(self.pm._current_story, "story-cr1")
        self.assertEqual(self.pm.get_next_story_script(), "story-cr1-1.txt")

        # Tell new story flag should toggle.
        self.assertFalse(self.pm._tell_new_story)

        # Need review story, no review or new story found.
        dbm.get_next_new_story.side_effect = [ None ]
        dbm.get_next_review_story.side_effect = [ None ]
        with self.assertRaises(NoStoryFound):
            self.pm.pick_next_story()

        # Need review story, no review story found, get new story.
        dbm.get_next_new_story.side_effect = [ "story-cf1" ]
        dbm.get_next_review_story.side_effect = [ None ]
        self.assertEqual(self.pm.pick_next_story(), "story-cf1")
        self.assertEqual(self.pm._current_story, "story-cf1")
        self.assertEqual(self.pm.get_next_story_script(), "story-cf1-1.txt")

        # Tell new story flag should toggle.
        self.assertTrue(self.pm._tell_new_story)

        # Need new story, new story found on 2nd try.
        dbm.get_next_new_story.side_effect = [ None, "story-cr1" ]
        dbm.get_next_review_story.side_effect = [ None ]
        self.assertEqual(self.pm.pick_next_story(), "story-cr1")
        self.assertEqual(self.pm._current_story, "story-cr1")
        self.assertEqual(self.pm.get_next_story_script(), "story-cr1-1.txt")

        # Tell new story flag should toggle.
        self.assertFalse(self.pm._tell_new_story)



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



