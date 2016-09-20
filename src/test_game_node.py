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
from ss_game_node import ss_game_node

class test_game_node(unittest.TestCase):

    #def setUp(self):
        #self.gn = ss_game_node()

    #@mock.patch('argparse.ArgumentParser', autospec=True)
    #def test_parse_arguments(self, mock_args):
        #mock_args.return_value.parse_args.side_effect = [
                #{"session": 1, "participant": "test"},
                #{"session": -1, "participant": "test"},
                #{"session": -4, "participant": "test"},
                #{"session": 0.4, "participant": "test"}
                #]

        #(session, participant) = self.gn.parse_arguments()
        #self.assertEqual(session, 1)
        #self.assertEqual(participant, 'test')

        #(session, participant) = self.gn.parse_arguments()
        #self.assertEqual(session, -1)
        #self.assertEqual(participant, "DEMO")

        #with self.assertRaises(ValueError):
            #self.gn.parse_arguments()

        #with self.assertRaises(ValueError):
            #self.gn.parse_arguments()


    def test_launch_game(self):
        pass


if __name__ == '__main__':
    unittest.main(verbosity=2)
