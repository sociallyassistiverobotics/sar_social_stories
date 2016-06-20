# Jacqueline Kory Westlund
# May 2016
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
from __future__ import print_function # get new version of print
import logging

class ss_logger():
    """ Log data to files and to the ROS log """

    def __init__(self, session, participant):
        """ Set up logger, tag logs with session number and participant ID """
        # TODO set up logger
        print("TODO initialize logger")


    def log(self, logme, *args):
        """ Log at the debug level """
        # TODO log to file or ROS or wherever (with timestamps?)
        if (args):
            print(logme, args)
        else:
            print(logme)



    def logdebug(self, logme, *args):
        """ Log at the debug level """
        # TODO log to file or ROS or wherever (with timestamps?)
        if (args):
            print(logme, args)
        else:
            print(logme)


    def loginfo(self, logme, *args):
        """ Log at the info level """
        # TODO log to file or ROS or wherever (with timestamps?)
        if (args):
            print(logme, args)
        else:
            print(logme)


    def logwarn(self, logme, *args):
        """ Log at the info level """
        # TODO log to file or ROS or wherever (with timestamps?)
        if (args):
            print(logme, args)
        else:
            print(logme)


    def logerr(self, logme, *args):
        """ Log at the info level """
        # TODO log to file or ROS or wherever (with timestamps?)
        if (args):
            print(logme, args)
        else:
            print(logme)


    def logfatal(self, logme, *args):
        """ Log at the info level """
        # TODO log to file or ROS or wherever (with timestamps?)
        if (args):
            print(logme, args)
        else:
            print(logme)


