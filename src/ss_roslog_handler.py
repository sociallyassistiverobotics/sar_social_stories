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
import logging
import rospy # for ROS logging
from __future__ import print_function # get new version of print

class ss_roslog_handler(logging.Handler):
    """ Logger handler to connect to ROS """

    # create mapping of python logger levels to rospy log levels
    MAP = {
        logging.DEBUG:rospy.logdebug,
        logging.INFO:rospy.loginfo,
        logging.WARNING:rospy.logwarn,
        logging.ERROR:rospy.logerr,
        logging.CRITICAL:rospy.logfatal
    }


    def emit(self, record):
        """ Emit a record (route log message to rospy) """
        # use the log level for this message to route the message to 
        # the appropriate rospy log function
        try:
            self.MAP[record.levelno](record.name, ":", record.msg)
        except KeyError:
            rospy.logerr("Unknown log level", record.levelno, "LOG:",
                    record.name, ":", record.msg)


# connect logging calls from this logger's children to ROS
logging.getLogger('trigger').addHandler(ss_roslog_handler())

# send all messages with this log level or higher to ROS
logging.getLogger('trigger').setLevel(logging.DEBUG)
