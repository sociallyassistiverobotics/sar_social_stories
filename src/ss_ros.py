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

import rospy # ROS
from sar_opal_msgs.msg import OpalCommand # ROS msgs to talk to tablet

class tega_teleop_ros():
    # ROS node
    # set up rostopics we publish: commands to the tablet and commands
    # to Tega
    tablet_pub = rospy.Publisher('opal_tablet_command', OpalCommand,
            queue_size = 10)


    def __init__(self, ros_node):
        """ Initialize ROS """
        # we get a reference to the main ros node so we can do callbacks
        # to publish messages, and subscribe to stuff
        self.ros_node = ros_node

        # subscribe to other ros nodes
        # TODO could we put list of nodes to subscribe to in config file?
        # TODO subscribe to messages from opal game
        # EXAMPLE rospy.Subscriber('tega_state', TegaState, self.on_tega_state_msg)


    def send_opal_message(self, command):
        """ Publish opal command message """
        print 'sending opal command: %s' % command
        msg = OpalCommand()
        msg.command = command
        # TODO opal command messages often take properties, add these!
        # TODO use sar_opal_sender as examples of how to add properties
        self.tablet_pub.publish(msg)
        rospy.loginfo(msg)


    # TODO add callbacks for any rosmsgs we subscribe to!
    # EXAMPLE:
    #def on_tega_state_msg(self, data):
        # when we get tega state messages, set a flag indicating whether the
        # robot is in motion or playing sound or not
        #self.flags.tega_is_playing_sound = data.is_playing_sound
        
        # Instead of giving us a boolean to indicate whether tega is in motion
        # or not, we get the name of the animation. Let's check whether it is
        # our "idle" animation (usually, the idle animation is either
        # MOTION_IDLESTILL or MOTION_BREATHING).
        #self.flags.tega_is_doing_motion = data.doing_motion
