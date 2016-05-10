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


class ss_script_parser(): 
    """ Determine which session scripts to load, load them, and provide the next
    line in the script file on request. """ 
    
    def __init__(self, logger):
        """ Initialize script parser manager """
        # save reference to logger for logging stuff later
        self.logger = logger


    def get_session_script(self, session):
        """ Get scripts for the specified session """
        if session == -1:
            # will use the demo session script
            # TODO get script!
            return "../session_scripts/demo.txt"
        else:
            # pick out the scripts for the specified session
            # TODO get script!
            return "../session_scripts/demo.txt"


    def load_script(self, script):
        """ Set up to load script """
        # open script for reading
        try:
            self.fh = open(script, "r")
        except IOError:
            self.logger.log("Cannot open script: " + script)
            # pass exception up so anyone trying to load a script
            # knows it didn't work
            raise
        else:
            # log that we opened script
            self.logger.log("Opened ", script)


    def next_line(self):
        """ Get the next line in the script """
        # read and return next line in script file
        try:
            return self.fh.next()
        
        # may get attribute error if file handle does not exist because no
        # script was loaded
        except AttributeError:
            self.logger.log("No script loaded!")
        
        except StopIteration:
            self.logger.log("At end of script file!")
            # close script file now that we're done
            self.fh.close()
            # pass on the stop iteration exception
            raise

