# sar\_social\_stories

The SAR Social Stories game node was designed for use during the SAR Year 5 study. This program loads and launches the social stories game, using scripts to list what the robot should be told to do, what stories should be loaded in the game, and what to do in response to stuff that happens in the game. It uses ROS to communicate with a SAR Opal game (on a tablet or on a PC) via a rosbridge\_server websocket connection and with the robot via the robot\_command node.

## Usage 

`ss\_game\_node.py session participant`

Arguments:

- session
    - Indicates which session we should run so the appropriate game scripts can
      be loaded. Defaults to -1, which indicates that the demo session should
      be loaded.

- participant
    - Indicates which participant is playing so personalized game scripts can
      be loaded. Defaults to DEMO, which indicates that the demo session should
      be loaded.


### Demo Version

To run the demo version of the game, run without arguments (the argument
defaults indicate the demo game should be loaded), enter "-1" as the session
number, and/or enter "DEMO" as the participant string. 

## ROS messages

This node subscribes to the ROS topic "robot\_state" to receive messages of the type "/[sar\_robot\_command\_msgs] (https://github.com/personal-robots/sar_robot_command_msgs "/sar_robot_command_msgs")/RobotState".

This node subscribes to the ROS topic "opal\_tablet\_action" to receive messages of the type "/[sar\_opal\_msgs] (https://github.com/personal-robots/sar_opal_msgs "/sar_opal_msgs")/OpalAction".

This node publishes "/[sar\_opal\_msgs] (https://github.com/personal-robots/sar_opal_msgs "/sar_opal_msgs")/OpalCommand" messages to the topic "opal\_tablet\_command".

This node publishes "/[sar\_robot\_command\_msgs] (https://github.com/personal-robots/sar_robot_command_msgs "/sar_robot_command_msgs")/RobotCommand" messages to the topic "robot\_command".

## Game Scripts

The program will use the interaction scripts in the "session\_scripts/" and
"story\_scripts/" directories.

### Session Scripts

By default, the demo session uses the "demo.txt" session script. When you
specify a particular session, the session script named "session-\[NUMBER\].txt"
will be used. For example, if you are loading session 2, the session script
"session-2.txt" will be used. 

Script lines should be tab-delimited. Look at the demo script located in
"session\_scripts/demo.txt" for an example.

The session script lists what happens during a game session. It should list, in
order, the actions the program should take. These actions include the
following, which are described in more detail below:

- ADD
- SET
- ROBOT
- OPAL
- WAIT
- REPEAT
- STORY

#### ADD

ADD is used to add a list of robot commands that can be used in response to a
particular trigger. Triggers may be actions the user takes, such as selecting a
correct or incorrect response for a question, input from sensors, or a
particular repeating action in the script, such as an introductory comment
before telling a story. ADD should list the trigger and the file containing the
list of robot commands. For example, the following command will load the
commands listed in "incorrect.txt" as response options for incorrect actions
taken by the user:

`ADD INCORRECT\_RESPONSE incorrect.txt`

Currently, the following lists can be added:

- CORRECT\_RESPONSES: Responses for correct user actions
- INCORRECT\_RESPONSES: Responses for incorrect user actions
- YES\_RESPONSES: Responses to the user selecting a "yes" button
- NO\_RESPONSES: Responses to the user selecting a "no" button
- STORY\_INTROS: Introductory comment before telling a story
- STORY\_CLOSINGS: Closing comment after telling a story
- TIMEOUT\_CLOSINGS: Responses for when the maximum game time is reached

#### SET

SET will set configuration options or other constants. Currently, you can set
the following:

- MAX\_INCORRECT\_RESPONSES: The maximum number of incorrect responses the user can provide for a question before the game moves on.
- MAX\_GAME\_TIME: The maximum amount of time, in minutes, that the user can play the game before it exits.
- MAX\_STORIES: The maximum number of stories to tell in one game session.

For example, the following commands will set the maximum incorrect responses to
2 and the maximum game time allowed to 10 minutes:

`SET MAX\_INCORRECT\_RESPONSES    2`
`SET MAX\_GAME\_TIME 10`

#### ROBOT

ROBOT is used to send the robot a speech and/or action command. For DO actions,
write it as a string containing what the robot should say and the actions the
robot should do, following the format for RobotCommand messages defined in
[sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs
"/sar_robot_command_msgs"). For example, the following would send a command to
the robot to say "Hi, I am a robot" while smiling:

`ROBOT   DO "Hi &lt;smile&rt;, I am a robot!"`

You can also send other RobotCommand messages, for example:

`ROBOT  SLEEP`

The robot commands also include a few special commands that indicate that one
of the robot commands from the STORY\_INTROS or STORY\_CLOSINGS lists should be
used, such as:

`ROBOT  STORY_INTRO` 

#### OPAL

OPAL is used to send commands to the SAR Opal game. You can include the full
command on the line, following the format defined in [sar\_opal\_msgs] (https://github.com/personal-robots/sar_opal_msgs "/sar_opal_msgs") for
OpalCommand messages. However, use the command name (not the number!) on the
line -- so you'd write "CLEAR" instead of "6". You can also use the command
"LOAD\_ALL" followed by the name of a text file that contains a list of objects
to load with their associated properties to load a particular set of objects
all at once. Similarly, the command "LOAD\_STORY" will load the graphics for the
next story.

For example, the following would send opal commands to clear the game scene,
set up the next story scene, load the pictures for the next story, and set the
correct responses for the next story:

`OPAL    CLEAR`
`OPAL    SETUP\_STORY\_SCENE`
`OPAL    LOAD\_STORY`
`OPAL    SET\_CORRECT`

#### WAIT

WAIT is used to wait for a response from the user via a particular trigger
before proceeding in the game. A timeout is specified so that if no response is
received, script playback will continue after that amount of time. The timeout
should be specified in seconds. For example, the following would wait for a
response to a YES or NO button press and would timeout after 10 seconds:

`WAIT    YES\_NO  10`

#### REPEAT

REPEAT allows you to specify a script that should be repeated multiple times
before continuing on to the next part of the main session script. You also must
specify how many times the script should be repeated by either providing an
integer or providing the name of a set constant, such as MAX\_STORIES. The
script will be repeated that many times or until the maximum game time has
elapsed, whichever is reached first. 

The following example would repeat the "demo-stories.txt" script for
MAX\_STORIES iterations or until the maximum game time elapsed: 

`REPEAT  MAX\_STORIES    demo-stories.txt`

#### STORY

STORY indicates that the next story for this session should be played back.
Since which stories are played back depends on the session, participant, and
personalization, this can only be determined at the start of a game session --
not ahead of time. Thus, we use the STORY command to indicate that it's time to
play back the next story. This command requires no arguments, as the program
will keep its own list of which stories should be played back next, so a line
will look like this:

STORY

### Story Scripts

The story scripts follow the same format as the main session scripts. See the example in "story\_scripts/demo-story-1.txt".

## Version Notes

This program was developed and tested with Python 2.7.6, ROS Indigo, [sar\_opal\_msgs](https://github.com/personal-robots/sar_opal_msgs "/sar_opal_msgs") 2.2.0, and [sar\_robot\_command\_msgs](https://github.com/personal-robots/sar_robot_command_msgs "/sar_robot_command_msgs") 0.1.0 on Ubuntu 14.04 LTS. 

## TODO
- Select scripts
- Consider listing the mapping of session script files to sessions in a config file.
- Load scripts
- Add personalization
- Add logger
- Start game, play scripts


