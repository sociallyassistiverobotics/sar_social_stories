# sar\_social\_stories

The SAR Social Stories game node was designed for use during the SAR Year 5 study. It loads and launches the social stories game, 

## Usage 

`ss\_game\_node.py session participant`

Arguments:

- session
    - Indicates which session we should run so the appropriate game scripts can
      be loaded. Defaults to -1, which indicates that the demo session should
      be loaded.

- participant
    - Indicates which participant is playing so personalizaed game scripts can
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


## Version Notes

This program was developed and tested with Python 2.7.6, ROS Indigo,
sar\_opal\_msgs 2.2.0, and sar\_robot\_command\_msgs 0.1.0 on Ubuntu 14.04 LTS. 

## TODO
- Select scripts
- Load scripts
- Add personalization
- Add logger
- Start game, play scripts


