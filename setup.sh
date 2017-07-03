#!/bin/bash

rm src/*.txt
rm src/*.db

cd src
python ss_init_db.py
# python ss_process_story_ods.py ../source_ods/SAR5_Stories_AA.ods ../source_ods/SAR5_Stories_MP.ods ../source_ods/SAR5_Stories_SK.ods
python ss_process_story_ods.py ../source_ods/SAR5-stories-with-graphics.ods

cp *.txt ../game_scripts/story_scripts
