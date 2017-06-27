#!/bin/bash

rm src/*.txt
rm src/*.db

python src/ss_init_db.py
python src/ss_process_story_ods.py source_ods/SAR5_Stories_AA.ods source_ods/SAR5_Stories_MP.ods source_ods/SAR5_Stories_SK.ods
