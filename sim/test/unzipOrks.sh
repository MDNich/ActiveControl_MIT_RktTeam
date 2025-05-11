#!/bin/bash

cd ~/Developer/ActiveControl_MIT_RktTeam/sim/dat/ork

cd xml

for orkfile in ../*.ork; do
    filename=$(basename "$orkfile")
    basename="${filename%.*}"
    unzip -p "$orkfile" rocket.ork > "${basename}.xml"
done
