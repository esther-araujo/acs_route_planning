#!/bin/bash

# Directory containing your files
directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/jody_map_creator/generated_maps"

rm -rf "/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"/*

log_directory="/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"

BAR='####################################################################################################' 
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
# Count the total number of files in the directory
total_files=10
processed_files=0

echo "Processando mapa: $1"
process_name="roslaunch acs_route_planning test_convergency.launch room:="$1" launch_file:="$2""

gnome-terminal -- bash -c "$process_name"

# Loop to check the existence of the log file
while true; do
    if [ -e "$log_directory/$1.log" ]; then
        pkill -2 "$process_name"
        sleep 1
        break
    fi
    sleep 2
done

sleep 5

#paplay /usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga
