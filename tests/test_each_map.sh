#!/bin/bash

# Directory containing your files
directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/jody_map_creator/generated_maps"
#directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/map_creator/generated_maps"

rm -rf "/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"/*
rm -rf "/home/esther/catkin_ws/src/acs_route_planning/tests/aco_logs"/*
rm -rf "/home/esther/catkin_ws/src/acs_route_planning/tests/a_star_logs"/*

log_directory="/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"
# log_directory="/home/esther/catkin_ws/src/acs_route_planning/tests/aco_logs"
# log_directory="/home/esther/catkin_ws/src/acs_route_planning/tests/a_star_logs"

tuw="/home/esther/catkin_ws/src/tuw_multi_robot/tuw_multi_robot_demo/cfg/maps"

BAR='####################################################################################################' 
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
# Count the total number of files in the directory
total_files=$(find "$directory" -maxdepth 1 -type f | wc -l)
processed_files=0
method="test_acs_ambient.launch"
# method="test_aco_ambient.launch"
# method="test_a_star_ambient.launch"

num_iterations=100
file_name=$1

# Loop through files in the directory
for ((iteration = 1; iteration <= num_iterations; iteration++)); do
    # Check if the file is a regular file
    if [ -e "$tuw/$file_name" ]; then
        # Extract the filename without extension
        # Run your command with the filename as an argument
        echo "Processando mapa: $file_name"
        process_name="roslaunch acs_route_planning "$method" room:="$file_name""
        
        gnome-terminal -- bash -c "$process_name"

        # Loop to check the existence of the log file
        while true; do
            if [ -e "$log_directory/$file_name.log" ]; then
                pkill -2 "$process_name"
                sleep 1
                break
            fi
            sleep 2
        done
        mv "$log_directory/$file_name.log" "$log_directory/$file_name"_it"$iteration.log"

        # Increment the processed files counter
        ((processed_files++))
    fi
    echo -ne "\r $processed_files% ${GREEN}${BAR:0:$processed_files}${RED}${BAR:$processed_files:$total_files}${NC}\n" 
done

#paplay /usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga
