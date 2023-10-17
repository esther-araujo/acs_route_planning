#!/bin/bash

# Directory containing your files
directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/jody_map_creator/generated_maps"
#directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/map_creator/generated_maps"

rm -rf "/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"/*

log_directory="/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"

BAR='####################################################################################################' 
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
# Count the total number of files in the directory
total_files=$(find "$directory" -maxdepth 1 -type f | wc -l)
processed_files=0

# Loop through files in the directory
for file in "$directory"/*; do
    # Check if the file is a regular file
    if [ -f "$file" ]; then
        # Extract the filename without extension
        filename=$(basename "$file" | cut -d. -f1)
        # Run your command with the filename as an argument
        echo "Processando mapa: $filename"
        process_name="roslaunch acs_route_planning test_acs_ambient.launch room:="$filename""
        
        gnome-terminal -- bash -c "$process_name"

        # Loop to check the existence of the log file
        while true; do
            if [ -e "$log_directory/$filename.log" ]; then
                pkill -2 "$process_name"
                sleep 1
                break
            fi
            sleep 2
        done

        # Increment the processed files counter
        ((processed_files++))
    fi
    echo -ne "\r $processed_files% ${GREEN}${BAR:0:$processed_files}${RED}${BAR:$processed_files:$total_files}${NC}\n" 
done

