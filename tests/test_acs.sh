#!/bin/bash

# Directory containing your files
directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/jody_map_creator/generated_maps"
#directory="/home/esther/catkin_ws/src/acs_route_planning/scripts/map_creator/generated_maps"

rm -rf "/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"/*

log_directory="/home/esther/catkin_ws/src/acs_route_planning/tests/acs_logs"

# Loop through files in the directory
for file in "$directory"/*; do
    # Check if the file is a regular file
    if [ -f "$file" ]; then
        # Extract the filename without extension
        filename=$(basename "$file" | cut -d. -f1)
        # Run your command with the filename as an argument
        echo "Processing file: $filename"
        process_name="roslaunch acs_route_planning test_acs_ambient.launch room:="$filename""
        
        gnome-terminal -- bash -c "$process_name"

        # Loop para verificar a existência do arquivo de log
        while true; do
            if [ -e "$log_directory/$filename.log" ]; then
                echo "Arquivo de log detectado. Fechando o terminal."
                pkill -2 "$process_name"
                sleep 1
                break
            fi
            sleep 2
        done
    fi
done
