#!/usr/bin/env bash

# Function to display usage
usage() {
    echo "Usage: $0 -d <directory> -m <models_directory>"
    exit 1
}

# Parse command-line arguments
while getopts ":d:m:" opt; do
    case $opt in
        d) directory="$OPTARG"
        ;;
        m) models_directory="$OPTARG"
        ;;
        \?) echo "Invalid option -$OPTARG" >&2
            usage
        ;;
        :) echo "Option -$OPTARG requires an argument." >&2
            usage
        ;;
    esac
done

# Check if both arguments are provided
if [ -z "$directory" ] || [ -z "$models_directory" ]; then
    usage
fi

# Check if the provided paths exist
if [ ! -d "$directory" ]; then
    echo "Directory $directory does not exist."
    exit 1
fi

if [ ! -d "$models_directory" ]; then
    echo "Models directory $models_directory does not exist."
    exit 1
fi

# Iterate over all ros-bag files in the specified directory
for bag_path in $(find "$directory"/* -maxdepth 0 | grep -v mp4); do
    # Extract the model name from the bag_path
    IFS='-' read -ra ADDR <<< "$(basename "$bag_path" .bag)"
    model_name=$(IFS=- ; echo "${ADDR[*]:1:${#ADDR[*]}-3}")

    # Define the model path
    model_path="$models_directory/$model_name"

    # Run the bag_analysis.py script with the appropriate arguments
    echo "Running analysis for $bag_path"
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    python3 $SCRIPT_DIR/bag_analysis.py --bag_path "$bag_path" --model_path "$model_path" --relative_labels True
done