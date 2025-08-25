#!/bin/bash

# This script launches multiple instances of an experiment defined by
# the 'start_dancers_minidancers.sh' script, each with a different
# configuration file from a specified folder.
# It limits the number of parallel experiments using the max_workers variable.

# --- Configuration ---
# Path to your original experiment script.
# Make sure this path is correct relative to where you run this mother script.
EXPERIMENT_SCRIPT="$ROS_WS/src/launch/tmux_scripts/start_dancers_minidancers.sh"

NETWORK_SIM_PKG="ns-3_sim"
NETWORK_SIM_NODE="wifi_flocking"
CONTROLLER_PKG="flocking_controller"
CONTROLLER_NODE="flocking_controller_vat"

# Default maximum number of parallel experiments.
# You can change this value as needed.
MAX_WORKERS=6

# --- Usage and Argument Parsing ---
if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <path_to_config_files_folder>"
    echo "       The folder should contain .yaml configuration files for the experiments."
    exit 1
fi

CONFIG_FOLDER="$1"

# Check if the provided folder exists.
if [ ! -d "$CONFIG_FOLDER" ]; then
    echo "Error: Configuration folder '$CONFIG_FOLDER' not found."
    exit 1
fi

# Get all YAML configuration files in the specified folder.
# We use find to ensure we get files even in subdirectories if desired,
# but for simplicity, we'll just look directly in the specified folder.
CONFIG_FILES=$(find "$CONFIG_FOLDER" -maxdepth 1 -name "*.yaml" | sort -V)

if [ -z "$CONFIG_FILES" ]; then
    echo "No .yaml configuration files found in '$CONFIG_FOLDER'."
    exit 0
fi

echo "Launching experiments with a maximum of $MAX_WORKERS parallel workers."
echo "Configuration files to process:"
echo "$CONFIG_FILES"
echo "---"

# Initialize an array to keep track of background PIDs.
declare -a PIDS

session_id=1

# --- Launching Experiments ---
for config_file in $CONFIG_FILES; do
    # Wait if the number of running processes exceeds MAX_WORKERS.
    while [ $(tmux ls | wc -l) -ge "$MAX_WORKERS" ]; do
        echo "Max workers ($MAX_WORKERS) reached. Waiting for a slot..."
        sleep 5 # Wait for a few seconds before re-checking
    done

    echo "Starting experiment for: $config_file"
    # Launch the experiment script in a new terminal in the background.
    gnome-terminal -- bash -c "$EXPERIMENT_SCRIPT \"$config_file\" \"$NETWORK_SIM_PKG\" \"$NETWORK_SIM_NODE\" \"$CONTROLLER_PkG\" \"$CONTROLLER_NODE\" \"$session_id\"" &
    session_id=$((session_id + 1))
    PIDS+=($!) # Store the PID of the background process.
    # print PIDS array
    sleep 2 # Small delay to give the terminal a chance to open and tmux to start
done

# Wait for all remaining background processes to complete.
echo "All experiments launched. Waiting for them to finish (or for you to close terminals)..."
# Loop until all PIDs are gone
while [ $(tmux ls | wc -l) -gt 0 ]; do
    if [ $(tmux ls | wc -l) -gt 0 ]; then
        echo "$(tmux ls | wc -l) experiments still running. Waiting..."
        sleep 10 # Wait longer at the end
    fi
done

echo "All experiments completed or terminals closed."
