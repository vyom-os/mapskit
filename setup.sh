#!/bin/bash

# This script runs setup scripts in a specific order.

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SETUP_DIR="$SCRIPT_DIR/setup"

# List of scripts to run in order
scripts_to_run=(
    "setup-ros.sh"
)

echo "Starting setup..."

for script in "${scripts_to_run[@]}"; do
    script_path="$SETUP_DIR/$script"
    if [ -f "$script_path" ]; then
        echo "Running $script..."
        # Make sure the script is executable
        chmod +x "$script_path"
        # Run the script
        if "$script_path"; then
            echo "SUCCESS: $script finished successfully."
        else
            echo "ERROR: $script failed." >&2
            # Exit the main script if a sub-script fails
            exit 1
        fi
    else
        echo "WARNING: $script not found in $SETUP_DIR."
    fi
done

echo "Setup finished successfully."
