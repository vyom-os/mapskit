#!/bin/bash

# This script runs setup scripts in a specific order.

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SETUP_DIR="$SCRIPT_DIR/setup"

# --- Add PYTHONPATH to .bashrc ---
echo "Configuring PYTHONPATH in ~/.bashrc..."
BASHRC_FILE="$HOME/.bashrc"
SITE_PACKAGES_PATH="/usr/local/lib/python3.10/site-packages"
PYTHONPATH_SNIPPET=$(cat <<'EOF'

# Add site-packages to PYTHONPATH if it's not already there
if [[ ":$PYTHONPATH:" != *":/usr/local/lib/python3.10/site-packages:"* ]]; then
    export PYTHONPATH="/usr/local/lib/python3.10/site-packages${PYTHONPATH:+":$PYTHONPATH"}"
fi
EOF
)

# Check if the configuration is already in .bashrc to avoid duplicates
if ! grep -qF "$SITE_PACKAGES_PATH" "$BASHRC_FILE"; then
    echo "Adding site-packages to PYTHONPATH in $BASHRC_FILE."
    echo "$PYTHONPATH_SNIPPET" >> "$BASHRC_FILE"
    echo "PYTHONPATH configured. Please run 'source ~/.bashrc' or open a new terminal for changes to take effect."
else
    echo "PYTHONPATH for site-packages already appears to be configured in $BASHRC_FILE."
fi
# --- End of PYTHONPATH section ---


# List of scripts to run in order
scripts_to_run=(
    "setup-deps.sh"
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
