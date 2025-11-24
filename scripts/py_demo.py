#!/usr/bin/env python3

import sys
import os
import mapskit

def main():
    mk = None
    if len(sys.argv) > 1:
        print(f"argv[1]: {sys.argv[1]}")
        mk = mapskit.MapsKit(sys.argv[1])
    else:
        mk = mapskit.MapsKit()

    # Fix launch path
    package_dir = os.path.dirname(mapskit.__file__)
    # Assuming installed in site-packages/mapskit and launch in site-packages/share/mapskit/launch
    launch_path = os.path.abspath(os.path.join(package_dir, '..', 'share', 'mapskit', 'launch', 'tracking.launch.py'))
    
    if os.path.exists(launch_path):
        print(f"Setting launch path to: {launch_path}")
        mk.set_launch_path(launch_path)
    else:
        print(f"Warning: Launch file not found at {launch_path}")

    pid = mk.run_server()
    print(f"pid is {pid}")
    
    if pid > 0:
        os.waitpid(pid, 0)

if __name__ == "__main__":
    main()
