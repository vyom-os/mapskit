# mapskit/__init__.py

try:
    # Import everything from the compiled C++ extension
    from .mapskit_core_py import *
except ImportError as e:
    print(f"Error importing mapskit_core C++ extension: {e}")
    # Handle the error appropriately, perhaps exit if the core is mandatory

# You can add Python wrappers or utility functions here
__version__ = "0.0.1"
