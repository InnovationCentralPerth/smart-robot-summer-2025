#!/bin/bash

# Default to the current active database
DB_PATH=~/.ros/rtabmap.db

# If argument provided, use that
if [ ! -z "$1" ]; then
    DB_PATH=$1
fi

if [ ! -f "$DB_PATH" ]; then
    echo "Error: Database file '$DB_PATH' not found."
    echo "Usage: ./view_scan.sh [path_to_db_file]"
    exit 1
fi

echo "Opening Database Viewer for: $DB_PATH"
echo "--------------------------------------------------------"
echo "INSTRUCTIONS TO SEE 3D SCAN:"
echo "1. If a window pops up asking about parameters, click 'Yes'."
echo "2. If you only see 2D images or lines:"
echo "   - Go to the menu bar: 'Window' -> 'Show view'"
echo "   - Check '3D Map' (or press Ctrl+3)"
echo "   - You might need to check 'Graph View' as well."
echo "3. In the 3D Map window:"
echo "   - If it's empty, click 'Edit' -> 'Download all clouds' (or 'Download graph')."
echo "   - Use Left Mouse to Rotate, Right Mouse to Zoom."
echo "--------------------------------------------------------"

# Force software rendering and X11 to fix "failed to create EGLContext" errors on Pi
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
rtabmap-databaseViewer "$DB_PATH"
