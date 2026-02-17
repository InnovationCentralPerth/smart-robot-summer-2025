#!/bin/bash

# Default to the current active database
DB_PATH=~/.ros/rtabmap.db
OUTPUT_DIR="/home/icp/icp/Rasberry Pi Side/Vision/src/exports"

mkdir -p $OUTPUT_DIR

# If argument provided, use that
if [ ! -z "$1" ]; then
    DB_PATH=$1
fi

if [ ! -f "$DB_PATH" ]; then
    echo "Error: Database file '$DB_PATH' not found."
    echo "Usage: ./export_scan.sh [path_to_db_file]"
    exit 1
fi

TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
FILENAME="scan_$TIMESTAMP"

echo "Exporting 3D Cloud from: $DB_PATH"
echo "To: $OUTPUT_DIR/$FILENAME.ply"

# Export as PLY (Standard Point Cloud)
# Syntax: rtabmap-export --cloud --output_dir [dir] --output [name] [database_path]
rtabmap-export --cloud --output_dir "$OUTPUT_DIR" --output "$FILENAME" "$DB_PATH"

echo "Done! You can open the .ply file in MeshLab or Blender."
