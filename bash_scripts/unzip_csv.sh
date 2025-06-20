#!/bin/bash
# Usage: ./unzip_csv.sh <file.csv.gz>
# This script decompresses a .csv.gz file to .csv in the same directory.

if [ $# -ne 1 ]; then
    echo "Usage: $0 <file.csv.gz>"
    exit 1
fi

input="$1"
output="${input%.gz}"

gunzip -c "$input" > "$output"
echo "Decompressed to $output"
