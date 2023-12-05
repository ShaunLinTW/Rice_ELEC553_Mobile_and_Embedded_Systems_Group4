#!/bin/bash

# Set the input and output file names
INPUT_PATTERN="snap%d.0.png"
OUTPUT_FILE="ELEC553_final_project.gif"
FRAME_RATE=10

# ffmpeg -y -i $INPUT_PATTERN -vf "fps=30,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 $OUTPUT_FILE]

ffmpeg -framerate $FRAME_RATE -i $INPUT_PATTERN $OUTPUT_FILE

echo "Conversion complete. Output saved as $OUTPUT_FILE"