#!/bin/bash
# docs/media/demo.gif from docs/media/demo.mp4 (GitHub READMEs do not play mp4 inline).
# Needs ffmpeg (sudo apt install ffmpeg). The committed GIF was made with the same two commands.
cd ~/autonomous_rescue_robot/docs/media
ffmpeg -y -loglevel error -i demo.mp4 -vf "fps=8,scale=420:-1:flags=lanczos,palettegen=max_colors=64" /tmp/pal.png
ffmpeg -y -loglevel error -i demo.mp4 -i /tmp/pal.png -lavfi "fps=8,scale=420:-1:flags=lanczos [x]; [x][1:v] paletteuse=dither=bayer" demo.gif
ls -la demo.gif
