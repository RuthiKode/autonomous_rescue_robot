#!/bin/bash
# Usage: probe_video.sh <video> <frame.png> [at_seconds]  -- print duration/stream, save one frame
F=$(python3 -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())")
"$F" -i "$1" 2>&1 | grep -E "Duration|Stream"
"$F" -y -loglevel error -ss "${3:-15}" -i "$1" -frames:v 1 "$2"
ls -la "$2"
