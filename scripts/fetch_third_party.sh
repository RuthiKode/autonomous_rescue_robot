#!/bin/bash
# Fetch third-party source that has no Jazzy binary package. Pinned to a commit; the
# result lives in src/third_party (git-ignored) and is built with the rest of the workspace.
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$WS/src/third_party"
cd "$WS/src/third_party"
# explore_lite (frontier exploration): https://github.com/robo-friends/m-explore-ros2, branch main
if [ ! -d m-explore-ros2 ]; then
  git clone https://github.com/robo-friends/m-explore-ros2.git
fi
git -C m-explore-ros2 fetch --quiet origin
git -C m-explore-ros2 checkout --quiet 326cf8a0b487c34246bb8f3326afbcd69576dc60
# Only explore_lite is used; keep the map-merge package out of the build.
touch m-explore-ros2/map_merge/COLCON_IGNORE
echo "m-explore-ros2 at $(git -C m-explore-ros2 rev-parse --short HEAD)"
