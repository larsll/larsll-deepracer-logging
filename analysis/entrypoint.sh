#!/bin/bash
set -e

# setup ros2 environment
source "/workspace/install/setup.bash" --
exec "$@"