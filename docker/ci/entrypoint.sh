#!/bin/bash
set -e

source "/opt/ros/noetic/setup.bash"
source "/root/.cargo/env"

exec "$@"
