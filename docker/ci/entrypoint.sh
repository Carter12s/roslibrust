#!/bin/bash
set -e
set -a

. "/opt/ros/noetic/setup.bash"
. "/root/.cargo/env"

exec "$@"
