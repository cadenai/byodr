#!/usr/bin/env bash
set -e

/bin/bash -c "/etc/init.d/ueyeusbdrc start"

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

#exec "$@"
exec su - app -c "export PYTHONPATH=.:/common \\
  && export LD_LIBRARY_PATH=/deps/peak-linux-driver-8.9.3/libpcanbasic/pcanbasic:/deps/peak-linux-driver-8.9.3/lib/lib \\
  && cd /app && $*"
