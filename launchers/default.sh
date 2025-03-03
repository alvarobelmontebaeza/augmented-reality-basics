#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# Select map_file to be "hud", "calibration_pattern" or "lane"
# You can also use the environment variable MAP_FILE to choose map when running the container
dt-exec roslaunch augmented_reality_basics augmented_reality_basics.launch map_file:="calibration_pattern" veh:="$VEHICLE_NAME"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
