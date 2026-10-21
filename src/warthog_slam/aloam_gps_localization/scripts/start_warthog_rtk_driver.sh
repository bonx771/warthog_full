#!/usr/bin/env bash
set -euo pipefail

# gps_pkg also exists in warthog_full, but the RTK STM32 driver requested by
# this package lives in /home/cuong/warthog. Source that workspace inside this
# process so rosrun resolves the intended gps_UBX_node.py implementation.
source /home/cuong/warthog/devel/setup.bash
exec rosrun gps_pkg gps_UBX_node.py "$@"

