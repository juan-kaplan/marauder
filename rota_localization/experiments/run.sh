#!/bin/bash
# Runs a localization comparison: simulation, the three localizers and the drive.
#
#   ./run.sh figure8                            # turtlebot3_world, with Gazebo and RViz
#   ./run.sh corridor gui:=false                # L corridor, headless
#   LOG=run.npz ./run.sh corridor gui:=false    # also save the run for render.py
#
# Extra arguments go to compare_launch.py (gui, noise_bias, noise_std).
# The workspace with rota_localization is taken from $TNR_WS (default ~/tp1_ws).
HERE=$(dirname "$(readlink -f "$0")")
SCENARIO=${1:?usage: run.sh figure8|corridor [launch args]}
shift
source /opt/ros/humble/setup.bash
source "${TNR_WS:-$HOME/tp1_ws}/install/setup.bash"
# VS Code's snap leaks its GTK/GIO environment, which crashes gzclient and rviz2.
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
[ -n "$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR LOCPATH GIO_LAUNCHED_DESKTOP_FILE

"$HERE/stop.sh" > /dev/null
setsid ros2 launch "$HERE/compare_launch.py" scenario:="$SCENARIO" "$@" > /tmp/tnr_compare_launch.log 2>&1 &
echo $! > /tmp/tnr_compare.pgid
echo "simulation starting (log: /tmp/tnr_compare_launch.log), driving in 15 s"
sleep 15
python3 "$HERE/drive_$SCENARIO.py" ${LOG:+--log "$LOG"}
echo "done; the simulation keeps running until $HERE/stop.sh"
