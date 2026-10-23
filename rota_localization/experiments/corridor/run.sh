#!/bin/bash
# Runs the L-corridor experiment: simulation, the three localizers and the drive.
#
#   ./run.sh                          # with Gazebo and RViz windows
#   ./run.sh gui:=false               # headless
#   LOG=run.npz ./run.sh gui:=false   # also save the run for render.py
#
# Extra arguments go to corridor_launch.py (gui, noise_bias, noise_std).
# The workspace with rota_localization is taken from $TNR_WS (default ~/tp1_ws).
HERE=$(dirname "$(readlink -f "$0")")
source /opt/ros/humble/setup.bash
source "${TNR_WS:-$HOME/tp1_ws}/install/setup.bash"
# VS Code's snap leaks its GTK/GIO environment, which crashes gzclient and rviz2.
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
[ -n "$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR LOCPATH GIO_LAUNCHED_DESKTOP_FILE

"$HERE/stop.sh" > /dev/null
setsid ros2 launch "$HERE/corridor_launch.py" "$@" > /tmp/tnr_corridor_launch.log 2>&1 &
echo $! > /tmp/tnr_corridor.pgid
echo "simulation starting (log: /tmp/tnr_corridor_launch.log), driving in 15 s"
sleep 15
python3 "$HERE/drive_corridor.py" ${LOG:+--log "$LOG"}
echo "done; the simulation keeps running until $HERE/stop.sh"
