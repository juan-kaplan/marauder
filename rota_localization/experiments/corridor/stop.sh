#!/bin/bash
# Stops the simulation started by run.sh (the whole process group).
PGID_FILE=/tmp/tnr_corridor.pgid
[ -f $PGID_FILE ] || exit 0
kill -INT -- -"$(cat $PGID_FILE)" 2>/dev/null
sleep 4
kill -9 -- -"$(cat $PGID_FILE)" 2>/dev/null
rm -f $PGID_FILE
echo "simulation stopped"
