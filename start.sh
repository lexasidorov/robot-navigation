#!/bin/bash

init_db=$1

# lidar service
#python3 lidars/lidar_coords.py &
#LIDAR_PID=$!

# visual navigation service
#python3 visual_nav/visual_coords.py &
#VISUAL_PID=$!

# drive controller service
#python3 gpio/drive_test.py &
#DRIVE_PID=$!
#START_PID=$$

#echo "$START_PID"
#echo "$LIDAR_PID"
#echo "$VISUAL_PID"
#echo "$DRIVE_PID"

#echo "$START_PID" > /tmp/running.pid
#echo "$LIDAR_PID" >> /tmp/running.pid
#echo "$VISUAL_PID" >> /tmp/running.pid
#echo "$DRIVE_PID" >> /tmp/running.pid

# web service
export FLASK_APP='web'
export FLASK_DEBUG=1

[[ $1 == "--init-db" ]] && export INIT_FLASK_DB_USER=1

flask run --host '0.0.0.0' --port 5000 --no-debug 
#WEB_PID=$!
#echo "$WEB_PID"

#echo "$WEB_PID" >> /tmp/running.pid

