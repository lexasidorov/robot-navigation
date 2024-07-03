#!/bin/bash

echo "$0 starting..."

while true
do
	# wait for sensors service
	curl -X GET http://localhost:4993/state/get 2> /dev/null && break
	sleep 2
done


source ~/robot-visual-nav/venv/bin/activate

python3 lidars/lidar_service.py 

echo "$0 finished"
