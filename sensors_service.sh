#!/bin/bash

echo "$0 starting..."

while true
do
	# wait for front lidar service
	curl -X GET http://localhost:4996/state/get 2> /dev/null && break
	sleep 2
done

source ~/robot-visual-nav/venv/bin/activate

python3 arduino/sensors_service.py 

echo "$0 finished"
