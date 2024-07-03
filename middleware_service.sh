#!/bin/bash

echo "$0 starting..."

while true
do
	# wait for main wash web interface service
	curl -X GET http://localhost:5000/ 2> /dev/null && break
	sleep 2
done

[ "$1" == "stop" ] && {
	PID=$( ps fax | grep middleware | cut -d ' ' -f 1 )
	kill $PID
} || {
	source ~/robot-visual-nav/venv/bin/activate

	python3 middleware_collector.py
}
echo "$0 started"
