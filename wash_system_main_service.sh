#!/bin/bash

echo "$0 starting..."

while true
do
	# wait for drive, visual, parking service, for lights - not	
	curl -X GET http://localhost:4998/state/get 2> /dev/null &&
	curl -X GET http://localhost:4992/state/get 2> /dev/null &&
	curl -X GET http://localhost:4997/state/get 2> /dev/null && break
	sleep 2
done

init_db=$1

source ~/robot-visual-nav/venv/bin/activate

#START_PID=$$

#echo "$START_PID" > /tmp/running.pid

# web service
export FLASK_APP='web'
export FLASK_DEBUG=0

[[ $1 == "--init-db" ]] && export INIT_FLASK_DB_USER=1

flask run --host '0.0.0.0' --port 5000 --no-debug

echo "$0 started"
