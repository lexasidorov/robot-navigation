#!/bin/bash

source /home/pi/robot-visual-nav/venv/bin/activate
for x in 1 2 3 4 5 6 7 8 9 10 11 12
do
	echo "ФОТКАЮЮЮЮ"
	python3 /home/pi/robot-visual-nav/distance_measurement_and_calibration/take_shot.py 0 1920 1080 &&
        mv /tmp/out_0_1920_1080.jpeg "/tmp/out_0_1920_1080_$x.jpeg"
	echo "ЕДУУУУУУУ"
	python3 /home/pi/robot-visual-nav/gpio/drv_process.py back 300
	echo "СТОЮЮЮЮЮЮ"
	sleep 30
	echo "$x"
done

