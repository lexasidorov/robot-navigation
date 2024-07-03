#!/bin/bash

echo "$0 starting..."

source ~/robot-visual-nav/venv/bin/activate

python3 lidars/front_lidar_service.py reset_ports

echo "$0 finished"
