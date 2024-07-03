#!/bin/bash

sensors=$(sudo dmesg | grep "usb 1-1.4: ch341-uart converter" | awk -F ' to ' '{print($2)}')

lidar=$(sudo dmesg | grep "usb 1-1.2.3.1: cp210x" | awk -F ' to ' '{print($2)}')

front_lidar=$(sudo dmesg | grep "usb 1-1.2.1: cp210x" | awk -F ' to ' '{print($2)}')

echo /dev/"$front_lidar" > /tmp/front_lidar_nav.conf

echo /dev/"$lidar" > /tmp/lidar_nav.conf

echo /dev/"$sensors" > /tmp/sensors.conf
