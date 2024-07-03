import math
import numpy as np
import os
import sys
import time
import ydlidar

ydlidar.os_init();
ports = ydlidar.lidarPortList();
print(f'[ INFO ] {ports.items()}', file=sys.stderr)
try:
    port = sys.argv[1];
except IndexError:
    port = "/dev/ttyUSB1"
#for key, value in ports.items():
#    port = value;
laser = ydlidar.CYdLidar();
print( ydlidar.YDLIDAR_TYPE_SERIAL )
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200);
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);

ret = laser.initialize();
print(f'[ INFO ] {ret, dir(ret)}', file=sys.stderr)
if ret:
    ret = laser.turnOn();
    scan = ydlidar.LaserScan();
    print(444444444444444444444444444444444444444444444, scan)
    res = {}
    iii = 0
    while ret and ydlidar.os_isOk() :
        r = laser.doProcessSimple(scan);
        print(1111111111111111111111111111111111111111, laser, r)
        if r:
            # print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
            iii += 1
            for point in scan.points:
                res.update({int(np.degrees(point.angle)): float(point.range)*1000})
                # print(point.angle)
                # print(np.degrees(point.angle))
                # print(int(np.degrees(point.angle)))
            if iii == 30:
                break
        else :
            print("Failed to get Lidar Data")
        time.sleep(0.05);
    print(res)
    laser.turnOff();
laser.disconnecting();

