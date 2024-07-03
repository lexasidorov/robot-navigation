import json
import math
import sys
from math import atan2, pi
from time import sleep, time
import numpy as np
from requests import get
import pickle
from json import load
from sklearn.neural_network import MLPClassifier
# import smbus

try:
    import smbus
    USEGPIO = True
    print("GPIO imported")
except ModuleNotFoundError:
    USEGPIO = False
    print("GPIO NOT imported")


offset_x =  -30.5
offset_y =  24.5
offset_z =  -94.5

class Compass():
    def __init__(self, debug=False):
        print("\n[ INFO ] Init compass ... ", end='', flush=1, file=sys.stderr)
        self.__debug = debug
        self.__calibrated = False
        self.last_z = [0, 0, 0]
        self.last_t = [time(), time(), time()]
        if USEGPIO:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(0x1E, 0x00, 0x70) # Write CRA (00) (8-average, 15 Hz default, normal measurement)
            self.bus.write_byte_data(0x1E, 0x01, 0xA0) # Write CRB (01)
            self.bus.write_byte_data(0x1E, 0x02, 0x00) # Write Mode (02) (Continuous-measurement mode)
            sleep(0.6)
            self.offset_x, self.offset_y, self.offset_z = self.__get_compass_offset()

            print("\n[ INFO ] compass init completed", file=sys.stderr)
            self.current_time = time()

        else:
            print("\n[ ERROR ] No bus found", file=sys.stderr)
            self.z = None

    def __get_compass_offset(self):
        try:
            with open("compass_calibration.json", "r") as f:
                data = load(f)
                if self.__debug: print(data)
                offset_x, offset_y, offset_z = data["offset_x"], data["offset_y"], data["offset_z"]
            if self.__debug: print('offset loaded')
        except:
            offset_x, offset_y, offset_z = 0, 0, 0
            if self.__debug: print('offset load failed')
        if self.__debug: print('offset_x, offset_y, offset_z = ',offset_x, offset_y, offset_z)

        return offset_x, offset_y, offset_z

    @staticmethod
    def __to_int(high, low):
        value = ((high << 8) | low)
        if (value > 32768):
            value -= 65536

        return value

    @staticmethod
    def __to_degrees(rad):
        deg = int(rad * 180 / pi)  # convert into angle
        if deg > 360:
            deg -= 360  # Due to declination check for >360 degree
        elif deg < 0:
            deg += 360  # check for sign

        return deg

    def get_xyz(self): # getting magnetic_intensity
        data = self.bus.read_i2c_block_data(0x1E, 0x03, 6)
        X_M, X_l = data[0], data[1]
        Y_M, Y_l = data[4], data[5]
        Z_M, Z_l = data[2], data[3]
        x = self.__to_int(X_M,X_l) - self.offset_x
        y = self.__to_int(Y_M,Y_l) - self.offset_y
        z = self.__to_int(Z_M,Z_l) - self.offset_z
        xyz = [x, y, z]
        return xyz

    def test_calibrate_die(self):
        sleep(5)
        results= {}
        # turn = get("http://localhost:4998/lft/1/3600")
        for iteration in range(3600):
            turn = get("http://localhost:4998/lft/1/1")
            sleep(0.1)
        # while True:
        #     i = 0
            angle = get("http://localhost:4998/state/get").json()["angle_z"]
            while True:
                if angle > 360:
                    angle -= 360
                elif angle < 0:
                    angle += 360
                else:
                    break
            data = self.bus.read_i2c_block_data(0x1E, 0x03, 6)
            X_M, X_l = data[0], data[1]
            Y_M, Y_l = data[4], data[5]
            Z_M, Z_l = data[2], data[3]
            x = self.__to_int(X_M, X_l) - self.offset_x
            y = self.__to_int(Y_M, Y_l) - self.offset_y
            z = self.__to_int(Z_M, Z_l) - self.offset_z
            xyz = [x,y,z]
            try:
                key = str(round(angle))
                results[key].append(xyz)
                # if len(results[key]) < 15:  results[key].append(xyz)
                    # i +=1
            except:
                results[key] = [xyz]
                # i += 1
            # if i >= 5399:
            #     break
        try:
            json_obj = json.loads(results)
            with open('test_compass_results.json', 'w') as f:
                json.dump(json_obj, f)
            print("json successfully created")
        except:
            print(results)

    def clf_load(self):
        try:
            clf = pickle.load(open('clf.pickle', 'rb'))
            if self.__debug: print('clf.pickle loaded')
            return clf
        except FileNotFoundError:
            print('[ ERROR ] Exception on loading clf.pickle', file=sys.stderr)

    def get_angle(self,clf):
        angle_z = get("http://localhost:4998/state/get").json()["angle_z"]
        clf_angle = np.argmax(clf.predict_proba([self.get_xyz()]))
        if self.__debug: print('angle_z = ',angle_z,'clf_angle = ',clf_angle)
        return clf_angle



# def byte_to_decimal(m , l):
#     d = m * 256 + l
#     if d > 32767: d -= 65536
#     return d

if __name__ == '__main__':
    c = Compass(debug=True)
    # clf = c.clf_load()
    # for x in range (100):
        # c.get_angle(clf)
    c.test_calibrate_die()
    # for i in range(300):
    #     c.compass_analysis()
    #     # c.get_heading()
    #     # print(c.get_heading())
    #     sleep(0.3)