import math
import sys
import numpy as np
from pathlib import Path
import pickle
from time import sleep, time
from json import load, dump
from requests import get
import csv

STD_GZ = 6
TO_DEGREES = 14 # 13.75
TO_DEGREES = 13.75
TO_DEGREES = 13.05
TO_DEGREES = 14.2
WW = 3
pickle_path = Path(Path.cwd(),'gpio','clf.pickle')
declination = 0.28390721          #define declination angle of location where measurement going to be done





try:
    import smbus
    USEGPIO = True
except ModuleNotFoundError:
    USEGPIO = False

def to_int(d0, d1):
    x = d0*256 +d1
    if x > 32767: x -= 65536
    return x



class GyroscopeError(Exception):
    pass


class Gyroscope():

    def __init__(self, debug=False):
        print("[ INFO ] Init gyro ... ", end='', flush=1, file=sys.stderr)
        self.__debug = debug
        self.__calibrated = False
        # self.__last_x = np.zeros(3)
        # self.__last_y = np.zeros(3)
        self.__last_gz = np.zeros(WW)
        self.__last_t = np.array([time()]*WW)
        if USEGPIO:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(0x68, 0x3E, 0x00)
            self.bus.write_byte_data(0x68, 0x15, 0x07)
            self.bus.write_byte_data(0x68, 0x16, 0x19)
            self.bus.write_byte_data(0x68, 0x17, 0x00)
            self.bus.write_byte_data(0x1E, 0x00, 0x70)  # Write CRA (00) (8-average, 15 Hz default, normal measurement)
            self.bus.write_byte_data(0x1E, 0x01, 0xA0)  # Write CRB (01)
            self.bus.write_byte_data(0x1E, 0x02, 0x00)  # Write Mode (02) (Continuous-measurement mode)
            # self.X_axis_H = 0x03  # Address of X-axis MSB data register
            # self.Z_axis_H = 0x05  # Address of Z-axis MSB data register
            # self.Y_axis_H = 0x07  # Address of Y-axis MSB data register
            self.declination = 0.28390721  # define declination angle of location where measurement going to be done
            self.offset_x, self.offset_y, self.offset_z = self.__get_compass_offset()
            # self.bus.write_byte_data(0x0d, 0x02, 0x00)
            sleep(0.6)
            print("completed", file=sys.stderr)
            self.current_time = time()
            self.S_ca, self.S_gz, self.gz, self.avg_gz = 0, 0, 0, 0
            # self.angle_x, self.angle_y, self.angle_z = 0, 0, 0
            self.c_angle, self.g_angle, self.angle = 0, 0, 0
            self.clf = self.clf_load()
            self.c_angle_0 = self.calibrate_compass(50)
            self.__calibrate__()
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


    # def __read_compass_data(self,axis):
    #     high = self.bus.read_byte_data(0x1E, axis)
    #     low = self.bus.read_byte_data(0x1E, axis + 1)
    #     value = ((high<<8) | low)
    #     if (value > 32768):
    #         value -= 65536
    #
    #     return value

    # def __get_compass_angle(self):
    #     x = self.__read_compass_data(self.X_axis_H)
    #     y = self.__read_compass_data(self.Y_axis_H)
    #     heading = math.atan2(y, x) + self.declination
    #
    #     # Due to declination check for >360 degree
    #     if (heading > 2 * math.pi):
    #         heading = heading - 2 * math.pi
    #
    #     # check for sign
    #     if (heading < 0):
    #         heading = heading + 2 * math.pi
    #
    #     # convert into angle
    #     heading_angle = int(heading * 180 / math.pi)
    #
    #     return heading_angle


    def __calibrate__(self):
        print("[ INFO ] Get average gz ... ", end='', flush=1, file=sys.stderr)
        # a_gx = np.zeros(10000)
        # a_gy = np.zeros(10000)
        a_gz = np.zeros(10000)
        for i in range(len(a_gz)):
            # a_gx[i], a_gy[i],
            a_gz[i] = self.__get_rot_acc__(False)[0]
            # sleep(0.1)
        # self.avg_gx = a_gx.mean()
        # self.avg_gy = a_gy.mean()
        self.avg_gz = a_gz.mean()
        std = np.std(a_gz)
        assert std < STD_GZ, f'[ ERROR ] Gyroscope fault! Exiting! std = {std}'
        if self.__debug:
            np.savetxt('/tmp/gz', a_gz)
            print(self.avg_gz)

        print(f" completed {self.avg_gz}.", file=sys.stderr)
        self.__calibrated = True

    def clf_load(self):
        try:
            if self.__debug:    print('pickle path = ',pickle_path)
            clf = pickle.load(open(pickle_path, 'rb'))
            if self.__debug:    print('clf.pickle loaded')
            return clf
        except FileNotFoundError:
            if self.__debug:    print('Can not load clf.pickle')
            print('[ ERROR ] Can not load clf.pickle', file=sys.stderr)

    def calibrate_compass(self, iters):
        if self.__debug:    print('[ compass ] Calibrating started...')
        c = []
        for i in range(iters):
            c_data = self.__try_read_bus()[1]
            cx = to_int(c_data[0], c_data[1]) - self.offset_x
            cz = to_int(c_data[2], c_data[3]) - self.offset_z
            cy = to_int(c_data[4], c_data[5]) - self.offset_y
            c_angle = np.argmax(self.clf.predict_proba([[cx, cy, cz]]))
            if self.__debug:    print('calibrating... c_angle = ', c_angle)
            c += [c_angle]
        if self.__debug:
            print('[compass] finished | c_angle_0 = ', np.average(np.array(c)))
        return np.average(np.array(c))



    def __try_read_bus(self, n=3):
        i, j, msg = 0, 0, []
        while i < n:
            i += 1
            try:
                # data = self.bus.read_i2c_block_data(0x68, 0x1D, 6)
                g_data = self.bus.read_i2c_block_data(0x68, 0x1D, 6)
                c_data = self.bus.read_i2c_block_data(0x1E, 0x03, 6)
                # print('bus data read successful') if self.__debug else None
                # print('compass data ', c_data) if self.__debug else None
                break
            except Exception as e:
                j += 1
                msg += [e]
                continue
        if j > n:
            raise GyroscopeError(f'[ ERROR ] max i2c retries {j} reached, ({",".join(msg)})')

        # return data
        return g_data, c_data

    def __try_read_busA(self, n=3):
        i, j, msg, data = 0, 0, [], 0
        while i < n:
            i += 1
            try:
                data = self.bus.read_i2c_block_data(0x0d, 0x1D, 6)

                break
            except Exception as e:
                j += 1
                msg += [e]
                continue
        if j > n:
            raise GyroscopeError(f'[ ERROR ] max i2c retries {j} reached, ({",".join(msg)})')

        if self.__debug: print(f'[ INFO ] gyro {data} {i} {j} {n}', file=sys.stderr)

        return data


    def __get_rot_acc__(self, is_moveless=False):
        if USEGPIO:
            try:
                g_data, c_data = self.__try_read_bus()
                gz = to_int(g_data[4], g_data[5])  # Gyro angle
                gz /= TO_DEGREES
                gz = self.avg_gz if is_moveless else gz
                cx = to_int(c_data[0], c_data[1]) - self.offset_x
                cz = to_int(c_data[2], c_data[3]) - self.offset_z
                cy = to_int(c_data[4], c_data[5]) - self.offset_y
                # if self.__debug: print('predict_proba ',[cx, cy, cz])
                c_angle = np.argmax(self.clf.predict_proba([[cx, cy, cz]]))
                print(f' predict_proba {cx, cy, cz}') if self.__debug else None
                c_angle -= self.c_angle_0
            except (GyroscopeError, TypeError) as e:
                print(f'[ ERROR ] gyro {e}', file=sys.stderr)
                try:
                    # xp = np.poly1d(np.polyfit(self.__last_t, self.__last_x, 1))
                    # yp = np.poly1d(np.polyfit(self.__last_t, self.__last_y, 1))
                    zp = np.poly1d(np.polyfit(self.__last_t, self.__last_z, 1))
                    # x, y, z = xp(time()), yp(time()), zp(time())
                    gz = zp(time())
                except Exception as e:
                    print(f'[ ERROR ] polyfit {e}', file=sys.stderr)
        else:
            z = (0,0,0)
        # return x, y, z
        return gz, c_angle

    def __get_z_smooth__(self, z_value=0, is_moveless=False):
        dt = time() - self.current_time
        self.current_time = time()
        gz, c_angle = self.__get_rot_acc__(is_moveless)
        gz -= self.avg_gz
        self.S_gz += gz * dt - self.gz
        self.gz = self.S_gz / WW
        self.g_angle += z_value + self.gz
        self.__last_gz = np.roll(self.__last_gz, 1);
        self.__last_gz[0] = self.g_angle
        self.__last_t = np.roll(self.__last_t, 1);
        self.__last_t[0] = time()
        # self.S_ca += c_angle - self.c_angle
        # self.c_angle = self.S_ca / WW
        self.c_angle = c_angle
        # if self.__debug:
            # print('t, dt', self.current_time, dt)
            # print('gz', gz)
            # print('avg_gz', self.avg_gz)
            # print('g_angle', self.g_angle)
            # print('c_angle', self.c_angle)
            # print('last_z', self.last_z)
            # print('last_t', self.last_t)
        return self.g_angle, self.c_angle
    # def __get_z_smooth__(self, z_value=0, is_moveless=False):
    #     if USEGPIO:
    #         dt = time() - self.current_time
    #         self.current_time = time()
    #         x, y, z = self.__get_rot_acc__(is_moveless)
    #         self.angle_x += (x - self.avg_gx) * dt
    #         self.angle_y += (y - self.avg_gy) * dt
    #         self.angle_z += (z - self.avg_gz) * dt
    #         self.__last_x = np.roll(self.__last_x, 1); self.__last_x[0] = self.angle_x
    #         self.__last_y = np.roll(self.__last_y, 1); self.__last_y[0] = self.angle_y
    #         self.__last_z = np.roll(self.__last_z, 1); self.__last_z[0] = self.angle_z
    #         self.__last_t = np.roll(self.__last_t, 1); self.__last_t[0] = time()
    #         if self.__debug:
    #             print('t, dt', self.current_time, dt)
    #             print('x,y,z', x, y, z)
    #             print('avg_gz', self.avg_gz)
    #             print('ax', self.angle_x)
    #             print('ay', self.angle_y)
    #             print('az', self.angle_z)
    #             # print('last_z', self.last_z)
    #             # print('last_t', self.last_t)
    #     else:
    #         self.angle_x = None
    #         self.angle_y = None
    #         self.angle_z = None
    #     return self.angle_x, self.angle_y, self.angle_z


    # def __get_z_smooth__A(self, z_value=0, is_moveless=False):
    #     if USEGPIO:
    #         dt = time() - self.current_time
    #         self.current_time = time()
    #         z = self.__get_rot_acc__(is_moveless) - self.avg_gz
    #         self.S_gz += z * dt - self.gz
    #         self.gz = self.S_gz / 3
    #         self.angle_z += z_value + self.gz
    #         self.__last_z = np.roll(self.__last_z, 1); self.__last_z[0] = self.angle_z
    #         self.__last_t = np.roll(self.__last_t, 1); self.__last_t[0] = time()
    #         if self.__debug:
    #             print('t, dt', self.current_time, dt)
    #             print('z', z)
    #             print('avg_gz', self.avg_gz)
    #             print('S_gz', self.S_gz)
    #             print('gz', self.gz)
    #             print('a', self.angle_z)
    #             # print('last_z', self.last_z)
    #             # print('last_t', self.last_t)
    #     else:
    #         self.angle_z = None
    #     return self.angle_z, z


    def get_z(self, z_value=0, is_moveless=False):
        return self.__get_z_smooth__(z_value, is_moveless)[0]
        # return self.__get_z_smooth__(z_value, is_moveless)[2]


    def get_angle(self, z_value=0, is_moveless=False):
        return self.__get_z_smooth__(z_value, is_moveless)

    # def get_new(self):
    #     return self.__get_compass_angle()

    def calibrate_compass_offset(self):
        x_max, y_max, z_max, x_min, y_min, z_min = 0, 0, 0, 0, 0, 0
        start_turning = get("http://localhost:4998/lft/1/3600")
        print('************** CALIBRATION STARTED ***************')
        for i in range(3600):
            c_data = self.__try_read_bus()[1]
            x = to_int(c_data[0], c_data[1])
            z = to_int(c_data[2], c_data[3])
            y = to_int(c_data[4], c_data[5])
            if x > x_max:
                x_max = x
            elif x < x_min:
                x_min = x
            if y > y_max:
                y_max = y
            elif y < y_min:
                y_min = y
            if z > z_max:
                z_max = z
            elif z < z_min:
                z_min = z
            print([x_max, y_max, z_max, x_min, y_min, z_min])
        offset_x, offset_y, offset_z = (x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2
        print("************** CALIBRATION DONE ***************")
        print('offset_x = ', offset_x, 'offset_y = ', offset_y, 'offset_z = ', offset_z)
        try:
            with open("compass_calibration.json", "w") as f:
                dump({'offset_x': offset_x, 'offset_y': offset_y, 'offset_z': offset_z}, f)
            print("Results have been written to compass_calibration.json")
        except:
            print('Can\'t write to file!')
        return offset_x, offset_y, offset_z



    def __repr__(self):
        if self.__calibrated:
            g_angle, c_angle = self.__get_z_smooth__()
        else:
            g_angle, c_angle = self.__get_rot_acc__()
        return f"{g_angle} {c_angle} {self.avg_gz}"

	
    def check_compass_angle(self):
        with open ('check_compass.csv','w',newline = '') as csvfile:
            fieldnames = ['Gyro','Compass']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for i in range (72):
                for n in range(3):
                    g_angle, c_angle = self.get_angle()
                    writer.writerow({'Gyro': g_angle, 'Compass': c_angle})
                    print('Gyro', g_angle, 'Compass', c_angle) if self.__debug else None
                    sleep(0.1)
                turn = get("http://localhost:4998/lft/1/10")
                sleep(2)
            csvfile.close()
            print('finished') if self.__debug else None
	


if __name__ == '__main__':
    g = Gyroscope(debug=1)
    # g.check_compass_angle()
    # g.calibrate_compass_offset()  # Needed for offset calibration. DO only once, when new compass chip installed
    for i in range(1000):
        # print('gyro =', g.get_angle()[0], 'compass =', g.get_angle()[1],'angle_0 = ', g.c_angle_0)
        print('gyro =', g.get_angle()[0], 'compass =', g.get_angle()[1])
        sleep(0.2)  

