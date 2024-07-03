import numpy as np
import sys
import ydlidar

from json import load, dump
from queue import Queue, Empty
from threading import Thread
from time import sleep, time

TEST_MODE = 0
USED_PORTS_JSON = '/tmp/used_ports.json'

class LidarNavX2(Thread):

    def __init__(self, q_cmd, q_result, debug=False, do_reset_used_ports=False):
        super().__init__()

        self.__do_debug = debug
        ydlidar.os_init()
        self.__lidar, self.__port = self.__find_lidar(do_reset_used_ports)
        self.__configure_queues(q_cmd, q_result)
        self.__init_state_dictionary()


    def __find_lidar(self, do_reset_used_ports=False):
        def __reset_used_ports():
            dump([], open(USED_PORTS_JSON, 'w'), ensure_ascii=0, indent=4)

        def __get_used_ports():
            return set(load(open(USED_PORTS_JSON)))

        def __update_used_ports(used_ports, port):
            used_ports |= {port}
            dump(list(used_ports), open(USED_PORTS_JSON, 'w'), ensure_ascii=0, indent=4)

        if do_reset_used_ports: __reset_used_ports()
        used_ports = __get_used_ports()

        found_port = None
        
        if not TEST_MODE:
            # for port in { f"/dev/ttyUSB{n}" for n in range(10) } - used_ports :
            ports = open('/tmp/front_lidar_nav.conf').readlines()[0].split('\n')[0]
            for port in [ports]:
                try:
                    ydlidar.lidarPortList()
                    lidar = ydlidar.CYdLidar()
                    lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
                    lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
                    lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
                    lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
                    lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
                    lidar.setlidaropt(ydlidar.LidarPropSampleRate, 9)
                    lidar.setlidaropt(ydlidar.LidarPropSingleChannel, True)
                    ret = lidar.initialize()
                    if not ret:
                        raise AssertionError(f"[ INFO ] Can not connect to lidar on port {port} on lidar.initialize()!")
                    else:
                        ret = lidar.turnOn()
                        if not ret:
                            raise AssertionError(f"[ INFO ] Can not connect to lidar on port {port} on lidar.turnOn()!")
                        else:
                            pass
                    found_port = port
                    if self.__do_debug: print(f"[ INFO ] Successful connection to {found_port}!")
                    __update_used_ports(used_ports, port)
                    break
                except AssertionError as e:
                    print(e, file=sys.stderr)

            assert found_port, "[ ERROR ] Can not connect to lidar on any port!"

        return lidar, found_port


    def __configure_queues(self, q_cmd, q_result):
        self.cmd = None
        self.q_cmd = q_cmd
        self.q_result = q_result


    def __init_state_dictionary(self):
        self.state = {
            'distances_x2'   : None,
            'running'     : 0,
        }


    def run(self):
        self.__get_lidar_data()


    def __get_state(self):
        self.q_result.put({**self.state})


    def __replace_zeros(self, arr):
        return [12000 if x == 0 else x for x in arr]


    def __update_state(self, data):
        # original_distances = data[0]
        fixed_distances = self.__replace_zeros(data)
        fixed_distances.reverse()
        self.state = {
            'distances_x2'   : fixed_distances,
            'running'        : 1,
        }


    def __get_lidar_data(self):
        scan = ydlidar.LaserScan()
        res = []
        R = {}
        r = np.zeros((3,180), dtype=np.int16)  # Каждая строка - расстояния от 0 до 359 (Всего 10 измерений)
        while scan and ydlidar.os_isOk():
            try:
                self.cmd = self.q_cmd.get(block=False)
                if self.cmd['action'] == 'EXIT' or self.cmd['action'] == 'stop':
                    break
                elif self.cmd['action'] == 'state':
                    self.__get_state()
            except Empty:
                pass
            r = np.roll(r, 1, axis=0)
            r_lidar = self.__lidar.doProcessSimple(scan)
            if r_lidar:
                # R -> {0: 2350, 1: 340, ... 359: 1268}
                for point in scan.points:
                    R.update({round(np.degrees(point.angle))+179: float(point.range)*1000})
                    # R.update({point.angle: float(point.range)*1000})
                # json.dump(fp=open('lidar_log.json', 'a'), indent=4)
                try:
                    # print(R)
                    res_tmp = np.zeros((360))
                    for k,v in R.items():
                        # print(v)
                        res_tmp[k] = v
                    left = np.flip(res_tmp[:91])
                    right = np.flip(res_tmp[271:])
                    # print(res_tmp[270:],len(res_tmp[270:]))
                    r[0] = np.concatenate([left, right])
                    # for idx in range(r[0].shape[0]):
                    #     if 90 < idx < 270:
                    #         r[0][idx] = 0
                    #     # ^ leave only fwd 180 -> we need 0-90; 270-360;
                except IndexError:
                    print("[ INFO ] Index error with lidar data", file=sys.stderr)
            else:
                print('[ INFO ] Error getting data from lidar x2', file=sys.stderr)
            sleep(0.05)
            lidar_res_median_arr = np.average(r, axis=0)  # Медианные расстояния от -90 до 90 среди 10 измерения среди 10 измерений

            res_spl = [np.median(x) for x in np.array_split(lidar_res_median_arr, 6)]
            if len(res) > 9:
                res = res[1:]
                res += [res_spl]
            else:
                res += [res_spl]

            sleep(0.01)
            # print(res, file=sys.stderr)
            # print(list(np.median(res, axis=0)))
            self.__update_state(list(np.median(res, axis=0)))

        self.__lidar.turnOff()
        self.__lidar.disconnecting()

        return res


    def __pol2cart(self, rho, phi):  # расстояние, угол
        x = rho * np.cos(np.radians(phi-45/2))
        y = rho * np.sin(np.radians(phi-45/2))
        return (float(x), float(y))


    def __get_shifted_coords(self, required_sectors: int, lidar_res):
        total_angles = lidar_res.shape[0]
        shift = round(total_angles / required_sectors / 2)  # Сдвиг в минус

        counter = 0
        result = []
        for index in range(-shift, total_angles - shift):
            result.append([counter, lidar_res[index]])
            counter += 1
        return np.array(result, dtype=np.int16)  # [0-360, расстояние] shape = (360, 2)


if __name__ == '__main__':
    q_cmd, q_result = Queue(), Queue()
    lidar_proc = LidarNavX2(q_cmd, q_result)
    lidar_proc.start()

    while 1:
        sleep(1)
        try:
            q_cmd.put({'action':'state'})
            r = q_result.get()
            print(f'{time()} {" ".join(( str(x) for x in r["distances"]))}')
        except KeyboardInterrupt:
            break

    q_cmd.put({'action':'EXIT'})

    lidar_proc.join()
