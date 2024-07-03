import numpy as np
import PyLidar3
import sys

from json import load, dump
from multiprocessing import Process, Queue
from queue import Empty
from time import sleep, time

USED_PORTS_JSON = '/tmp/used_ports.json'
LIDAR = {'model_number': '6', 'firmware_version': '1.5', 'hardware_version': '1', 'serial_number': '201903300'}


class LidarNav(Process):

    def __init__(self, q_cmd, q_result, debug=False, do_reset_used_ports=False):
        super().__init__()

        self.__do_debug = debug
        self.__lidar, self.__port = self.__find_lidar(do_reset_used_ports)
        self.__configure_queues(q_cmd, q_result)
        self.__init_state_dictionary()


    def __del__(self):
        try:
            if self.__do_debug: print(f"[ INFO ] Lidar disconnecting from {self.__port}...")
            # self.__lidar.StopScanning()
            self.__lidar.Disconnect()
            if self.__do_debug: print(f"[ INFO ] Lidar disconnected from {self.__port}!")
        except Exception as e:
            print(f"[ ERROR ] Error {e}!")
        

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

        # for port in { f"/dev/ttyUSB{n}" for n in range(10) } - used_ports:
        ports = open('/tmp/lidar_nav.conf').readlines()[0].split('\n')[0]
        for port in [ports]:
            try:
                if self.__do_debug: print(f"[ INFO ] Trying port {port}...")
                lidar = PyLidar3.YdLidarX4(port)
                if not lidar.Connect(): lidar.Reset()
                info = lidar.GetDeviceInfo()
                assert info == LIDAR, f"[ ERROR ] Can not connect to lidar on port {port}!"
                found_port = port
                if self.__do_debug: print(f"[ INFO ] Successful connection to {found_port}, {info}!")
                __update_used_ports(used_ports, port)
                break
            except AssertionError as e:
                print(e, file=sys.stderr)
            except IOError as e:
                print(e, file=sys.stderr)
            except Exception as e:
                print(e, file=sys.stderr)
            except TimeoutException as e:
                print(f"[ INFO ] Timed out! {e}", file=sys.stderr)
            except FileNotFoundError as e:
                print(e, file=sys.stderr)

        assert found_port, "[ ERROR ] Can not connect to lidar on any port!"

        return lidar, found_port


    def __configure_queues(self, q_cmd, q_result):
        self.cmd = None
        self.q_cmd = q_cmd
        self.q_result = q_result


    def __init_state_dictionary(self):
        self.state = {
            'distances'   : None,
            'lines'       : None,
            'running'     : 0,
        }


    def run(self):
        self.__get_lidar_data()


    def __get_state(self):
        self.q_result.put({**self.state})


    def __replace_zeros(self, arr):
        return [12000 if x == 0 else x for x in arr]


    def __update_state(self, data):
        original_distances = data[0]
        fixed_distances = self.__replace_zeros(original_distances)
        fixed_distances.reverse()
        self.state = {
            'distances'   : fixed_distances,
            'lines'       : data[1],
            'running'           : 1,
        }


    def __get_lidar_data(self):
        gen = self.__lidar.StartScanning()

        res = []

        l_idx = 0
        r = np.zeros((3,360), dtype=np.int16)  # Каждая строка - расстояния от 0 до 359 (Всего 10 измерений)
        while True:
            try:
                self.cmd = self.q_cmd.get(block=False)
                if self.cmd['action'] == 'EXIT' or self.cmd['action'] == 'stop':
                    if self.__do_debug: print(f"[ INFO ] Lidar disconnecting from {self.__port}...")
                    self.__lidar.StopScanning()
                    self.__lidar.Disconnect()
                    if self.__do_debug: print(f"[ INFO ] Lidar disconnected from {self.__port}!")
                    break
                elif self.cmd['action'] == 'state':
                    self.__get_state()
            except Empty:
                pass
            r = np.roll(r, 1, axis=0)
            R = next(gen)
            r[0] = np.array(list(R.values()))
            lidar_res_median_arr = np.median(r, axis=0)  # Медианные расстояния от 0 до 359 среди 10 измерения среди 10 измерений
            eq = self.__get_lines(lidar_res_median_arr, 1000000)

            lidar_res_median_arr = self.__get_shifted_coords(12, lidar_res_median_arr, 135)
            lidar_res_median_arr = lidar_res_median_arr[:,1:].reshape((360))
            # eq = sorted([x for x in eq], key=lambda x: x[2])
            res_spl = [np.median(x) for x in np.array_split(lidar_res_median_arr, 12)]
            if len(res) > 9:
                res = res[1:]
                res += [res_spl]
            else:
                res += [res_spl]

            sleep(0.01)

            self.__update_state((list(np.median(res, axis=0)), eq))
            l_idx += 1
        
        return res


    def __pol2cart(self, rho, phi):  # расстояние, угол
        x = rho * np.cos(np.radians(phi))
        y = rho * np.sin(np.radians(phi))
        return (float(x), float(y))


    def __get_shifted_coords(self, required_sectors: int, lidar_res, addition_shift=0):
        total_angles = lidar_res.shape[0]
        shift = round(total_angles / required_sectors / 2) + addition_shift
        # print(shift, file=sys.stderr)
        counter = 0
        result = []
        for index in range(-shift, total_angles - shift):
            result.append([counter, lidar_res[index]])
            counter += 1
        return np.array(result, dtype=np.int16)  # [0-360, расстояние] shape = (360, 2)


    # lidar_res - Медианные расстояния от 0 до 359 среди 10 измерений
    # coeff - 100
    def __get_lines(self, lidar_res, coeff):
        # print(lidar_res.shape)
        # pts = np.array([ (phi,lidar_res[phi]) for phi in range(360) ], dtype=np.int16)  # [0-360, расстояние] shape = (360, 2)
        pts = self.__get_shifted_coords(8, lidar_res, 135)

        pts_spl = np.array_split(pts, 8)[::2]  # 8 секторов, выкидываем каждый второй, получаем 4 сектора, каждый shape = (45, 2)
        final_res = {}
        sector_cnt = 0
        x_x = []
        y_y = []
        for sector in pts_spl:  # sector - shape = (45, 2), (угол, расстояние)
            X, Y = [], []
            i = 0
            # data is (angle, distance)
            for data in sector:  # data - (угол, расстояние), на выходе декартовые X, Y для сектора, больше 0
                if data[1] > 0:
                    X += [self.__pol2cart(data[1], data[0])[0]]  # Из полярных координат в декартовые. Центр координат - центр робота
                    Y += [self.__pol2cart(data[1], data[0])[1]]
                i += 1

            sector_cnt += 1
            try:
                k, acc = np.polyfit(X, Y, 1, cov=1)
            except (TypeError, ValueError, np.linalg.LinAlgError) as e:
                if self.__do_debug: print(f'[ INFO ] {e}, X: {X}, Y: {Y}')
                k, acc = [0, 0], np.array([0])
            final_res[str(sector_cnt)] = (float(np.degrees(np.arctan(k[0]))), float(np.mean(acc)))
            x_x += [X]
            y_y += [Y]
        # print(f'[ INFO_LINES ] final res: {final_res}',file=sys.stderr)
        return final_res


if __name__ == '__main__':
    q_cmd, q_result = Queue(), Queue()
    lidar_proc = LidarNav(q_cmd, q_result, debug=True, do_reset_used_ports=True)
    lidar_proc.start()

    while 1:
        sleep(1)
        try:
            q_cmd.put({'action':'state'})
            r = q_result.get()
            print(f'{time()} {" ".join(( str(x) for x in r["distances"]))} {" ".join(( str(x[1]) for x in r["lines"]))}')
        except KeyboardInterrupt:
            break

    q_cmd.put({'action':'EXIT'})

    lidar_proc.join()
