import re 
import sys

from json import load, dump
from multiprocessing import Process, Queue
from queue import Empty
from serial import Serial, SerialException
from time import sleep, time

READ_UNTIL_CHARACTER = b'<'
USED_PORTS_JSON = '/tmp/used_ports.json'

class SensorsProcess(Process):

    def __init__(self, q_cmd, q_result, debug=False, do_reset_used_ports=False):
        super().__init__()

        self.__do_debug = debug
        self.__set_initial_values()
        self.__ser, self.__port = self.__find_sensors(do_reset_used_ports)
        self.__configure_queues(q_cmd, q_result)
        self.__init_state_dictionary()


    def __set_initial_values(self):
        self.__port = None
        self.__read_until_character = READ_UNTIL_CHARACTER
        self.__ser = None

    
    def __find_sensors(self, do_reset_used_ports=False):
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
        ports = open('/tmp/sensors.conf').readlines()[0].split('\n')[0]
        for port in [ports]: # temp hardcoded
            try:
                if self.__do_debug: print(f'[ INFO ] trying port {port}...')
                ser = Serial(port, baudrate=115200, timeout=0.1)
                sleep(3)
                ser.write(b'>')
                recieved  = ser.read_until(self.__read_until_character).decode('ascii')
                if self.__do_debug: print(recieved, file=sys.stderr)
                assert len(recieved) > 10, f"[ ERROR ] Can not connect to sensors on port {port}!"
                found_port = port
                if self.__do_debug: print(f"[ INFO ] Successful connection to sensors on {port}!")
                __update_used_ports(used_ports, port)
                break
            except IndexError as e:
                print(e, file=sys.stderr)
            except AssertionError as e:
                print(e, file=sys.stderr)
            except UnicodeDecodeError as e:
                print(e, file=sys.stderr)
            except IOError as e:
                print(e, file=sys.stderr)

        assert found_port, "[ ERROR ] Can not connect to sensors on any port!"

        return ser, found_port

    
    def __configure_queues(self, q_cmd, q_result):
        self.cmd = None
        self.q_cmd = q_cmd
        self.q_result = q_result


    def __set_arduino(self):
        try:
            self.__ser = Serial(self.__port, baudrate=self.__baudrate, timeout=self.__timeout)
        except SerialException as e:
            print("[ ERROR ] ", self.__port, 'failed with %s' % e)
            sys.exit(1)
        except UnicodeDecodeError as e:
            print("[ ERROR ] ", self.__port, 'failed with %s' % e)
            sys.exit(1)

    
    def __init_state_dictionary(self):
        self.state = {
            'v_battery'         : None,
            'nc'                : None,
            'current_lft_motor' : None,
            'current_rgt_motor' : None,
            'frw_lft_ik'        : None,
            'frw_rgt_ik'        : None,
            'bkw_lft_ik'        : None,
            'lft_frw_ik'        : None,
            'lft_bkw_ik'        : None,
            'bkw_rgt_ik'        : None,
            'rgt_bkw_ik'        : None,
            'rgt_frw_ik'        : None,
            'running'           : False,
        }

    
    def run(self):
        while 1:
            try:
                self.cmd = self.q_cmd.get(block=False)
                if self.cmd['action'] == 'stop':
                    self.__ser.close()
                    break
                elif self.cmd['action'] == 'state':
                    self.__get_state()
            except Empty:
                pass
            self.__update_state()

    
    def __get_state(self):
        self.q_result.put({**self.state})


    def __update_state(self):
        parsed = []
        try:
            self.__ser.write(b'>')
            recieved  = self.__ser.read_until(self.__read_until_character).decode('ascii').replace('\n',' ')
            data = re.findall(r'(.*?)<', recieved)[-1]
            assert data, f'wrong packet has come ({recieved})!'
            parsed = [int(x) for x in re.findall(r'(\d+)',data)]
            self.state = {
                'v_battery'         : round(parsed[0] * 0.0261489, 2),
                'nc'                : parsed[1],
                'current_lft_motor' : round(parsed[2] / 57.692307, 2),
                'current_rgt_motor' : round(parsed[3] / 57.692307, 2),
                'frw_lft_ik'        : 0 if parsed[ 4] else 1,
                'frw_rgt_ik'        : 0 if parsed[ 5] else 1,
                'bkw_lft_ik'        : 0 if parsed[ 6] else 1,
                'lft_frw_ik'        : 0 if parsed[ 7] else 1,
                'lft_bkw_ik'        : 0 if parsed[ 8] else 1,
                'bkw_rgt_ik'        : 0 if parsed[ 9] else 1,
                'rgt_bkw_ik'        : 0 if parsed[10] else 1,
                'rgt_frw_ik'        : 0 if parsed[11] else 1,
                'running'           : 1,
            }
            open('/tmp/sensors.log', 'a').write(f'{round(time(), 4)}, {self.state["frw_lft_ik"]}, {self.state["frw_rgt_ik"]}; \n')
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            if self.__do_debug: print(f"[ INFO ] {e} {parsed} TEST TEST D", file=sys.stderr)
            

if __name__ == '__main__':
    q_cmd, q_result = Queue(), Queue()
    sensors_proc = SensorsProcess(q_cmd, q_result, debug=1, do_reset_used_ports=1)
    sensors_proc.start()

    while 1:
        sleep(1)
        try:
            q_cmd.put({'action':'state'})
            print(time(), " ", q_result.get())
        except KeyboardInterrupt:
            break

    q_cmd.put({'action':'EXIT'})

    sensors_proc.join()
