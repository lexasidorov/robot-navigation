import re 
import sys

from json import load, dump
from multiprocessing import Process, Queue
from queue import Empty
from serial import Serial, SerialException
from time import sleep, time

READ_UNTIL_CHARACTER = b'$'
USED_PORTS_JSON = '/tmp/used_ports.json'

class MainBoardProcess(Process):

    def __init__(self, q_cmd, q_result, debug=False, do_reset_used_ports=False):
        super().__init__()

        self.__do_debug = debug
        self.__set_initial_values()
        self.__ser, self.__port = self.__find_mainboard(do_reset_used_ports)
        self.__configure_queues(q_cmd, q_result)
        self.__init_state_dictionary()


    def __set_initial_values(self):
        self.__port = None
        self.__read_until_character = READ_UNTIL_CHARACTER
        self.__ser = None

    
    def __find_mainboard(self, do_reset_used_ports=False):
        def __reset_used_ports():
            dump(open(USED_PORTS_JSON, 'w'), [], ensure_ascii=0, ident=4)

        def __get_used_ports():
            return set(load(open(USED_PORTS_JSON)))

        def __update_used_ports(used_ports, port):
            used_ports |= {port}
            dump(open(USED_PORTS_JSON, 'w'), list(used_ports), ensure_ascii=0, ident=4)

        if do_reset_used_ports: __reset_used_ports()
        used_ports = __get_used_ports()
        
        found_port = None

        for port in [ f"/dev/ttyUSB{n}" for n in range(10) ]:
            try:
                if self.__do_debug: print(f'[ INFO ] trying port {port}...')
                ser = Serial(port, baudrate=115200, timeout=0.1)
                sleep(3)
                recieved  = ser.read_until(self.__read_until_character)
                recieved  = ser.read_until(self.__read_until_character).decode('ascii')
                # print(recieved, file=sys.stderr)
                assert len(recieved) > 10, f"[ ERROR ] Can not connect to mainboard on port {port}!"
                found_port = port
                if self.__do_debug: print(f"[ INFO ] Successful connection to mainboard on {port}!")
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

        assert found_port, "[ ERROR ] Can not connect to mainboard on any port!"
        open('/var/used_ports', 'a').write(f'{ser}, {found_port}\n')

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
        self.state_to_mb = {
            'robot_state'       : None,
            'dump_water'        : None,
            'dump_clear_water'  : None,
            'is_charging'       : None,
            'fill_water'        : None,
            'lights_state'      : None,
            'pump_mode'         : None,
            'undock'            : None,
            'crc'               : None,
            'running'           : None,
        }
        self.state_from_mb = {
            'robot_state'       : None,
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
        self.q_result.put({**self.state_from_mb})


    def __update_state(self):
        
        def find_groups(in_str):
            r = re.compile('^\$(.{3})(.{5}){1}(.{7})(.{2})(.{2})(.{3})(.{2})(.{10})\\r')
            #string = '$01a7823401111102023023801с0211314x\r'
            return re.findall(r, in_str)

        def parse_group(g, g_id):
            res = []
            for e in g:
                # Error parsing here for every group its different. WTF!
                if e == 'a' and g_id == 0:
                    res += [-255]

            return res

        parsed = []
        try:
            recieved  = self.__ser.read_until(self.__read_until_character).decode('ascii').replace('\n',' ')
            groups = find_groups(recieved)[0]
            assert groups, f'wrong packet has come ({recieved})!'
            clear_status = groups[0]
            water_consumption_value = groups[1]
            water_brush_states = groups[2]
            brush_currents = groups[3:5]

            parsed = [g for g in groups]
            self.state_from_mb = {
                'dust_sucker': parsed[0],
                'srakel_state': parsed[1],
                'clean_water_pump': parsed[2],
                'water_consumption_value': parsed[3:8],
                'water_consumption_state': parsed[9],
                'water_brush_water_state': parsed[],
                'dump_water_to_clear': parsed[],
                'water_level_dirt': parsed[],
                'water_level_clear': parsed[],
                'brush_lft_state': parsed[],
                'brush_rgt_state': parsed[],
                'brush_lft_current': parsed[],
                'brush_rgt_current': parsed[],
                'act_center': parsed[],
                'act_lft_brush': parsed[],
                'act_rgt_brush': parsed[],
                'charge_level': parsed[],
                'charging': parsed[],
                'lights_state': parsed[],
                'dirt_water_pump': parsed[],
                'dynamic_hard_boost': parsed[],
                'unstacking_charge_station': parsed[],
                'charge_on': parsed[],
                'fluid_type': parsed[],
                'stacking_in_pin': parsed[],
                'now_mode': parsed[], 
                'crc'               : parsed[8],
                'running'           : 1,
            }
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            if self.__do_debug: print(f"[ INFO ] {e} {parsed}", file=sys.stderr)
            

if __name__ == '__main__':
    q_cmd, q_result = Queue(), Queue()
    mainboard_proc = MainBoardProcess(q_cmd, q_result, debug=1)
    mainboard_proc.start()

    while 1:
        sleep(1)
        try:
            q_cmd.put({'action':'state'})
            print(time(), " ", q_result.get())
        except KeyboardInterrupt:
            break

    q_cmd.put({'action':'EXIT'})

    mainboard_proc.join()
