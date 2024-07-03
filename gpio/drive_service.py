import sys
import os
import json

import requests
# import numpy as np
import traceback as tb

from queue import Empty
from time import sleep, time
from flask import Flask, jsonify, current_app, make_response
from multiprocessing import Queue, Process, Manager
from drv_process import DrvProcess
from datetime import datetime
from sympy import Point2D, Line2D, Ray2D, pi
import numpy as np
from math import atan2, sin, cos, asin

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from settings import SERVICES

''' Available commands

1. go
2. stop
3. EXIT
4. left
5. right

'''

DEFAULT_PWM_VALUE = 18
DEFAULT_MAX_PWM = 25
DEFAULT_MIN_PWM = 10

PATH = [
    ["go", 700],
    ["right", 30],
    ["left", 30],
    ["right", 45],
    ["go", 1300],
    ["left", 45],
    ["go", 5000],
    ["right", 180],
    ["go", 6200],
    ["left", 180],
    ["go", 6200],
    ["right", 180],
    ["go", 6200],
    ["right", 90],
    ["go", 1700],
    ["left", 90],
    ["go", 2800],
    ["right", 60],
    ["go", 1000],
    ["right", 120]
]


class StopWashException(Exception):
    def __init__(self, message):
        self.message = message


class CommunicationException(IOError):
    def __init__(self, message):
        self.message = message


class DriveController(Flask):

    def __init__(self, application):
        super().__init__(application)
        self.app = application
        self.current_app = current_app
        self.__cmd_hist = []
        self.__time_to_continue = time()
        self.inject_cmd = True
        self.apw_distance = 750
        self.__obstacle_time = time()
        # self.__control_state.value = 'standby'
        self.__cur_path = None
        self.__obstacle_path = None
        # self.__avoid_cnt = None
        # self.__return_cnt = None
        self.__need_to_localize = False

        self.__configure_endpoints()
        self.__configure_queues()
        self.__init_processes()

    def __configure_endpoints(self):
        self.add_url_rule('/state/get', view_func=self.__get_state, methods=['GET', ])
        self.add_url_rule('/fwd/1/<distance>', view_func=self.__fwd_go, methods=['GET', ])
        self.add_url_rule('/fwd/0', view_func=self.__stop, methods=['GET', ])
        self.add_url_rule('/bck/1/<distance>', view_func=self.__bck_go, methods=['GET', ])
        self.add_url_rule('/bck/0', view_func=self.__stop, methods=['GET', ])
        self.add_url_rule('/lft/1/<angle>', view_func=self.__lft_go, methods=['GET', ])
        self.add_url_rule('/bck/lft/1/<angle>', view_func=self.__bck_lft_go, methods=['GET', ])
        self.add_url_rule('/bck/rgt/1/<angle>', view_func=self.__bck_rgt_go, methods=['GET', ])
        self.add_url_rule('/lft/0', view_func=self.__stop, methods=['GET', ])
        self.add_url_rule('/lft/0/<reset>', view_func=self.__stop, methods=['GET', ])  # TODO
        self.add_url_rule('/stop', view_func=self.__stop, methods=['GET', ])
        self.add_url_rule('/rgt/1/<angle>', view_func=self.__rgt_go, methods=['GET', ])
        self.add_url_rule('/goto/<x>/<y>/<angle_z>', view_func=self.__go_to, methods=['GET', ])
        self.add_url_rule('/rgt/0', view_func=self.__stop, methods=['GET', ])
        self.add_url_rule('/rgt/0/<reset>', view_func=self.__stop, methods=['GET', ])  # TODO
        self.add_url_rule('/wash/1', view_func=self.__wash_start, methods=['GET', ])
        self.add_url_rule('/wash/0', view_func=self.__wash_stop, methods=['GET', ])
        self.add_url_rule('/park/1', view_func=self.__park, methods=['GET', ])
        self.add_url_rule('/exit/1', view_func=self.__exit, methods=['GET', ])
        self.add_url_rule('/reset/angle/', view_func=self.__reset_angle, methods=['GET', ])
        self.add_url_rule('/reset/passed/', view_func=self.__reset_passed, methods=['GET', ])
        self.add_url_rule('/set/coords/<x>/<y>', view_func=self.__set_coords, methods=['GET', ])
        self.add_url_rule('/set/angle/<angle>', view_func=self.__set_angle, methods=['GET', ])
        # self.add_url_rule('/apw', view_func=self.__approach_wall, methods=['GET', ]) # TODO: delete this
        self.add_url_rule('/gld', view_func=self.__get_lidar_data, methods=['GET', ])  # TODO: delete this
        self.__wash_cmds = []
        self.__force_stop_wash = False

    def __configure_queues(self):
        self.__q_cmd = Queue()
        self.__q_result = Queue()
        self.__q_wash_control_cmd = Queue()
        self.__q_control_state = Queue(maxsize=1)
        self.__q_cur_path = Queue(maxsize=1)

    def __init_processes(self):
        self.__control_state_manager = Manager()
        self.__control_state = self.__control_state_manager.Value('control_state', None)
        self.__avoid_cnt_manager = Manager()
        self.__avoid_cnt = self.__avoid_cnt_manager.Value('avoid_cnt', 0)
        self.__drv_proc = DrvProcess(self.__q_cmd, self.__q_result, debug=0)
        self.__control_proc = Process(target=self.__do_control,
                                      args=(self.__q_cmd,
                                            self.__q_result,
                                            self.__q_control_state,
                                            self.__q_cur_path,
                                            self.__control_state,
                                            self.__avoid_cnt,
                                            True))
        self.__wash_proc = Process(target=self.__execute_path,
                                   args=(self.__q_cmd,
                                         self.__q_wash_control_cmd,
                                         self.__q_control_state,
                                         self.__q_cur_path,
                                         # self.__control_state,
                                         self.__avoid_cnt,
                                         )
                                   )
        self.__drv_proc.start()
        self.__control_proc.start()
        self.__wash_proc.start()

    def __do_control(self, __q_cmd, __q_result, __q_control_cmd, __q_cur_path, __control_state, __avoid_cnt, parallel):
        # sensors, environment, state = None, None, None
        while 1:
            sleep(0.1)
            try:
                environment = requests.get(f'http://localhost:{SERVICES["ports"]["lidars_x2"]}/environment').json()
                # TODO: add from lidars_x2!
                state = self.__get_state_parameters()
                drive_state = state["drive_state"]
                sensors = requests.get(f'http://localhost:{SERVICES["ports"]["sensors"]}/state/get').json()
                lidar_obstacle = self.__get_lidar_obstacle()
                self.__do_rules(drive_state, environment, sensors, lidar_obstacle, parallel, state)
                if __q_control_cmd.get(block=False) != {'action': 'EXIT'}:
                    print('[ WARNING ] Break!', file=sys.stderr)
                    break
            except Empty:
                pass
            except (ConnectionError, ValueError) as e:
                print(f'[ ERROR1 ] {e} MARK2', file=sys.stderr)
            except KeyError as e:
                print(tb.format_exc(), file=sys.stderr)
                print(f'[ ERROR2 ] {e} MARK2', file=sys.stderr)
            except Exception as e:
                print(tb.format_exc(), file=sys.stderr)
                print(f'[ ERROR3 ] {e} MARK2', file=sys.stderr)
            finally:
                pass

    def __do_rules(self, drive_state, environment, sensors, lidar_obstacle, parallel, state):
        # self.__update_control_state()
        is_approached_wall = self.__is_approached_wall(self.apw_distance)
        print(f"[ INFO_WALL ] 143 __control_state={self.__control_state.value}", file=sys.stderr)
        if self.__is_approaching() and is_approached_wall:
            print('[ MY_INFO ]145 DO RULES IF', file=sys.stderr)
            self.__stop(reset=False, q=self.__q_wash_control_cmd)
            # self.__put_in_q_control_state('stop')
            self.__control_state.value = 'stop'

        if drive_state == 'go' and not self.__is_approaching():
            print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST B",
                  file=sys.stderr)
            if ((sensors['frw_lft_ik'] or sensors['lft_frw_ik']) and not sensors['frw_rgt_ik']):
                print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST B",
                      file=sys.stderr)
                # print('[ MY_INFO ]173', file=sys.stderr)
                self.__obstacle_avoidance(side='right')
                sleep(0.2)
            elif ((sensors['frw_rgt_ik'] or sensors['rgt_frw_ik']) and not sensors['frw_lft_ik']):
                print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST B",
                      file=sys.stderr)
                # print('[ MY_INFO ]188', file=sys.stderr)
                self.__obstacle_avoidance(side='left')
                sleep(0.2)

            # elif sensors['frw_lft_ik'] and sensors['frw_rgt_ik']:
            #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST B",
            #           file=sys.stderr)
            #     self.__bck_go(300)
            #     self.__obstacle_avoidance(side='left')
            #     sleep(0.2)

        # elif (drive_state == 'right' or drive_state == 'left') and not self.__is_approaching():
        #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST B1",
        #           file=sys.stderr)
        #     if sensors['frw_rgt_ik'] or sensors['frw_lft_ik']:
        #         self.__obstacle_back_go(200)
        #         sleep(0.2)
        #
        # elif drive_state == 'back' and not self.__is_approaching():
        #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST B2",
        #           file=sys.stderr)
        #     if sensors['bkw_rgt_ik'] or sensors['bkw_lft_ik']:
        #         self.__stop(reset=False, q=self.__q_wash_control_cmd)
        #         sleep(0.2)

        elif drive_state == 'stop' and not self.__is_approaching():
            print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST C",
                  file=sys.stderr)
            # if sensors['frw_lft_ik'] and sensors['frw_rgt_ik']:
            #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST TEST B",
            #           file=sys.stderr)
            #     self.__bck_go(200)
            #     # self.__synchro(self.__bck_go, 200, parallel)

            # elif sensors['bkw_lft_ik'] or sensors['bkw_rgt_ik']:
            #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST TEST C",
            #           file=sys.stderr)
            #     self.__fwd_go(200)
            #     # self.__synchro(self.__fwd_go, 200, parallel)

            # elif sensors['lft_bkw_ik'] or sensors['rgt_frw_ik']:
            #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST TEST C",
            #           file=sys.stderr)
            #     self.__lft_go(10)
            #     # self.__synchro(self.__lft_go, 10, parallel)

            # elif sensors['rgt_bkw_ik'] or sensors['lft_frw_ik']:
            #     print(f"[ INFO ] drive_state={drive_state}, environment={environment}, sensors={sensors} TEST TEST C",
            #           file=sys.stderr)
            #     self.__rgt_go(10)
            #     # self.__synchro(self.__rgt_go, 10, parallel)

    # def __update_control_state(self):
    #     prev_state = self.__control_state.value 
    #     try:
    #         control_state = self.__q_control_state.get(timeout=0.1)
    #         self.__control_state.value = control_state
    #     except Empty:
    #         pass
    #     if prev_state != self.__control_state.value :
    #         print(f'[ MY_INFO ] 262 control_state is changed to: {self.__control_state.value}', file=sys.stderr)

    @staticmethod
    def __get_lidar_data():
        data = requests.get(f'http://localhost:{SERVICES["ports"]["lidars"]}/state/get').json()

        return data

    @staticmethod
    def __get_lidar_x2_env():
        data = requests.get(f'http://localhost:{SERVICES["ports"]["lidars_x2"]}/environment').json()

        return data

    def __get_lidar_obstacle(self):
        # {"env": {"frw": {"lft": 12000, "mdl": 7166.75, "rgt": 1766.1666666666665}, "lft": {"frw": 12000},
        #          "rgt": {"frw": 1393.75}}}

        # env = self.__get_lidar_x2_env()["env"]
        # if env["frw"]["mdl"] < 400 or env["frw"]["lft"] < 400 or env["lft"]["frw"] < 300:
        #     print('[ MY_INFO ] 282 Lidar obstacle left', file=sys.stderr)
        #     return "left"
        # elif env["frw"]["rgt"] < 400 or env["rgt"]["frw"] < 300:
        #     print('[ MY_INFO ] 285 Lidar obstacle right', file=sys.stderr)
        #     return "right"
        # else:
        return None

    def __get_lidars_distance(self):
        l1 = int(np.mean(self.__get_lidar_data()['distances'][0]))
        l2 = int(self.__get_lidar_x2_env()['env']['frw']['mdl'])
        if l2 < l1:
            print(f'[ INFO_WALL ]229 l1 > l2', file=sys.stderr)
            return l2
        else:
            print(f'[ INFO_WALL ]232 l1 < l2', file=sys.stderr)
            return l1

    def __is_approaching(self):
        return self.__control_state.value == 'approaching'

    def __is_turning(self):
        return

    def __is_routing(self):
        return self.__control_state.value == 'routing' and self.__avoid_cnt.value == 0

    def __is_recording_cur_path(self):
        return self.__control_state.value == 'recording_cur_path'

    def __is_approached_wall(self, approach_distance):
        distance = self.__get_lidars_distance()
        sensors = requests.get(f'http://localhost:{SERVICES["ports"]["sensors"]}/state/get').json()
        if distance < approach_distance:
            print(f'[ INFO_WALL ]242 {distance} Wall is approached!{self.__control_state.value}', file=sys.stderr)

            return True

        elif sensors['frw_lft_ik'] or sensors['frw_rgt_ik']:
            print(f'[ INFO_WALL ]247 Wall is approached SENSORS!{self.__control_state.value}', file=sys.stderr)
            # self.__bck_go(300)
            return True

        else:
            print(f'[ INFO_WALL ]252 {distance}', file=sys.stderr)

            return False

    def __obstacle_back_go(self, value=200):
        now = time()
        print(f"[ MY_INFO ] avoiding obstacle while turning. now = {now} | obstime = {self.__obstacle_time}",
              file=sys.stderr)
        if now - self.__obstacle_time > 1.5:
            self.__set_drv_counters(1 + self.__avoid_cnt)
            # self.__put_in_q_control_state('avoiding')
            self.__control_state.value = 'avoiding'
            self.__bck_go(value, q=self.__q_wash_control_cmd)
            self.__obstacle_time = time()

    def __obstacle_avoidance(self, side='left'):
        assert side == 'left' or side == 'right'
        contra = 'left' if side == 'right' else 'right'
        # self.__rec_cur_path(state['coords'])
        now = time()
        print(f"[ MY_INFO ] avoiding obstacle from left. now = {now} | obstime = {self.__obstacle_time}",
              file=sys.stderr)
        if now - self.__obstacle_time > 5:
            self.__obstacle_time = time()
            cmds = [
                # {'action': 'rec_cur_path', 'value': None},
                {'action': side, 'value': 40},
                {'action': 'go', 'value': 800},
                {'action': contra, 'value': 40},
                {'action': 'go', 'value': 800},
                # {'action': contra, 'value': 10},
                # {'action': 'go', 'value': 200},
                # {'action': contra, 'value': 10},
                # {'action': 'go', 'value': 200},

                # {'action': 'right', 'value': 80},
                # {'action': 'go', 'value': 1200},
                # {'action': 'left', 'value': 40},
                # {'action': 'ap_wall', 'value': 1000},
            ]
            if self.__is_routing():
                self.__control_state.value = 'recording_cur_path'
            else:
                self.__control_state.value = 'avoiding'
            self.__set_drv_counters(len(cmds))
            self.__put_in_q_wash_control(cmds)
            print('[ MY_INFO ]372', file=sys.stderr)

    def __get_state(self):
        state = {'result': 'empty'}
        cmd = {'action': 'state', 'source': 'def __get_state(self)'}
        self.__q_cmd.put({**cmd})
        for i in range(10):
            try:
                state = self.__q_result.get(timeout=0.1)
                state.update({'result': 'ready'})
                break
            except Empty as e:
                # print('[ MY_INFO ] 373 Except Empty in __get_state_parameters',file=sys.stderr)
                continue
        if state['result'] == 'empty':
            raise CommunicationException('[ ERROR ] Number of retries exceeded')

        return jsonify(state), 200

    def __get_state_parameters(self):
        def __get_drv_counters(state_data):
            try:
                prev_cnt = self.__avoid_cnt.value
                self.__avoid_cnt.value = state_data['avoid_cmd_count']
                # self.__return_cnt = state_data['return_cmd_count']
                if prev_cnt != self.__avoid_cnt.value:
                    print(f'[ MY_INFO ] 369 avoid_cnt now = {self.__avoid_cnt.value}', file=sys.stderr)
            except Exception as e:
                print(f'[ ERROR ] Exception: {e} in __get_drv_counters', file=sys.stderr)

        cmd = {'action': 'state', 'source': 'def __get_state_parameters(self)'}
        self.__q_cmd.put({**cmd})
        for i in range(10):
            try:
                x = self.__q_result.get(timeout=0.1)
                __get_drv_counters(x)
                return x
            except Empty as e:
                continue
        raise CommunicationException('[ ERROR ] Number of retries exceeded')

    def __set_drv_counters(self, avoid_cnt, return_cnt=0):
        avoid_cmd = {'action': 'set_avoid_cmd_count', 'value': avoid_cnt}
        # return_cmd = {'action': 'set_return_cmd_count', 'value': return_cnt}
        self.__q_cmd.put({**avoid_cmd})
        # self.__q_cmd.put({**return_cmd})
        # self.__avoid_cnt.value = avoid_counter

    def __park(self):
        cmd = {
            'action': 'park',
            'value' : None
        }
        self.__put_in_q_wash_control([cmd])

        return jsonify({'command': 'park'}), 200


        # def __success():
        #     try:
        #         sensors = requests.get(f'http://localhost:{SERVICES["ports"]["sensors"]}/state/get').json()
        #         bkw_lft_ik = sensors["bkw_lft_ik"]
        #         bkw_rgt_ik = sensors["bkw_rgt_ik"]
        #         return bkw_lft_ik and bkw_rgt_ik
        #     except requests.exceptions.ConnectionError as e:
        #         data = {'error': f"Can not get state from `sensors`. ConnectionError {e}."}
        #         return None
        #
        # def __timeout():
        #     return (time() - a_time) > 10
        #
        # a_time = time()
        # self.__q_cmd.put({'action': 'back', 'value': 1000})
        # while not __success() and not __timeout():
        #     sleep(0.1)
        # self.__q_cmd.put({'action': 'stop'})
        #
        # return jsonify({'command': 'park', 'success': __success(), 'timeout': __timeout()}), 200

    # def __synchro(self, action, val, parallel=False):
    #     print('[ MY_INFO ]386 START of __synchro', file=sys.stderr)
    #     res = action(val, parallel)
    #     print(f'[ MY_INFO ]388, res= {res}', file=sys.stderr)
    #     drive_state = self.__get_state_parameters()["drive_state"]
    #     print(f'[ MY_INFO ]390 drive_state = {drive_state}', file=sys.stderr)
    #     while drive_state != 'stop':
    #         drive_state = self.__get_state_parameters()["drive_state"]
    #         print(f'[ MY_INFO ]393 drive_state in while = {drive_state}', file=sys.stderr)
    #
    #     print(f'[ MY_INFO ]399 END of __synchro {action}({val})', file=sys.stderr)
    #     return res

    def __reset_angle(self, parallel=False):
        cmd = {
            'action': 'reset_angle',
        }
        self.__q_cmd.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})
        return jsonify({'command': cmd, 'sent': 1}), 200

    def __reset_passed(self, parallel=False):
        cmd = {
            'action': 'reset_passed',
        }
        self.__q_cmd.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})
        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __set_coords(self, x, y, parallel=False):
        try:
            cmd = {
                'action': 'set_coords',
                'value': (float(x), float(y))
            }
            self.__q_cmd.put({**cmd})
            if parallel:
                return json.dumps({'command': cmd, 'sent': 1})
            return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)
        except ValueError as e:
            return make_response(json.dumps({'error': e}), 501)

    def __set_angle(self, angle, parallel=False):
        try:
            cmd = {
                'action': 'set_angle',
                'value': float(angle)
            }
            self.__q_cmd.put({**cmd})
            if parallel:
                return json.dumps({'command': cmd, 'sent': 1})
            return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)
        except ValueError as e:
            return make_response(json.dumps({'error': e}), 501)

    def __fwd_go(self, distance, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if distance is None:
            distance = 500
        cmd = {
            'action': 'go',
            'value': int(distance)
        }
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __bck_go(self, distance, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if distance is None:
            distance = 500
        cmd = {
            'action': 'back',
            'value': int(distance)
        }
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __bck_lft_go(self, angle, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if angle is None:
            angle = 10
        cmd = {
            'action': 'back_left',
            'value': int(angle)
        }
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __bck_rgt_go(self, angle, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if angle is None:
            angle = 10
        cmd = {
            'action': 'back_right',
            'value': int(angle)
        }
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __lft_go(self, angle, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if angle is None:
            angle = 10
        cmd = {
            'action': 'left',
            'value': int(angle)
        }
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __rgt_go(self, angle, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if angle is None:
            angle = 10
        cmd = {
            'action': 'right',
            'value': int(angle)
        }
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __go_to(self, x, y, angle_z, parallel=False):
        cmd = {
            'action': 'goto',
            'value': [int(x), int(y), int(angle_z)]
        }
        self.__put_in_q_wash_control([cmd])
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __stop(self, reset=False, parallel=False, q=None):
        if not q:
            q = self.__q_cmd
        if reset is None:
            reset = 0
        cmd = {'action': 'stop', }
        if reset:
            self.__q_cmd.put({'action': 'reset_angle'})
            sleep(.3)
        q.put({**cmd})
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __exit(self, parallel=False):
        cmd = {
            'action': 'EXIT',
        }
        self.__q_cmd.put({**cmd})
        sleep(1)
        self.__drv_proc.stop()
        self.__drv_proc.join()
        self.__control_proc.join()
        self.__wash_proc.join()
        if parallel:
            return json.dumps({'command': cmd, 'sent': 1})

        return make_response(json.dumps({'command': cmd, 'sent': 1}), 200)

    def __wash_start(self):
        commands = {}
        try:
            commands = json.load(open('/home/pi/robot-visual-nav/commands.json'))
        except FileNotFoundError:
            commands = {'path': [
                ("go", 200)
            ]}
        except Exception as e:
            print(f'[ INFO ] 355 Exceeeeeeept {e}', file=sys.stderr)
        try:
            cmds = []
            for i in commands['path']:
                cmd = {'action': i[0], 'value': i[1]}
                cmds.append(cmd)
            self.__put_in_q_wash_control(cmds)
        except KeyError as e:
            cmd = {'action': 'stop', 'value': None}
            self.__q_wash_control_cmd.put({**cmd})
            print(f"[ ERROR ] 362 KeyError {e} in __wash_start", file=sys.stderr)
        except Exception as e:
            print(f'[ INFO ] 364 Exceeeeeeept {e}', file=sys.stderr)

        return make_response(json.dumps({'command': 'wash', 'sent': 1, 'time': str(datetime.now())}), 200)

    def __wash_stop(self, force=False):
        state = {'result': 'empty'}
        self.__force_stop_wash = True
        s = {'action': 'stop', 'value': 'naxer'}
        self.__q_wash_control_cmd.put({**s})

        return jsonify(state), 200

    # def __get_return_path(self, begin_coords, begin_angle):
    #     avoid_cnt = self.__get_state()['avoid_cmd_count']
    #     if avoid_cnt > 0:
    #         coords = self.__get_coords_after_obstackle(begin_coords, begin_angle)

    # def __get_next_Ray(self, command, state):
    #     assert command['action'] == 'go'
    #     angle_z = float(state['angle_z'])*(pi/180) # radians
    #     to_coords = [
    #         float(state['coords'][0]) + command['value'] * sin(angle_z),
    #         float(state['coords'][1]) + command['value'] * cos(angle_z)
    #         ]
    #     r = Ray2D(Point2D(to_coords[0],to_coords[1]), angle=angle_z)
    #
    #     return r

    # def __get_current_Ray(self, state):
    #     angle_z = float(state['angle_z'])*(pi/180) # radians
    #     r = Ray2D(Point2D(state['coords'][0], state['coords'][1]), angle=angle_z)
    #
    #     return r

    def __rec_cur_path(self, begin_coords):
        if self.__is_recording_cur_path():
            self.__q_cmd.put({**{'action': 'stop',}})
            print(f'[ MY_INFO ] 655 recording cur_path with {begin_coords}', file=sys.stderr)
            p = Point2D(begin_coords[1], begin_coords[0])  # оси наоборот, так надо
            angle = begin_coords[2] * (pi / 180)
            r = Ray2D(p, angle=float(angle))
            l = Line2D(r)
            cur_path = {'x0': l.points[0].y, 'y0': l.points[0].x,
                        'x1': l.points[1].y, 'y1': l.points[1].x}  # оси наоборот, так надо
            self.__q_cur_path.put({**cur_path})
            self.__control_state.value = 'avoiding'

    def __get_next_direction(self, command, coords):
        # if command['action'] == 'go' and self.__is_routing():
        assert command['action'] == 'go' and self.__is_routing()
        angle_z = coords[2] * (pi / 180)
        next_coords = [
            float(coords[0] + command['value'] * sin(angle_z)),
            float(coords[1] + command['value'] * cos(angle_z)),
            coords[2]
        ]
        print(f'[ MY_INFO ] 688 next direction: {next_coords} | control_state = {self.__control_state.value}',
              file=sys.stderr)

        return next_coords

    def __get_path_to_direction(self, coords, next_coords):
        try:
            # d, turn_0, turn_1 = self.__get_path_to_direction_old(coords, next_coords)
            d, turn_0, turn_1 = self.__get_path_to_direction_new(coords, next_coords)
        except Exception as e:
            print(f'[ MY_INFO ] 698 {e}',file=sys.stderr)
            d, turn_0, turn_1 = self.__get_path_to_direction_old(coords, next_coords)

        return d, turn_0, turn_1

    @staticmethod
    def __get_path_to_direction_old(coords, next_coords):
        x = next_coords[0] - coords[0]
        y = next_coords[1] - coords[1]
        d = float((x ** 2 + y ** 2) ** 0.5)  # distance
        d_angle = float(atan2(x, y) * (180 / pi))
        turn_0 = d_angle - coords[2]  # first turn
        turn_1 = next_coords[2] - d_angle  # second turn
        turn_0 += 360 if turn_0 < -180 else -360 if turn_0 > 180 else 0
        turn_1 += 360 if turn_1 < -180 else -360 if turn_1 > 180 else 0
        print(f'[ MY_INFO ] 699 OLD path to direction: {d, turn_0, turn_1}', file=sys.stderr)

        return d - 500, turn_0, turn_1

    @staticmethod
    def __get_path_to_direction_new(coords: list, next_coords: list):
        # p_0, p_1, p_2, p_3 - are the points of a triple move, p0 is start (coords), p3 is end(next_coords)
        # Global variables:
        r = 500  # turn radius
        D, T_0, T_3 = 0, 0, 0  # distance, first turn, second turn

        def round_a(angle):
            angle += 360 if angle < -180 else -360 if angle > 180 else 0
            return angle

        def reverse_a(angle):
            angle += 360 if angle < 0 else -360
            return angle

        def length(p0: list, p1: list):
            return float(((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5)

        def angle(p0: list, p1: list):
            return deg(atan2((p1[0] - p0[0]), (p1[1] - p0[1])))

        def rad(degrees):
            return float(degrees * (pi / 180))

        def deg(radians):
            return float(radians * (180 / pi))

        def get_rot_center(coords, turn):
            rad_z = rad(coords[2])
            if turn > 0:  # right turn
                x = float(coords[0] + r * cos(rad_z))
                y = float(coords[1] - r * sin(rad_z))
            else:  # left turn
                x = float(coords[0] - r * cos(rad_z))
                y = float(coords[1] + r * sin(rad_z))

            return [x, y]

        def is_straight(t_0, t_1):
            return round(t_0, 2) == 0 and round(t_1, 2) == 0

        def is_inner_tangent(c0):
            # this angle is for tangent from finishing point to first turn circle
            a_cp3 = angle(c0, next_coords)
            B = deg(asin(r / length(c0, next_coords))) if t_00 > 0 \
                else -deg(asin(r / length(c0, next_coords)))
            angle_kp3 = B + a_cp3
            check = (round_a(next_coords[2] - angle_kp3) * t_00)
            return check < 0

        # converting income angles to proper values ( -180 to + 180)
        coords = [coords[0], coords[1], round_a(coords[2])]
        next_coords = [next_coords[0], next_coords[1], round_a(next_coords[2])]

        # a_03 = angle(coords, tail(next_coords))       # angle between starting and finishing points
        a_03 = angle(coords, next_coords)  # angle between starting and finishing points
        t_00 = round_a(a_03 - coords[2])  # angle between starting direction and segment(p_0,p_3)
        t_03 = round_a(next_coords[2] - a_03)  # angle between segment(p_0,p_3) and finishing direction

        # needed to find side of first turn:
        if t_00 == 0:
            t_00 = 0.0001 if t_03 < 0 else - 0.0001
        elif t_00 == 180:
            t_00 = -180 if t_03 < 0 else 180
        elif t_00 == -180:
            t_00 = -180 if t_03 < 0 else 180

        if t_03 == 0:
            t_03 = 0.0001 if t_00 < 0 else - 0.0001

        print(coords, next_coords)
        if is_straight(t_00, t_03):
            print('[ INFO ] Trajectory is straight line', file=sys.stderr)
            D = length(coords, next_coords)

            return D, T_0, T_3  # no need to calculate trajectory

        c_0 = get_rot_center(coords, t_00)  # center of first turn

        if is_inner_tangent(c_0):
            print(f'[ INFO ] Trajectory on inner tangent', file=sys.stderr)

            # center of second turn
            c_2 = get_rot_center(next_coords, 90) if t_00 < 0 else get_rot_center(next_coords, -90)
            # c_2 = get_rot_center(next_coords, t_03)
            a_cc = angle(c_0, c_2)
            s_centers = length(c_0, c_2)

            # line p_1-p_2 is a tangent to 2 equal circles with centers in c_0 and c_2
            # B is angle between line c_0-c_2 (s_centers) and radius from c_0 to p_0 or c_2 to p_2
            # angle between tangent and Y

            B_rad = -asin(2 * r / s_centers) if t_00 < 0 else asin(2 * r / s_centers) if t_00 > 0 else 0
            A_t = rad(a_cc) + B_rad  # angle of tangent line

            # tangent points on turning circles
            p_1 = get_rot_center([c_0[0], c_0[1], deg(A_t)], -t_00)
            p_2 = get_rot_center([c_2[0], c_2[1], deg(A_t)], -90) if t_00 < 0 else get_rot_center(
                [c_2[0], c_2[1], deg(A_t)], 90)

            # a_12 = angle(p_1, p_2)
            # print(f'a_12 = {a_12}')
            # D = length(p_1, p_2)
            # T_0 = a_12 - coords[2]  # first turn
            # T_3 = next_coords[2] - a_12  # second turn

            D = length(p_1, p_2)
            T_0 = deg(A_t) - coords[2]  # first turn
            T_3 = next_coords[2] - deg(A_t)  # second turn
            print(f'[ INFO ] Trajectory params: p_1 = {p_1}, p_2 = {p_2}, D = {D}, A_t(deg) = {deg(A_t)}', file=sys.stderr)

        else:
            print(f'[ INFO ] Trajectory on outer tangent', file=sys.stderr)
            # this means we have 2 turns to one side
            c_0 = get_rot_center(coords, t_00)
            # center of second turn
            c_2 = get_rot_center(next_coords, -t_03) if t_00 * t_03 < 0 else get_rot_center(next_coords, t_03)
            a_cc = angle(c_0, c_2)

            s_centers = length(c_0, c_2)
            D = s_centers
            T_0 = a_cc - coords[2]
            T_3 = (next_coords[2] - a_cc)

            if t_00 < 0:  # if turns are left:
                T_0 = reverse_a(T_0) if T_0 > 0 else T_0
                T_3 = reverse_a(T_3) if T_3 > 0 else T_3

            else:
                T_0 = reverse_a(T_0) if T_0 < 0 else T_0
                T_3 = reverse_a(T_3) if T_3 < 0 else T_3

            # if T_0 * T_3 < 0:
            #     print("need to reverse")
            #     T_3 = reverse_a(T_3)

        return round(D, 4), T_0, T_3

    
    # def __get_path_to_direction_new(coords, next_coords):
    #     r = 350 # radius of robot turn
    #     def rad(angle):
    #         return angle * (pi/180)
    #
    #     def turn(t):
    #         t += 360 if t < -180 else -360 if t > 180 else 0
    #         return t
    #
    #     def rot_center(coords, turn):
    #         # turn is negative on left, positive on right
    #         x_r = coords[0] + r * sin(rad(coords[2])) if turn > 0 else coords[0] - r * sin(rad(coords[2]))
    #         y_r = coords[1] - r * cos(rad(coords[2])) if turn > 0 else coords[1] + r * cos(rad(coords[2]))
    #
    #         return x_r, y_r
    #
    #     x0, y0, a0 = coords                         # starting point (p0)
    #     x7, y7, a7 = next_coords                    # target point (p3)
    #     a = float(atan2((x7 - x0), (y7 - y0)) * (180 / pi)) # angle of line p0-p3, will be same in p1
    #
    #     t0 = turn(a - a0)  # turn at starting point
    #     t6 = turn(a7 - a)  # turn at penultimate point
    #
    #     # coordinates of center of firs turn
    #     x_r0, y_r0 = rot_center(coords, t0)
    #
    #     # coordinates of center of last turn
    #     x_r2, y_r2 = rot_center(next_coords, t6)
    #
    #     x1 = x_r0 + r * cos(rad(t0))
    #     y1 = y0 + r * sin(rad(t0))
    #     x2 = x_r2 + r * cos(-rad(t6))
    #     y2 = y_r2 + r * sin(-rad(t6))
    #
    #     # distance between points 1 & 2
    #     d = float(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5)
    #     print(f'[ MY_INFO ] 742 NEW path to direction: {d, t0, t6}', file=sys.stderr)
    #
    #     return d, t0, t6

    def __get_turn_cmd(self, angle):
        if angle < 0:
            cmd = {'action': 'left', 'value': -angle}
        else:
            cmd = {'action': 'right', 'value': angle}

        return cmd

    def __get_go_to_point_cmds(self, d, a1, a2):
        cmds = []
        if d > 300:
            cmds += [self.__get_turn_cmd(a1)]
            cmds += [{'action': 'go', 'value': d}]
            cmds += [self.__get_turn_cmd(a2)]
        else:
            print('[ MY_INFO ] 769 too small value to go',file=sys.stderr)
            cmds += [{'action': 'stop', 'value': None}]

        return cmds

    def __put_in_q_wash_control(self, cmds):
        for cmd in cmds[::-1]:
            self.__q_wash_control_cmd.put({**cmd})

    # def __put_in_q_control_state(self,control_state):
    #     self.__q_control_state.put(control_state)

    @staticmethod
    def __localize():
        res = requests.get(f'http://localhost:{SERVICES["ports"]["visual_nav"]}/v_coords/calc')

        return res.json()

    def __get_back_coords(self, coords):
        # returns coords to go back on route
        try:
            p0 = Point2D(coords[0], coords[1])
            cur_p_c = self.__q_cur_path.get(timeout=0.1)
            cur_path = Line2D(
                Point2D(cur_p_c['x0'], cur_p_c['y0']),
                Point2D(cur_p_c['x1'], cur_p_c['y1'])
            )
            p_line = cur_path.perpendicular_line(p0)
            p_x = cur_path.intersection(p_line)[0]
            # coords_x = [float(p_x.x), float(p_x.y)]
            coords_x = [float(p_x.x), float(p_x.y)]  # axes in sympi & robot are vice versa

            print(f'[ MY_INFO ] 755 back coords: {coords_x}')
            return coords_x
        except Empty as e:
            print(f'[ MY_INFO ] 775 Empty {e}', file=sys.stderr)
            self.__rec_cur_path(coords)
        except KeyError as e:
            print(f'[ MY_INFO ] Key Error: no keys found - {tb.format_exc()}', file=sys.stderr)
        except Exception as e:
            print(f'[ MY_INFO ] Exception {tb.format_exc()}', file=sys.stderr)

    @staticmethod
    def __get_cor_coords(state):
        x = state['coords'][0]
        y = state['coords'][1]
        angle_z = state['coords'][2]
        while True:
            if angle_z > 360:
                angle_z -= 360
            elif angle_z < -360:
                angle_z += 360
            else:
                break

        return x, y, angle_z

    # def __update_avoid_state(self):
    #     prev_state = self.__avoid_state
    #     try:
    #         self.__avoid_state = self.__q_avoid_state.get(timeout=0.1)
    #     except Empty:
    #         pass
    #     if self.__avoid_state != prev_state:
    #         print(f'[ MY_INFO ] avoid_state changed to {self.__avoid_state}')

    def __execute_path(self, q_cmd, q_wash_control_cmd, __q_cur_path, __control_state, __avoid_cnt):
        # local global variables
        self.__avoid_cnt.value = 0
        cmds = []
        stop = {'action': 'stop', }
        command = {'action': 'stop', }
        # cur_coords, next_coords, back_coords = [0, 0, 0], [0, 0, 0], [0, 0, 0]
        cur_coords, next_coords, back_coords = None, None, None

        def is_next_command(s):
            # sleep(0.2)
            return s["drive_state"] == 'stop'
            # if s["drive_state"] == 'stop':
            #     try:
            #         self.__avoid_cnt.value -= 1 if self.__avoid_cnt.value != 0 else None
            #     except:
            #         self.__avoid_cnt.value = 0
            #     return True

        def is_finished_avoidance():
            return self.__avoid_cnt.value == 0 and self.__control_state.value == 'avoiding'

        def is_finished_returning():
            return self.__avoid_cnt.value == 0 and self.__control_state.value == 'returning'

        def set_routing():
            print(f'[ MY_INFO ] 783 avoid_cnt = {self.__avoid_cnt.value}', file=sys.stderr)
            if self.__avoid_cnt.value == 0 and self.__control_state.value not in [
                # 'avoiding',
                # 'returning',
                'approaching',
            ]:
                self.__control_state.value = 'routing'
                print(f'[ MY_INFO ] 785 __control_state is now {self.__control_state.value}', file=sys.stderr)
            else:
                print(f'[ MY_INFO ] 787 __control_state is still {self.__control_state.value}', file=sys.stderr)

        # def wait_and_calculate_path(state):
        #     if self.__is_recording_cur_path():
        #         print('[ MY_INFO ] 783 waiting for rec cur path',file=sys.stderr)
        #         self.__rec_cur_path(state['coords'])

        def process_command(c):
            if c['action'] == 'ap_wall':
                self.__control_state.value = 'approaching'
                print(f'[ MY_INFO ] 822 command = {c}', file=sys.stderr)
                lidars_distance = self.__get_lidars_distance() - 100
                c = {'action': 'go', 'value': lidars_distance}

            elif c['action'] == 'localize_visual':
                self.__need_to_localize = True

            elif c['action'] == 'goto':
                go_to_point(command['value'], is_count=False)
                c = {'action': 'stop',}

            elif c['action'] == 'park':
                res = requests.get(f'http://localhost:{SERVICES["ports"]["parking"]}/do_park')
                print(f'[ MY_INFO ] 828 PARKING STARTED {res}',file=sys.stderr)
                c = {'action': 'stop', }

            return c

        # def go_to_point_command(command):
        #     if command['action'] == 'goto':
        #         go_to_point(command['value'], count=False)
        #

        def go_to_point(point_coords, is_count=True):
            d, a1, a2 = self.__get_path_to_direction(cur_coords, point_coords)
            cmds = self.__get_go_to_point_cmds(d, a1, a2)
            self.__set_drv_counters(len(cmds)) if is_count else self.__set_drv_counters(0)
            self.__put_in_q_wash_control(cmds)
            print('[ MY_INFO ] 817 going to point', file=sys.stderr)

        # def localize(c):
        #     if c['action'] == 'localize_visual':
        #         self.__need_to_localize = True

        while 1:
            # while there are any commands in q_wash_control_cmd queue:
            # q_wash_control_cmd can be filled via wash_start, manual control from user or in case of obstacle avoidance
            try:
                cmd = q_wash_control_cmd.get(timeout=0.1)
                print(f'[ MY_INFO ] 838 cmd from q_wash_control: {cmd}', file=sys.stderr)
                q_cmd.put({**stop})
                cmds.insert(0, cmd)
            # when nothing is in queue doing commands synchronously:
            except Empty:
                state = self.__get_state_parameters()
                cur_coords = self.__get_cor_coords(state)

                # checks if robot is stopped
                if is_next_command(state):

                    # checks if robot has finished any unplanned commands
                    if is_finished_returning():
                        print(f'[ MY_INFO ] 1096', file=sys.stderr)
                        if not next_coords:
                            print(f'[ MY_INFO ] 1098 No next point. Next coords = {back_coords}', file=sys.stderr)
                        else:
                            go_to_point(next_coords)
                        self.__control_state.value = 'routing'

                    elif is_finished_avoidance():
                        back_coords = self.__get_back_coords(cur_coords)
                        if not back_coords:
                            print(f'[ MY_INFO ] 1102 No returning. Back coords = {back_coords}', file=sys.stderr)
                            self.__avoid_cnt.value = 0
                        else:
                            back_coords.append(next_coords[2])
                            print(f'[ MY_INFO ] 1106 back coords: {back_coords}', file=sys.stderr)
                            go_to_point(back_coords)
                        self.__control_state.value = 'returning'

                    # else finished planned commands or didn't finish unplanned commands
                    else:
                        try:
                            command = cmds.pop(0)
                            # go_to_point_command(command)
                            # localize(command)
                            command = process_command(command)
                            self.__rec_cur_path(self.__get_cor_coords(state))
                            print(f'[ MY_INFO ] 851 **** EXECUTING **** {command}', file=sys.stderr)
                            print(f'[ MY_INFO ] 852 {self.__control_state.value} | {self.__avoid_cnt.value}', file=sys.stderr)
                            set_routing()
                            # sleep(0.2)
                            q_cmd.put({**command})
                            next_coords = self.__get_next_direction(command, cur_coords)
                            print(f'[ MY_INFO ] 855 next_coords: {next_coords}\n', file=sys.stderr)
                        except AssertionError as e:
                            print(f'[ MY_INFO ] 876 {e}', file=sys.stderr)
                        except IndexError as e:
                            # print(f'[ MY_INFO ] 877 ***** STANDBY *****', file=sys.stderr)
                            self.__control_state.value = 'standby'
                        finally:
                            if self.__need_to_localize:
                                # new_coords = self.__localize()
                                new_coords = [100, 100]
                                sleep(0.2)
                                print(f'[ MY_INFO ] Localization result: {new_coords}', file=sys.stderr)
                                self.__need_to_localize = False

    def __del__(self):
        print('Call __del__()', file=sys.stderr)
        self.__q_cmd.put({'action': 'EXIT'})
        self.__drv_proc.stop()
        self.__drv_proc.join()
        self.__control_proc.join()
        self.__wash_proc.join()


if __name__ == '__main__':
    drive_app = DriveController(__name__)
    drive_app.run(host='0.0.0.0', port=4998, debug=False)
