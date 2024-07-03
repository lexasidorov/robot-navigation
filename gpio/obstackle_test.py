import sys
# import json
# import numpy as np

sys.path.insert(1, '../arduino/')

# from queue import Empty
from time import sleep, time
from multiprocessing import Queue
from pprint import pprint

from drv_process import DrvProcess
from sensors import SensorsProcess

PATH = [
    ('go', 2000),
    # ('left', 90),
    # ('go', 4000),
    # ('left', 90),
    # ('go', 800),
    # ('right', 180),
    # ('go', 1500),
    # ('left', 180),
    # ('go', 1500),
    # ('right', 180),
    # ('go', 1000),
    # ('right', 90),
    # ('go', 5000),
    # ('right', 90),
    # ('go', 1500),
]

class ObstacklePass():

    def __init__(self, debug=0):

        self.__do_debug = debug
        self.__create_queues()
        self.__create_processes()


    def __create_queues(self):
        self.__q_drv_cmd, self.__q_drv_result = Queue(), Queue()
        self.__q_sens_cmd, self.__q_sens_result = Queue(), Queue()


    def __create_processes(self):
        self.__drv_proc = DrvProcess(self.__q_drv_cmd, self.__q_drv_result, debug=self.__do_debug)
        self.__sens_proc = SensorsProcess(self.__q_sens_cmd, self.__q_sens_result, debug=self.__do_debug)
        self.__drv_proc.start()
        self.__sens_proc.start()


    def __extract_distance(self, state):
        dst = {
            'lft': 0,
            'rgt': 0, 
        }
        
        for k in dst.keys():
            ik = 400 if state[f'frw_{k}_ik'] else 10000
            #us = state[f'frw_{k}_dst']
            #us = 10000 if us == 0 or us == None else us
            dst[k] = ik

        return dst


    def __check_obstackle(self, obstacle_distance):
        return obstacle_distance['rgt'] <= 400 or obstacle_distance['lft'] <= 400


    def __get_bypass_commands(self, obstacle_distance, to_go, passed):
        rgt, lft = obstacle_distance['rgt'], obstacle_distance['lft']
        X = max(to_go - passed - 400 - 500, 0)
        if rgt < 400 and lft < 400:
            cmds = [('EXIT', None)]
        elif lft <= 400:
            cmds = [('right', 90),('go', 200), ('left', 90),('go', 1650),('left', 90),('go', 200), ('right', 90),('go', X)]
        elif rgt <= 400:
            cmds = [('left', 90),('go', 200), ('right', 90),('go', 1650),('right', 90),('go', 200),('left', 90),('go', X)]
        return cmds


    def run(self):
        for cmd, value in PATH:
            self.__q_drv_cmd.put({'action':cmd,'value':value})
            print({'action':cmd,'value':value}, end=' ... ', flush=1)
            for i in range(700):
                self.__q_drv_cmd.put({'action':'state'})
                self.__q_sens_cmd.put({'action':'state'})
                sleep(0.1)
                try:
                    drv = self.__q_drv_result.get()
                    to_go = drv['to_go']
                    passed = drv['passed']
                    if drv['drive_state'] == 'stop':
                        break
                    obstacle_distance = self.__extract_distance(self.__q_sens_result.get())
                    #print(obstacle_distance)
                    if self.__check_obstackle(obstacle_distance):
                        # self.__q_drv_cmd.put({'action':'stop'})
                        for ocmd, ovalue in self.__get_bypass_commands(obstacle_distance, to_go, passed):
                            print(ocmd, ovalue)
                            self.__q_drv_cmd.put({'action':ocmd,'value':ovalue})
                            for i in range(450):
                                self.__q_drv_cmd.put({'action':'state'})
                                sleep(0.1)
                                try:
                                    rez = self.__q_drv_result.get()
                                    # pprint(rez)
                                    state = rez['drive_state']
                                    if state == 'stop':
                                        break
                                except Exception as e:
                                    raise e
                except Exception as e:
                    raise e
            print('done')

    def __no_run(self):
        self.__q_drv_cmd.put({'action':'go','value':10000})
        for i in range(300):
            
            pprint(self.__q_sens_result.get())
            sleep(0.1)
        

    def stop(self):
        self.__q_drv_cmd.put({'action':'stop'}), sleep(0.01)
        self.__q_drv_cmd.put({'action':'EXIT'}), sleep(0.01)
        self.__drv_proc.stop(), self.__drv_proc.join()
        self.__q_sens_cmd.put({'action':'EXIT'}), sleep(0.01)
        self.__sens_proc.join()


if __name__ == '__main__':
    p = ObstacklePass(debug=1)
    p.run()
    # sleep(10)
    p.stop()
