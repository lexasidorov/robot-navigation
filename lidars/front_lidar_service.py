from flask import Flask, current_app, jsonify
from lidar_x2_nav import LidarNavX2
from multiprocessing import Queue
from queue import Empty
from sys import argv
from time import sleep

import numpy as np

TIMEOUT = 10

class FrontLidarServiceException(Exception):
    pass


class LidarCoords(Flask):

    def __init__(self, application, reset_ports=False, *args, **kwargs):
        super().__init__(application)
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET',])
        self.add_url_rule('/environment', view_func=self.get_environment, methods=['GET',])
        self.data = {}
        self.__create_lidar_service(do_reset_used_ports=reset_ports)
        self.__wait_for_service()


    def __wait_for_service(self):
        try:
            for x in range(TIMEOUT):
                self.q_cmd.put({'action':'state'})
                sleep(0.1)
                rez = self.q_result.get(timeout=TIMEOUT)
                if rez['running']:
                    break
            assert rez['running'], f'[ ERROR ] Front lidar service failed, state="{rez}"'
        except (Empty, AssertionError, KeyError) as e:
            self.stop()
            raise FrontLidarServiceException(str(e))


    def stop(self):
        self.q_cmd.put({'action':'stop'})


    def __create_lidar_service(self, do_reset_used_ports=False):
        self.q_cmd, self.q_result = Queue(), Queue()
        self.lidar_proc = LidarNavX2(self.q_cmd, self.q_result, do_reset_used_ports=do_reset_used_ports, debug=1)
        self.lidar_proc.start()


    def get_environment(self):
        self.q_cmd.put({'action':'state'})
        res = {}
        resp = self.q_result.get()['distances_x2']
        res.update({
            "env": {
                "lft": {
                    "frw": resp[0],
                },
                "rgt": {
                    "frw": resp[5],
                },
                "frw": {
                    "lft": resp[1],
                    "mdl": np.average(resp[2:4]),
                    "rgt": resp[4],
                },
            }
        })

        return jsonify(res), 200


    def get_state(self):
        self.q_cmd.put({'action':'state'})
        self.data.update(self.q_result.get())

        return jsonify(self.data), 200


    def __del__(self):
        self.q_cmd.put({'action':'EXIT'})
        self.lidar_proc.join()


if __name__ == '__main__':
    print(argv)
    try:
        reset_ports = argv[1] == 'reset_ports'
    except (IndexError, AssertionError):
        reset_ports = False
    lidar_coords_app = LidarCoords(__name__, reset_ports)
    lidar_coords_app.run(host='0.0.0.0', port=4993, debug=False)

    lidar_coords_app.stop()