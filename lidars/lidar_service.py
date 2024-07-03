from flask import Flask, current_app, jsonify
from lidar_nav import LidarNav
from multiprocessing import Queue
from queue import Empty
from sys import argv
from time import sleep

TIMEOUT = 10

class LidarServiceException(Exception):
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
            assert rez['running'], f'[ ERROR ] Lidar service failed, state="{rez}"'
        except (Empty, AssertionError, KeyError) as e:
            self.stop()
            raise LidarServiceException(str(e))


    def stop(self):
        self.q_cmd.put({'action':'stop'})


    def __create_lidar_service(self, do_reset_used_ports=False):
        self.q_cmd, self.q_result = Queue(), Queue()
        self.lidar_proc = LidarNav(self.q_cmd, self.q_result, do_reset_used_ports=do_reset_used_ports, debug=1)
        self.lidar_proc.start()


    def get_environment(self):
        self.q_cmd.put({'action':'state'})
        res = {}
        resp = self.q_result.get()['distances']
        res.update({
            "env": {
                "lft": {
                    "frw": resp[ 2],
                    "mdl": resp[ 3],
                    "bck": resp[ 4],
                },
                "rgt": {
                    "frw": resp[10],
                    "mdl": resp[ 9],
                    "bck": resp[ 8],
                },
                "frw": {
                    "lft": resp[ 1],
                    "mdl": resp[ 0],
                    "rgt": resp[11],
                },
                "bck": {
                    "lft": resp[ 5],
                    "mdl": resp[ 6],
                    "rgt": resp[ 7],
                }
            }
        })

        return jsonify(res), 200


    def get_state(self):
        self.q_cmd.put({'action':'state'})
        data = {'empty':'true'}
        for i in range(100):
            try:
                data = self.q_result.get()
                break
            except Empty:
                continue
        self.data.update(data)

        return jsonify(self.data), 200


    def __del__(self):
        self.q_cmd.put({'action':'EXIT'})
        self.lidar_proc.join()


if __name__ == '__main__':
    try:
        assert argv[1] == 'reset_ports'
        reset_ports = True
    except (IndexError, AssertionError):
        reset_ports = False
    lidar_coords_app = LidarCoords(__name__, reset_ports)
    lidar_coords_app.run(host='0.0.0.0', port=4996, debug=False)

    lidar_coords_app.stop()
