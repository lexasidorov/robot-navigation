from flask import Flask, current_app, jsonify
from mainboard import MainBoardProcess
from multiprocessing import Queue
from queue import Empty
from sys import argv
from time import sleep

TIMEOUT = 10

class MainboardServiceException(Exception):
    pass


class MainboardService(Flask):

    def __init__(self, application, reset_ports=False, *args, **kwargs):
        super().__init__(application)
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET',])
        self.data = {}
        # self.__create_sensors_service(do_reset_used_ports=reset_ports)
        # self.__wait_for_service()


    def __wait_for_service(self):
        try:
            for x in range(TIMEOUT):
                self.q_cmd.put({'action':'state'})
                sleep(0.1)
                rez = self.q_result.get(timeout=1)
                if rez['running']:
                    break
            assert rez['running'], f'[ ERROR ] Sensors service failed, state="{rez}"'
        except (Empty, AssertionError, KeyError) as e:
            self.stop()
            raise MainboardServiceException(str(e))


    def __create_sensors_service(self):
        self.q_cmd, self.q_result = Queue(), Queue()
        self.sensors_proc = MainBoardProcess(self.q_cmd, self.q_result, debug=1)
        self.sensors_proc.start()


    def get_state(self):
        self.q_cmd.put({'action':'state'})
        self.data.update(self.q_result.get())

        return jsonify(self.data), 200


    def stop(self):
        self.q_cmd.put({'action':'stop'})


    def __del__(self):
        self.q_cmd.put({'action':'EXIT'})
        self.sensors_proc.join()


if __name__ == '__main__':
    try:
        assert argv[1] == 'reset_ports'
        reset_ports = True
    except (IndexError, AssertionError):
        reset_ports = False
    mainboard_app = MainboardService(__name__, reset_ports)
    mainboard_app.run(host='0.0.0.0', port=4991, debug=False)

    mainboard_app.stop()


