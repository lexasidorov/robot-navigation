from flask import Flask, current_app, jsonify
from multiprocessing import Queue
from queue import Empty
from sensors import SensorsProcess
from sys import argv
from time import sleep, time

TIMEOUT = 10

class SensorsServiceException(Exception):
    pass


class SensorsService(Flask):

    def __init__(self, application, reset_ports=False, *args, **kwargs):
        super().__init__(application)
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET',])
        self.data = {}
        self.__create_sensors_service(do_reset_used_ports=reset_ports)
        self.__wait_for_service()


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
            raise SensorsServiceException(str(e))


    def __create_sensors_service(self, do_reset_used_ports=False):
        self.q_cmd, self.q_result = Queue(), Queue()
        self.sensors_proc = SensorsProcess(self.q_cmd, self.q_result, do_reset_used_ports=do_reset_used_ports, debug=1)
        self.sensors_proc.start()


    def get_state(self):
        self.q_cmd.put({'action':'state'})
        while 1:
            try:
                x = self.q_result.get(block=False, timeout=0.1)
                break
            except Empty:
                continue
        self.data.update(x)                
        open('/tmp/sensors_service.log', 'a').write(f'{round(time(), 4)}, {x["frw_lft_ik"]}, {x["frw_rgt_ik"]}; \n')

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
    sensors_app = SensorsService(__name__, reset_ports)
    sensors_app.run(host='0.0.0.0', port=4995, debug=False)

    sensors_app.stop()


