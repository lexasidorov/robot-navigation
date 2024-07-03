import sys

from flask import Flask, jsonify
from multiprocessing import Process, Queue
from queue import Empty
from test_park import VisualPark

class Parker(Process):

    def __init__(self, q_cmd, q_result):
        super().__init__()

        self.cmd = None
        self.__configure_queues(q_cmd, q_result)
        self.__init_state_dictionary()
        self.vp = VisualPark(do_debug=True)


    def __configure_queues(self, q_cmd, q_result):
        self.cmd = None
        self.q_cmd = q_cmd
        self.q_result = q_result


    def __init_state_dictionary(self):
        self.state = {
            'state': None,
        }

    def run(self):
        while 1:
            try:
                self.cmd = self.q_cmd.get(block=False)
                if self.cmd['action'] == 'park':
                    self.vp.do_park()
            except Empty as e:
                pass

    def __get_state(self):
        self.q_result.put({**self.state})


    def __update_state(self, state):
        self.state = {
            'state': state
        }


class ParkingService(Flask):
    def __init__(self, application):
        super().__init__(application)
        self.__configure_endpoints()
        self.__create_parking_service()
        self.data = {"state": None}


    def __configure_endpoints(self):
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET', ])
        self.add_url_rule('/parking/<state>', view_func=self.parking, methods=['GET', ])
        self.add_url_rule('/do_park', view_func=self.do_park, methods=['GET', ])


    def __create_parking_service(self):
        self.q_cmd, self.q_result = Queue(), Queue()
        self.parker_proc = Parker(self.q_cmd, self.q_result)
        self.parker_proc.start()

    def do_park(self):
        self.q_cmd.put({**{"action": "park"}})
        res = {'state': 'get_park_command'}
        try:
            park_state = self.q_result.get(block=False)
            res.update({'state': park_state})
        except Empty:
            pass

        return jsonify(res)

    def get_state(self):
        try:
            self.q_cmd.put({'action': 'state'})
            self.data.update(self.q_result.get(block=False))
        except Empty:
            pass
            
        return jsonify(self.data), 200


    def parking(self, state):
        if state == '1':
            self.q_cmd.put({"action": "state"})
            return jsonify({'state': '1'})
        elif state == '0':
            self.q_cmd.put({"action": "state"})
            return jsonify({'state': '0'})
        else:
            print("[ ERROR ] Unknown argument <state>",file=sys.stderr)
            return jsonify({'error': 'unknown argument <state>'}), 200


if __name__ == '__main__':
    # q_cmd, q_result = Queue(), Queue()
    # parking_proc = Parker(q_cmd, q_result)
    # parking_proc.start()

    drive_app = ParkingService(__name__)
    drive_app.run(host='0.0.0.0', port=4992, debug=False)
