import sys
import json
import numpy as np

from queue import Empty
from time import sleep, time
from flask import Flask, request, jsonify
from multiprocessing import Queue, Process
from drv_process import DrvProcess

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

class DriveController(Flask):

    def __init__(self, application):
        super().__init__(application)
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET',])
        self.add_url_rule('/fwd/1', view_func=self.fwd_go, methods=['GET',])
        self.add_url_rule('/fwd/0', view_func=self.stop, methods=['GET',])
        self.add_url_rule('/wash/1', view_func=self.wash_start, methods=['GET',])
        self.add_url_rule('/wash/0', view_func=self.wash_stop, methods=['GET',])
        self.add_url_rule('/lft/1', view_func=self.lft_go, methods=['GET',])
        self.add_url_rule('/lft/0', view_func=self.stop, methods=['GET',])
        self.add_url_rule('/rgt/1', view_func=self.rgt_go, methods=['GET',])
        self.add_url_rule('/rgt/0', view_func=self.stop, methods=['GET',])
        self.add_url_rule('/exit/1', view_func=self.exit, methods=['GET',])
        self.add_url_rule('/move/<coords>', view_func=self.move_to, methods=['GET',])
        self.q_cmd = Queue()
        self.q_result = Queue()
        self.drv_proc = DrvProcess(self.q_cmd, self.q_result)
        self.drv_proc.start()
        self.w_cmds = json.load(open('/home/pi/robot-visual-nav/wash_commands.json'))
        self.w_proc = Process(target=self.make_wash, args=(self.w_cmds,))
        self.force_stop_wash = False

    def make_wash(self, cmds):
        self.force_stop_wash = False
        for c in cmds:
            if self.force_stop_wash == True:
                self.w_proc.join()                
                break
            self.q_cmd.put(c)
            sleep(int(c['slp_val']))

    def wash_start(self, cmds=[]):
        self.w_proc.start()
        
        return jsonify({'command': 'wash', 'sent': 1}), 200

    def wash_stop(self, force=False):
        self.q_cmd.put({'action': 'stop'})
        state = {'result': 'empty'}
        self.force_stop_wash = True
        self.w_proc.join()
        if force:
            self.q_cmd.put({'action': 'stop'})
            self.q_cmd.put({'action': 'EXIT'})
            self.w_proc.join()            
        try:
            state = self.q_result.get(block=True, timeout=1)
        except Empty:
            print('[ WARNING ] Not getting result from drv_process.', file=sys.stderr)

        return jsonify(state), 200

    def move_to(self, x, y, angle):
        if not all([x,y,angle]):
            coords = (0,0,0)
        else:
            coords = (int(x),int(y),int(angle))
        self.q_cmd.put({'action': 'move', 'coords0': coords})

        return jsonify({'command': 'move', 'sent': 1}), 200

    def get_state(self):
        cmd = {
            'action'     : 'state',
        }
        self.q_cmd.put(cmd)
        state = {'result': 'empty'}
        now = time()
        while (time() - now) < 3.14:
            try:
                state = self.q_result.get(block=True, timeout=1)
                break
            except Empty:
                print('[ WARNING ] Not getting result from drv_process.', file=sys.stderr)
            
        return jsonify(state), 200

    def fwd_go(self, distance=5000):
        if distance == None:
            distance = 500
        cmd = {
            'action'     : 'go',
            'value'      : distance
        }
        self.q_cmd.put(cmd)
        
        return jsonify({'command': cmd, 'sent': 1}), 200

    def lft_go(self, angle=10):
        if angle == None:
            angle = 10
        cmd = {
            'action'     : 'left',
            'value'      :   angle
        }
        self.q_cmd.put(cmd)
        
        return jsonify({'command': cmd, 'sent': 1}), 200

    def rgt_go(self, angle=10):
        if angle == None:
            angle = 10
        cmd = {
            'action'     : 'right',
            'value'      :   angle
        }
        self.q_cmd.put(cmd)
        
        return jsonify({'command': cmd, 'sent': 1}), 200

    def stop(self):
        cmd = {
            'action'     : 'stop',
        }
        self.q_cmd.put(cmd)
        
        return jsonify({'command': cmd, 'sent': 1}), 200

    def exit(self):
        cmd = {
            'action'     : 'EXIT',
        }
        self.q_cmd.put(cmd)
        sleep(1)
        self.drv_proc.stop()
        self.drv_proc.join()
        
        return jsonify({'command': cmd, 'sent': 1}), 200

    # def run(self):
    #     cnt = 0
    #     print('run')
        # while 1:
        #     sleep(1)
        #     print('while')
        #     try:
        #         cnt+=1
        #         if cnt > 5:
        #             self.q_cmd.put({'action': 'EXIT'})
        #             break
        #         else:
        #             self.q_cmd.put({'action': 'go'})
        #         res = self.q_result.get(block=False)
        #         print(res)
        #     except Empty as e:
        #         print('.')
        #     print(cnt)

    def __del__(self):
        print('Call __del__()', file=sys.stderr)
        self.q_cmd.put({'action' : 'EXIT'})
        self.drv_proc.stop()
        self.drv_proc.join()
        del self.drv_proc


if __name__ == '__main__':

    drive_app = DriveController(__name__)
    drive_app.run(host='0.0.0.0', port=4998)

    drive_app.stop()
    