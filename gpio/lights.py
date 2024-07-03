import sys

from flask import Flask, jsonify
from multiprocessing import Process, Queue

class LightsProcess(Process):
    def __init__(self, q_state, q_result, gpio, pins):
        self.pins = pins
        try:
            assert gpio
            self.gpio = gpio
            self.gpio.setup(list(self.pins.values()), gpio.OUT, initial=gpio.HIGH)
            self.usegpio = 1
        except AssertionError:
            self.usegpio = 0
        self.q_state = q_state
        self.q_result = q_result
        super().__init__(target = self.run)
     
    def run(self):
        while 1:
            state = self.q_state.get()
            if state == 'EXIT':
                break
            else:
                self.set(state)
                result = state
                self.q_result.put(result)
                
    def set(self, state):
        if self.usegpio:
            for pin, state in state.items():
                print(pin, state)
                self.gpio.output(self.pins[pin], state)
        else:
            print(state, file=sys.stderr)

class Lights(Flask):
    def __init__(self, application, gpio, pins):
        super().__init__(application)
        self.add_url_rule('/lights/1', view_func=self.all_l_on, methods=['GET',])
        self.add_url_rule('/lights/0', view_func=self.all_l_off, methods=['GET',])
        self.add_url_rule('/left_l_on', view_func=self.left_l_on, methods=['GET',])
        self.add_url_rule('/left_l_off', view_func=self.left_l_off, methods=['GET',])
        self.add_url_rule('/right_l_on', view_func=self.right_l_on, methods=['GET',])
        self.add_url_rule('/right_l_off', view_func=self.right_l_off, methods=['GET',])
        self.gpio = gpio
        self.pins = pins
        self.q_state = Queue()
        self.q_result = Queue()
        self.lights_process = LightsProcess(self.q_state, self.q_result, self.gpio, self.pins)
        self.lights_process.start()
 
    def __del__(self):
        self.q_state.put('EXIT')
        self.lights_process.join()

    def stop(self):
        self.q_state.put('EXIT')
        self.lights_process.join()

    # 1 -> OFF   !!!
    # 0 -> ON    !!!
    def all_l_on(self):
        self.q_state.put({
            'forward_left'  : 0,
            'forward_right' : 0,
            'backward_right': 0,
            'backward_left' : 0,
        })
        print(self.q_result.get())
        state = {'all_lights': 1}
        return jsonify(state), 200

    def all_l_off(self):
        self.q_state.put({
            'forward_left'  : 1,
            'forward_right' : 1,
            'backward_right': 1,
            'backward_left' : 1,
        })
        print(self.q_result.get())
        state = {'all_lights': 0}
        return jsonify(state), 200

    # def create_route(self, route, methods=['GET',]):

    # 1 -> OFF   !!!
    # 0 -> ON    !!!
    def left_l_on(self):
        self.q_state.put({
            'forward_left'  : 0,
            'backward_left' : 0,
        })
        print(self.q_result.get())
        state = {'left_lights': 1}
        return jsonify(state), 200

    # 1 -> OFF   !!!
    # 0 -> ON    !!!
    def left_l_off(self):
        self.q_state.put({
            'forward_left'  : 1,
            'backward_left' : 1,
        })
        print(self.q_result.get())
        state = {'left_lights': 0}
        return jsonify(state), 200

    # 1 -> OFF   !!!
    # 0 -> ON    !!!
    def right_l_on(self):
        self.q_state.put({
            'forward_right' : 0,
            'backward_right': 0,
        })
        print(self.q_result.get())
        state = {'right_lights': 1}
        return jsonify(state), 200

    # 1 -> OFF   !!!
    # 0 -> ON    !!!
    def right_l_off(self):
        self.q_state.put({
            'forward_right' : 1,
            'backward_right': 1,
        })
        print(self.q_result.get())
        state = {'right_lights': 0}
        return jsonify(state), 200


# class FlaskWrapper(object):
#     app = None

#     def __init__(self, name):
#         self.app = Flask(name)

#     def run(self):
#         self.app.run()

#     def add_endpoint(self, endpoint=None, endpoint_name=None, handler=None):
#         self.app.add_url_rule(endpoint, endpoint_name, EndpointAction(handler))

 
if __name__ == '__main__':
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(True)
    except ModuleNotFoundError:
        GPIO = None
    pins = {
        'forward_left'  : 31,
        'forward_right' : 36,
        'backward_right': 38,
        'backward_left' : 40,
    }
    lights_app = Lights(__name__, GPIO, pins)
    print(lights_app.q_state)
    
    lights_app.run(host='0.0.0.0', port=4999)
    lights_app.stop()
    try:
        GPIO.cleanup()
    except AttributeError as e:
        print(e)


