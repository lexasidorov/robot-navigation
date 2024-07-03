import sys

try:
    import RPi.GPIO as GPIO
    USEGPIO = True
except ModuleNotFoundError:
    USEGPIO = False

from multiprocessing import Process, Queue
from queue import Empty
from time import sleep

PWM_EXIT = -1000

class PwmProcess(Process):
    def __init__(self, name, gpio, pins, q_pwm_value, frequency=50, debug=False, debug_delay=1000):
        super().__init__(target=self.run)
        
        self.__name = name
        self.__gpio = gpio
        self.__frequency = frequency
        self.__q_pwm_value = q_pwm_value
        self.__debug_delay = debug_delay
        self.__debug = debug
        self.__loop_count = 0
        self.__pwm = 0

        self.__pwm_pin = pins[f'pwm_{self.__name}']
        self.__backward = pins['backward']
        self.__forward = pins['forward']

        self.__gpio.setup(list(pins.values()), self.__gpio.OUT, initial=self.__gpio.LOW)


    def __do_debug__(self):
        self.__loop_count += 1
        if self.__loop_count == self.__debug_delay:
            print(f'[ PWM_PROCESS ] {self.__name} pwm {self.__pwm}', file=sys.stderr)
            self.__loop_count = 0


    def run(self):
        while 1:
            if self.__debug: self.__do_debug__()
            try:
                _in = self.__q_pwm_value.get(block=False)
                if _in == PWM_EXIT:
                    break
                else:
                    self.__pwm = abs(_in)
                    _en = 1 if self.__pwm > 0 else 0
                    if _in > 0:
                        self.__gpio.output(self.__forward,  _en)
                        self.__gpio.output(self.__backward,   0)
                    elif _in < 0:
                        self.__gpio.output(self.__forward,    0)
                        self.__gpio.output(self.__backward, _en)
                    T = 1 / self.__frequency # 1/100, pwm = 10 
                    dT = T / 100             # 1/10000, pwm = 10
                    dT1 = dT * self.__pwm    # 1/1000, pwm = 10
                    dT2 = T - dT1            # 10/1000 - 1/1000 = 9/1000
            except Empty:
                pass
            if self.__pwm > 0:
                try:
                    self.__gpio.output(self.__pwm_pin, 1)
                    sleep(dT1)
                    self.__gpio.output(self.__pwm_pin, 0)
                    sleep(dT2)
                except KeyboardInterrupt as e:
                    self.__gpio.output(self.__pwm_pin, 0)
                    raise e

pins = {
    'pwm_left'  : 37,
    'pwm_right' : 32,
    'backward'  : 26,
    'forward'   : 35,
}

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(True)
    
    lft_pwm_queue = Queue()
    rgt_pwm_queue = Queue()
    print(51, 'rgt_pwm_queue = Queue(), lft_pwm_queue = Queue()')


    lft_pwm_process = PwmProcess('left', GPIO, pins, lft_pwm_queue, debug=1, debug_delay=100)
    rgt_pwm_process = PwmProcess('right', GPIO, pins, rgt_pwm_queue, debug=1, debug_delay=100)
    print(63, "rgt_pwm_process = PwmProcess('right', gpio, pins['pwm_right'], rgt_pwm_queue)")
    lft_pwm_process.start()
    rgt_pwm_process.start()
    print(66, "rgt_pwm_process.start()")

    for p in [-40,-30,-20,-10,10,20,30,40,PWM_EXIT]:
        print(69, p)
        lft_pwm_queue.put(p)
        rgt_pwm_queue.put(p)
        sleep(10)

    for p in [lft_pwm_process, rgt_pwm_process]:
        print(75, p)
        p.join()
