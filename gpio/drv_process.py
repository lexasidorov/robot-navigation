import json
import numpy as np
import sys

from gyroscope import Gyroscope
from pwm_process import PwmProcess, PWM_EXIT

from math import sin, cos, radians
from multiprocessing import Process, Queue
from queue import Empty
from time import sleep, time

BRAKING_DISTANCE = 200
DEFAULT_BRAKING_ANGLE = 25
DEFAULT_MAX_PWM = 100
DEFAULT_MIN_PWM = 1
DEFAULT_PWM_BACK = 15
DEFAULT_PWM_VALUE = 15
DEFAULT_SPEED = 200
DT = 0.005
LW_COEFF = 11.821519634164229
MIN_SPEED = 1
RW_COEFF = 12.011542972247089


try:
    coeffs = json.load(open('/home/pi/robot-visual-nav/reg_coefs.json'))
    KL = coeffs[0] #, 63.98344012992514, 63.98344012992514, 63.98344012992514*1.5, 63.98344012992514*1.5, 63.98344012992514*1.5, 63.98344012992514/1.5, 63.98344012992514/1.5, 63.98344012992514/1.5]
    KR = coeffs[1] #, 63.98344012992514, 63.98344012992514, 63.98344012992514*1.5, 63.98344012992514*1.5, 63.98344012992514*1.5, 63.98344012992514/1.5, 63.98344012992514/1.5, 63.98344012992514/1.5]
except FileNotFoundError:
    KL = [63.98344012992514/0.3879781420765028]
    KR = [63.98344012992514/0.3879781420765028]
    
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(True)

    USEGPIO = True
except ModuleNotFoundError:
    USEGPIO = False

class DrvProcess(Process):

    def __init__(self, q_cmd, q_result, debug=False):
        super().__init__()

        self.__do_debug = debug
        self.__set_initial_values()
        self.__configure_pins()
        self.__configure_gyro()
        self.__init_pwm_processes()
        self.__configure_queues(q_cmd, q_result)
        self.__init_state_dictionary()


    def __set_initial_values(self):
        self.KpA, self.KiA, self.KdA, self.KpV, self.KiV, self.KdV= json.load(open('/home/pi/k.json'))
        self.__eA, self.__eAI, self.__eAD, self.__eA0 = 0, 0, 0, 0
        self.__eV, self.__eVI, self.__eVD, self.__eV0 = 0, 0, 0, 0

        self.__S_left, self.__S_right, self.__v, self.__v0 = 0, 0, 0, 0
        self.__v_left, self.__v_right = 0, 0
        
        self.__average_pwm, self.__default_pwm = 0, 0
        self.__passed, self.__to_go = 0, 0
        self.__rgt_pwm, self.__lft_pwm = 0, 0
        self.__drive_state = 'stop'
        self.__z0, self.__angle_z, self.__angle_z_acc = 0, 0, 0
        self.__passed_step, self.__passed_acc0, self.__passed_acc = 0, 0, 0

        self.__dt = DT
        self.__coords0 = (0,0,0)
        self.__coords = (0,0,0)
        self.__target_coords = (0,0)

        self.__passed_pseudo = 0
        self.__nk = 0

        self.__right_wheel_coeff = RW_COEFF
        self.__left_wheel_coeff = LW_COEFF

        self.__l_lft_rot = 0
        self.__l_rgt_rot = 0

        self.__corr_angle = 0

        self.__saved_lft_pwm = -100000
        self.__saved_rgt_pwm = -100000

        self.__avoid_cmd_count = 0
        self.__return_cmd_count = 0

    def __configure_pins(self):
        self.pins = {
            'pwm_left'  : 37,
            'pwm_right' : 32,
            'backward'  : 26,
            'forward'   : 35,
        }
        self.pins_encoder = {
            'encoder_left'  : 40,
            'encoder_right' : 16,
        }
        try:
            assert USEGPIO, '[ ERROR ] No GPIO or smbus detected!'
            self.gpio = GPIO
            self.gpio.setup(list(self.pins_encoder.values()), self.gpio.IN, pull_up_down=GPIO.PUD_DOWN)
        except AssertionError as e:
            print(e)


    def __configure_gyro(self):
        self.g = Gyroscope(debug=False)


    def __configure_encoders(self):

        def __counter_left__(channel):
            #self.__now_lft_encoder = self.gpio.input(self.pins_encoder['encoder_left'])
            #if self.__now_lft_encoder != self.__old_lft_encoder:
            #    self.__lft_encoder += 1
            #self.__old_lft_encoder = self.__now_lft_encoder
            if self.gpio.input(self.pins_encoder['encoder_left']):
                self.__lft_encoder += 1

        
        def __counter_right__(channel):

            #self.__now_rgt_encoder = self.gpio.input(self.pins_encoder['encoder_right'])
            #if self.__now_rgt_encoder != self.__old_rgt_encoder:
            #    self.__rgt_encoder += 1
            #self.__old_rgt_encoder = self.__now_rgt_encoder
            if self.gpio.input(self.pins_encoder['encoder_right']):
                self.__rgt_encoder += 1


        self.__rgt_encoder, self.__lft_encoder = 0, 0
        self.__l_right, self.__l_left = 0, 0
        self.__v_left, self.__v_right = 0, 0
        self.__left, self.__right = 0, 0
        self.__S_left, self.__S_right = 0, 0
        
        self.__now_lft_encoder = self.gpio.input(self.pins_encoder['encoder_left'])
        self.__now_rgt_encoder = self.gpio.input(self.pins_encoder['encoder_right'])
        self.__old_lft_encoder = self.__now_lft_encoder
        self.__old_rgt_encoder = self.__now_rgt_encoder
        
        self.gpio.add_event_detect(self.pins_encoder['encoder_left'],   self.gpio.RISING)
        self.gpio.add_event_callback(self.pins_encoder['encoder_left'], __counter_left__)
    
        self.gpio.add_event_detect(self.pins_encoder['encoder_right'],   self.gpio.RISING)
        self.gpio.add_event_callback(self.pins_encoder['encoder_right'], __counter_right__)

        if self.__do_debug: open('/tmp/enclog','w').write(f"{self.__lft_encoder} {self.__rgt_encoder}\n")


    def __init_pwm_processes(self):
        self.__lft_pwm_queue = Queue()
        self.__rgt_pwm_queue = Queue()
        self.__lft_pwm_process = PwmProcess('left',  GPIO, self.pins, self.__lft_pwm_queue, debug=0, debug_delay=1000, frequency=242)
        self.__rgt_pwm_process = PwmProcess('right', GPIO, self.pins, self.__rgt_pwm_queue, debug=0, debug_delay=1000, frequency=242)
        self.__lft_pwm_process.start()
        self.__rgt_pwm_process.start()


    def __configure_queues(self, q_cmd, q_result):
        self.cmd = None
        self.q_cmd = q_cmd
        self.q_result = q_result


    def __init_state_dictionary(self):
        self.state = {
            'angle_z': self.__angle_z,
            'avoid_cmd_count': self.__avoid_cmd_count,
            'return_cmd_count': self.__return_cmd_count,
            'coords': self.__coords,
            'coords0': self.__coords0,
            'corr_angle': self.__corr_angle,
            'default_pwm': self.__default_pwm,
            'do_debug': self.__do_debug,
            'drive_state': self.__drive_state,
            'l_lft_rot' : self.__l_lft_rot,
            'l_rgt_rot' : self.__l_rgt_rot,
            'lft_pwm': self.__lft_pwm,
            'passed': self.__passed,
            'passed_acc': self.__passed_acc,
            'passed_step': self.__passed_step,
            'rgt_pwm': self.__rgt_pwm,
            'S_v_left': self.__S_left,
            'S_v_right': self.__S_right,
            'to_go': self.__to_go,
            'v': self.__v,
            'v_left': self.__v_left,
            'v_right': self.__v_right,
            'z0': self.__z0,
        }


    def __calculate_pwm(self):
        if self.__drive_state == 'go' or self.__drive_state == 'back':
            if self.__drive_state == 'go':
                self.__average_pwm = DEFAULT_PWM_VALUE \
                    if abs(self.__to_go - self.__passed) > BRAKING_DISTANCE \
                    else DEFAULT_PWM_VALUE / 3
            elif self.__drive_state == 'back':
                self.__average_pwm = DEFAULT_PWM_BACK \
                    if abs(self.__to_go - self.__passed) > BRAKING_DISTANCE \
                    else DEFAULT_PWM_BACK / 2

            self.__eV = self.__v0 - self.__v
            self.__eVI += self.__eV
            self.__eVD = self.__eV - self.__eV0
            self.__eV0 = self.__eV

            self.__eA = self.__z0 - self.__angle_z
            self.__eAI += self.__eA
            self.__eAD = self.__eA - self.__eA0
            self.__eA0 = self.__eA

            V = self.KpA * self.__eA + self.KiA * self.__eAI + self.KdA * self.__eAD
            # P = self.KpV * self.__eV + self.KiV * self.__eVI + self.KdV * self.__eVD
            P = 0
            V = - V if self.__drive_state == 'back' else V
            # P = - P if self.__drive_state == 'back' else P

            self.__rgt_pwm = max(min((self.__average_pwm + P - V),DEFAULT_MAX_PWM),DEFAULT_MIN_PWM)
            self.__lft_pwm = max(min((self.__average_pwm + P + V),DEFAULT_MAX_PWM),DEFAULT_MIN_PWM)
        
        elif self.__drive_state == 'left' or self.__drive_state == 'back_right':
            self.__e = abs(self.__z0 - self.__angle_z)
            if self.__e > self.__braking_angle :
                self.__rgt_pwm, self.__lft_pwm = DEFAULT_PWM_VALUE , 1
            else:
                self.__rgt_pwm, self.__lft_pwm = DEFAULT_PWM_VALUE / 2, 1
        
        elif self.__drive_state == 'right' or self.__drive_state == 'back_left':
            self.__e = abs(self.__z0 - self.__angle_z)
            if self.__e > self.__braking_angle :
                self.__rgt_pwm, self.__lft_pwm = 1, DEFAULT_PWM_VALUE 
            else:
                self.__rgt_pwm, self.__lft_pwm = 1, DEFAULT_PWM_VALUE / 2
        
        elif self.__drive_state == 'stop':
            self.__rgt_pwm, self.__lft_pwm = 0,0
            
        elif 'debug' in self.__drive_state:
            pass
            
        else:
            self.__rgt_pwm, self.__lft_pwm = 0,0

        if self.__drive_state == 'back':
            self.state.update({'rgt_pwm':-self.__rgt_pwm})
            self.state.update({'lft_pwm':-self.__lft_pwm})
        else:
            self.state.update({'rgt_pwm':self.__rgt_pwm})
            self.state.update({'lft_pwm':self.__lft_pwm})

    
    def __update_angle(self):
        # k = 85/90 if 'right' in self.__drive_state else 1
        self.__angle_z = -self.g.get_z(self.__corr_angle, abs(self.__v) < MIN_SPEED) 
        self.state.update({'angle_z':self.__angle_z})


    def __calculate_coords(self):
        self.__passed_step = self.__passed_acc - self.__passed_acc0
        self.__passed_acc0 = self.__passed_acc
        y = self.__coords[1]
        y += self.__passed_step * cos(radians(self.__angle_z))
        x = self.__coords[0]
        x += self.__passed_step * sin(radians(self.__angle_z))
        self.__coords = (x,y,self.__angle_z) # calculate here
        self.__coords0 = self.__coords
        self.state.update({'coords': self.__coords, 'coords0': self.__coords0})


    def __reset_passed(self):
        self.__l_left = 0
        self.__l_right = 0
        self.__left = 0
        self.__right = 0
        self.__lft_encoder = 0
        self.__passed = 0
        self.__rgt_encoder = 0
        self.__S_left = 0
        self.__S_right = 0
        self.__v = 0
        self.__v_left = 0
        self.__v_right = 0


    def __update_passed(self):
        if self.__do_debug: open('/tmp/enclog','a').write(f"{self.__lft_encoder} {self.__rgt_encoder}\n")

        self.__lft_encoder = 10 if self.__lft_encoder > 10 else self.__lft_encoder
        self.__rgt_encoder = 10 if self.__rgt_encoder > 10 else self.__rgt_encoder
        self.__lft_encoder = - self.__lft_encoder if self.__drive_state == 'back' else self.__lft_encoder
        self.__rgt_encoder = - self.__rgt_encoder if self.__drive_state == 'back' else self.__rgt_encoder
        self.__S_left += self.__lft_encoder - self.__left
        self.__left = self.__S_left / 20
                
        self.__S_right += self.__rgt_encoder - self.__right
        self.__right = self.__S_right / 20

        try:
            kl = KL[self.__nk]
            kr = KR[self.__nk]
        except IndexError:
            kl = KL[0]
            kr = KR[0]

        self.__v_left = self.__left * kl
        self.__v_right = self.__right * kr

        # self.__l_left += self.__lft_encoder  # / 130.20499210249923 # * (800.7/187.9)
        # self.__l_right += self.__rgt_encoder # / 82.50582124548092 # * (800.7/165.1)
        self.__l_left  += self.__v_left  * self.__dt
        self.__l_right += self.__v_right * self.__dt
        self.__l_lft_rot += self.__v_left  * self.__dt
        self.__l_rgt_rot += self.__v_right * self.__dt
        
        self.__passed = np.mean((self.__l_left, self.__l_right))
        self.__passed_acc += np.mean((self.__v_left  * self.__dt, self.__v_right * self.__dt))


        self.__lft_encoder = 0
        self.__rgt_encoder = 0
                
        self.__v = (self.__v_left + self.__v_right) / 2

        self.state.update({
            'angle_z': round(self.__angle_z, 4),
            'avoid_cmd_count': self.__avoid_cmd_count,
            'return_cmd_count': self.__return_cmd_count,
            'coords': self.__coords,
            'coords0': self.__coords0,
            'corr_angle': round(self.__corr_angle, 4),
            'default_pwm': round(self.__default_pwm, 4),
            'do_debug': round(self.__do_debug, 4),
            'drive_state': self.__drive_state,
            'l_lft_rot' : round(self.__l_lft_rot, 4),
            'l_rgt_rot' : round(self.__l_rgt_rot, 4),
            'lft_pwm': round(self.__lft_pwm, 4),
            'passed': round(self.__passed, 4),
            'passed_acc': round(self.__passed_acc, 4),
            'passed_step': round(self.__passed_step, 4),
            'rgt_pwm': round(self.__rgt_pwm, 4),
            'S_v_left': round(self.__S_left, 4),
            'S_v_right': round(self.__S_right, 4),
            'to_go': round(self.__to_go, 4),
            'v': round(self.__v, 4),
            'v_left': round(self.__v_left, 4),
            'v_right': round(self.__v_right, 4),
            'z0': round(self.__z0, 4),
        })


    def __update_state(self):
        previous_state = self.__drive_state
        if self.__drive_state == 'go' or self.__drive_state == 'back':
            if abs(self.__passed) >= self.__to_go:
                self.__drive_state = 'stop'
        
        elif self.__drive_state == 'left':
            if self.__angle_z <= self.__z0:
                self.__drive_state = 'stop'
        
        elif self.__drive_state == 'right':
            if self.__angle_z >= self.__z0:
                self.__drive_state = 'stop'

        elif self.__drive_state == 'back_left':
            if self.__angle_z <= self.__z0:
                self.__drive_state = 'stop'
        
        elif self.__drive_state == 'back_right':
            if self.__angle_z >= self.__z0:
                self.__drive_state = 'stop'

        if previous_state != self.__drive_state and \
           self.__avoid_cmd_count and \
           self.__drive_state == 'stop':
            self.__avoid_cmd_count -= 1
            self.state.update({'avoid_cmd_count': self.__avoid_cmd_count})

        if previous_state != self.__drive_state and \
           self.__return_cmd_count and \
           self.__drive_state == 'stop':
            self.__return_cmd_count -= 1
            self.state.update({'return_cmd_count': self.__return_cmd_count})

        self.state.update({'drive_state': self.__drive_state})
        # print(f'[ DRV_INFO ] Drive state is {self.__drive_state}.', file=sys.stderr)


    def __get_state(self):
        self.q_result.put({**self.state})


    def __calc_braking_angle(self, angle_to_turn):
        rez = min(angle_to_turn / 5 if angle_to_turn > DEFAULT_BRAKING_ANGLE else 0, DEFAULT_BRAKING_ANGLE)
        return rez


    def run(self):
        self.__configure_encoders()
        self.__update_angle()
        t = time()
        t0 = t
        while 1:
            to_log = ", ".join([ str(round(x, 2)) for x in [
                                                            # self.__l_lft_rot,
                                                            # self.__l_rgt_rot,
                                                            # self.__v0, 
                                                            # self.__v, 
                                                            # self.__l_left,
                                                            # self.__l_right,
                                                            # self.__v_left, 
                                                            # self.__v_right, 
                                                            # self.__lft_pwm, 
                                                            # self.__rgt_pwm, 
                                                            self.__avoid_cmd_count,
                                                            # self.__z0,
                                                            # self.__angle_z,
                                                            # self.__coords[0],
                                                            # self.__coords[1],
                                                            ]])
            if self.__do_debug: print(f'[ DRV_PROCESS ] {time()}, {self.__drive_state}, {to_log}', file=sys.stderr) # {self.__passed}, {self.__coords[0]}, {self.__coords[1]},
            try:
                self.cmd = self.q_cmd.get(block=False)
                if self.cmd['action'] == 'go':
                    self.__drive_state = 'go'
                    self.__v0 = DEFAULT_SPEED
                    self.__reset_passed()
                    self.__to_go = self.cmd['value']
                    # self.__z0 = self.__angle_z
                
                elif self.cmd['action'] == 'back':
                    self.__drive_state = 'back'
                    self.__v0 =  - DEFAULT_SPEED # * 1.5
                    self.__reset_passed()
                    self.__to_go = self.cmd['value']
                
                elif self.cmd['action'] == 'stop':
                    self.__drive_state = 'stop'
                    self.__average_pwm = 0
                    self.__v0 = 0
                
                elif self.cmd['action'] == 'left':
                    self.__drive_state = 'left'
                    self.__z0 -= self.cmd['value']
                    self.__braking_angle = self.__calc_braking_angle(self.cmd['value'])
                
                elif self.cmd['action'] == 'right':
                    self.__drive_state = 'right'
                    self.__z0 += self.cmd['value']
                    self.__braking_angle = self.__calc_braking_angle(self.cmd['value'])

                
                elif self.cmd['action'] == 'back_left':
                    self.__drive_state = 'back_left'
                    self.__z0 -= self.cmd['value']
                    self.__braking_angle = self.__calc_braking_angle(self.cmd['value'])

                
                elif self.cmd['action'] == 'back_right':
                    self.__drive_state = 'back_right'
                    self.__z0 += self.cmd['value']
                    self.__braking_angle = self.__calc_braking_angle(self.cmd['value'])

                
                elif self.cmd['action'] == 'reset_angle':
                    self.__z0 = self.__angle_z
                
                elif self.cmd['action'] == 'set_angle':
                    self.__corr_angle = self.cmd['value']
                    
                elif self.cmd['action'] == 'reset_passed':
                    self.__passed = self.__to_go
                
                elif self.cmd['action'] == 'set_coords':
                    x, y = self.cmd['value']
                    z = self.__coords[2]
                    self.__coords = (x, y, z)
                
                elif self.cmd['action'] == 'state':
                    try:
                        x = self.cmd['source']
                    except:
                        x = 'unknown'
                    # print(f'[ INFO ] get state source = {x}', flush=1)
                    self.__get_state()

                elif self.cmd['action'] == 'set_avoid_cmd_count':
                    self.__avoid_cmd_count = self.cmd['value']

                elif self.cmd['action'] == 'set_return_cmd_count':
                    self.__return_cmd_count = self.cmd['value']
                
                elif self.cmd['action'] == 'EXIT':
                    break

                # elif self.cmd['action'] == 'debug_left':
                #     self.__drive_state = 'debug_left'
                #     self.__z0 -= self.cmd['value']
                
                # elif self.cmd['action'] == 'debug_right':
                #     self.__drive_state = 'debug_right'
                #     self.__z0 += self.cmd['value']
                
                if self.cmd['action'] in ['left', 'right', 'back_left', 'back_right']:
                    self.__l_lft_rot = 0
                    self.__l_rgt_rot = 0

            except Empty:
                pass

            self.__update_angle()
            self.__update_passed()
            self.__calculate_pwm()
            self.__set_pwm()
            self.__update_state()
            self.__calculate_coords()
            # self.__debug_angle(stop=100000)
            sleep(DT)
            self.__dt, t = time() - t, time()
            self.__nk = int(t-t0)
    

    # def __debug_angle(self, stop):
    #     if abs(self.__angle_e) < stop:
    #         if self.__drive_state == 'debug_left':
    #             if abs(self.__angle_e) / stop < 3/4:
    #                 self.__rgt_pwm, self.__lft_pwm = DEFAULT_PWM_VALUE , 1
    #             else:
    #                 self.__rgt_pwm, self.__lft_pwm = DEFAULT_PWM_VALUE / 4, 1
            
    #         elif self.__drive_state == 'debug_right':
    #             if abs(self.__angle_e) / stop < 3/4:
    #                 self.__rgt_pwm, self.__lft_pwm = 1, DEFAULT_PWM_VALUE 
    #             else:
    #                 self.__rgt_pwm, self.__lft_pwm = 1, DEFAULT_PWM_VALUE / 4
    #     else:
    #         self.__drive_state == 'stop'
    #         self.__rgt_pwm, self.__lft_pwm = 0, 0

    #     self.state.update({'drive_state': self.__drive_state})


    def __set_pwm(self):
        def is_lft_pwm_changed(a_pwm):
            rez = self.__saved_lft_pwm != a_pwm
            self.__saved_lft_pwm = a_pwm
            return rez

        def is_rgt_pwm_changed(a_pwm):
            rez = self.__saved_rgt_pwm != a_pwm
            self.__saved_rgt_pwm = a_pwm
            return rez

        if 'back' in self.__drive_state:
            if is_lft_pwm_changed(self.__lft_pwm): self.__lft_pwm_queue.put(-self.__lft_pwm)
            if is_rgt_pwm_changed(self.__rgt_pwm): self.__rgt_pwm_queue.put(-self.__rgt_pwm)
        else:
            if is_lft_pwm_changed(self.__lft_pwm): self.__lft_pwm_queue.put(self.__lft_pwm)
            if is_rgt_pwm_changed(self.__rgt_pwm): self.__rgt_pwm_queue.put(self.__rgt_pwm)


    def stop(self):
        print('stop')
        self.__lft_pwm_queue.put(PWM_EXIT)
        self.__rgt_pwm_queue.put(PWM_EXIT)
        self.__lft_pwm_process.join()
        self.__rgt_pwm_process.join()

        self.gpio.output(self.pins['backward'], 0)
        self.gpio.output(self.pins['forward'], 0)

        self.gpio.remove_event_detect(self.pins_encoder['encoder_left'])
        self.gpio.remove_event_detect(self.pins_encoder['encoder_right'])
        self.gpio.cleanup()


if __name__ == '__main__':
    try:
        cmds = []
        for i in range(1,len(sys.argv))[::2]:
            a_cmd = sys.argv[i]
            a_value = int(sys.argv[i+1])
            cmds += [(a_cmd, a_value)]
    except Exception as e:
        print(f'''Use $ {sys.argv[0]} [action value action value ...]+''')
        print(e)
        exit(0)
    print(273, 'q_state, q_result = Queue(), Queue()')
    q_cmd, q_result = Queue(), Queue()
    print(275, 'drv_proc = DrvProcess(self.q_state, self.q_result)')
    drv_proc = DrvProcess(q_cmd, q_result, debug=1)
    drv_proc.start()
    sleep(1)
    for cmd, value in cmds:
        q_cmd.put({'action':cmd,'value':value})
        for i in range(300):
            q_cmd.put({'action':'state'})
            sleep(0.1)
            try:
                rez = q_result.get()
                #pprint(rez)
                state = rez['drive_state']
                if state == 'stop':
                    break
            except Exception as e:
                raise e
    q_cmd.put({'action':'stop'})
    sleep(1)
    q_cmd.put({'action':'EXIT'})
    sleep(1)
    drv_proc.stop()
    print(284, 'drv_proc.join()')
    drv_proc.join()
    # del drv_proc