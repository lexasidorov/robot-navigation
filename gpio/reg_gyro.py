import Adafruit_ADS1x15
import RPi.GPIO as GPIO
import smbus 

from time import sleep, time

dt = 0.01
#trg_spd = 10
 
gz = 0
FREQUENCY = 100
left_encoder = 0
right_encoder = 0

def init_gyro(bus):
    bus.write_byte_data(0x68, 0x3E, 0x01)
    bus.write_byte_data(0x68, 0x16, 0x18) 
    sleep(0.5)

def get_z(bus):
    data = bus.read_i2c_block_data(0x68, 0x1D, 6)
    zGyro = data[4] * 256 + data[5] 
    if zGyro > 32767 : 
        zGyro -= 65536 
    return zGyro

def left_forward(spd):
    left_pwm = max(min(spd,100),0)
    return left_pwm

def right_forward(spd):
    right_pwm = max(min(spd,100),0)
    return right_pwm

def light_on():
    GPIO.output(31, 0)
    GPIO.output(36, 0)
    GPIO.output(38, 0)
    GPIO.output(40, 0)

def light_off():
    GPIO.output(31, 1)
    GPIO.output(36, 1)
    GPIO.output(38, 1)
    GPIO.output(40, 1)

def left_counter(channel):
    global left_encoder 
    left_encoder = left_encoder + 1

def right_counter(channel):
    global right_encoder
    right_encoder = right_encoder + 1

if __name__ == "__main__":
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(True)  
    
    GPIO.setup([31, 36, 38, 40], GPIO.OUT, initial=GPIO.HIGH)   
    
    GPIO.setup([33, 32], GPIO.OUT)
    
    GPIO.setup([35, 26], GPIO.OUT, initial=GPIO.LOW)
    
    GPIO.output(26, 1)
    
    GPIO.setup([16, 18], GPIO.IN)
    
    GPIO.add_event_detect(16, GPIO.RISING)
    GPIO.add_event_callback(16, left_counter)
    
    GPIO.add_event_detect(18, GPIO.RISING)
    GPIO.add_event_callback(18, right_counter)
    
    rgt_pwm = GPIO.PWM(33, FREQUENCY)
    lft_pwm = GPIO.PWM(32, FREQUENCY)
    
    bus = smbus.SMBus(1) 
    #init_gyro(bus)
    
    adc = Adafruit_ADS1x15.ADS1115()
    
    
    z = 0
    
    v_left  = 0
    v_right = 0
    
    #Kp = 5
    Kp_l = 1
    Kp_r = 1
    #Ki = 0.0000015
    Ki = 0
    Kp_angle = 1/2
    
    S_e_left = 0
    S_e_right = 0
    
    v_0_left  = 40
    v_0_right = 40
    
    S_v_left = 0
    S_v_right = 0
    
    S_pwm_left  = 0
    S_pwm_right = 0
    
    pwm_left, pwm_right = 0, 0
    S_gz = 0
    gz = 0
    light_off()
    angle_z = 0
    
    e = 0
    v0 = 40
    pwm = 0
    v = 0
    S_pwm = 0
    
    encoder_l = 0
    encoder_r = 0
    
    l_right = 0
    l_left = 0
    mean_gz = 0
    '''
    a_gz = np.zeros(14936)
    #print("get mean")
    for i in range(len(a_gz)):
        a_gz[i] = get_z(bus)
    
    mean_gz = a_gz.mean()
    del a_gz
    '''
    #print('ready!')
    light_on()
    
    t = time()
    while(1):
        try:
            e = v0 - v
            S_pwm += Kp_l * e - pwm
            pwm = S_pwm / 30000
            
            pwm_left  = 10 + Kp_angle * (z - angle_z)
            pwm_right = 10 - Kp_angle * (z - angle_z)
            
            lft_pwm.start(max(min(pwm_left,100),5))
            rgt_pwm.start(max(min(pwm_right,100),5))
            
            if (l_left + l_right) / 2 > 3000:
                light_off()
                lft_pwm.stop()
                rgt_pwm.stop()
                GPIO.cleanup()
                #print("bye")
                break
            
            dt = time() - t

            if dt > 0.10:
                S_gz += (get_z(bus) - mean_gz) * dt - gz
                gz = S_gz / 10
                angle_z += gz / 13.69
                
                S_v_left += left_encoder / dt - v_left
                v_left = S_v_left / 200
                
                S_v_right += right_encoder / dt - v_right
                v_right = S_v_right /200
                
                l_left += left_encoder * (800.7/187.9)
                l_right += right_encoder * (800.7/165.1)
                left_encoder = 0
                right_encoder = 0
                
                v = (v_left + v_right) / 2
                t = time()
            print(f"{time()} {adc.read_adc(0, gain=2/3)} {adc.read_adc(1, gain=2/3)} {adc.read_adc(2, gain=2/3)} {adc.read_adc(3, gain=2/3)}")
            #print(f"{time()} {left_encoder} {right_encoder} {gz} {angle_z} {pwm_left} {pwm_right}")
            #        1           2               3            4       5          6            7
        except Exception as e:
            print(e)
            light_off()
            lft_pwm.stop()
            rgt_pwm.stop()
            GPIO.cleanup()
            #print("bye")
            break
        except KeyboardInterrupt:
            light_off()
            lft_pwm.stop()
            rgt_pwm.stop()
            GPIO.cleanup()
            #print(f"{encoder_l}  ---  {encoder_r}")
            #print("  -bye-  ")
            break
