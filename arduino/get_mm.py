from serial import Serial
from time import sleep

def arduino_read(old_status):
	try:
		arduino_status  = str(ser.read_until('<').decode('ascii'))
		print(arduino_status)
		if len(arduino_status) > 1:
			indices = [i for i in range(len(''.join(arduino_status.split('\r')).split('\n'))) if ''.join(arduino_status.split('\r')).split('\n')[i] == '<']
			arduino_status = [float(''.join(arduino_status.split('\r')).split('\n')[i]) for i in range(indices[-2] + 1, indices[-1])]
			return {
    			'frw_rgt_dst'       : arduino_status[0],
    			'frw_lft_dst'       : arduino_status[2],
    			'lft_frw_dst'       : arduino_status[1],
    			'lft_bkw_dst'       : arduino_status[5],
    			'rgt_frw_dst'       : arduino_status[3],
    			'rgt_bkw_dst'       : arduino_status[4],
    			'v_battery'         : arduino_status[6],
    			'nc'                : arduino_status[7],
    			'current_lft_motor' : arduino_status[8],
	    		'current_rgt_motor' : arduino_status[9],
	    		'frw_rgt_ik'        : arduino_status[10],
	    		'frw_lft_ik'        : arduino_status[11],
	    		'lft_ik'            : arduino_status[12],
	    		'rgt_ik'            : arduino_status[13],
	    		'bkw_ik'            : arduino_status[14],
    		}
		else:
			return old_status
	except Exception as e:
		print(e)
		return old_status

if __name__ == "__main__":
    ser = Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.01)
    sleep(1.5)
    ser.write('>')
    distance = {
    	'frw_rgt_dst'       : 0,
    	'frw_lft_dst'       : 0,
    	'lft_frw_dst'       : 0,
    	'lft_bkw_dst'       : 0,
    	'rgt_frw_dst'       : 0,
    	'rgt_bkw_dst'       : 0,
    	'v_battery'         : 0,
    	'nc'                : 0,
    	'current_lft_motor' : 0,
	    'current_rgt_motor' : 0,
	    'frw_rgt_ik'        : 0,
	    'frw_lft_ik'        : 0,
	    'lft_ik'            : 0,
	    'rgt_ik'            : 0,
	    'bkw_ik'            : 0,
    }
    distance = arduino_read(distance)
    print(distance)
    ser.close()
    
