

SERVICES = {
	'ports': 
		{
			'web_interface': 5000,
			'lights': 4999,
			'drive': 4998,
			'visual_nav': 4997,
			'lidars': 4996,
			'sensors': 4995,
            'middleware': 4994,
            'lidars_x2': 4993,
            'parking': 4992
		}

}
# input: USB 2.0 Camera: USB 2.0 Camera as /devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.2/1-1.2.3/1-1.2.3.2/1-1.2.3.2:1.0/input/input3

# WASH_TYPES = {"dry": "wash_start",
#               "wet": "wash_start"}

WASH_TYPES = [
    {'id': 'dry', 'name': 'Сухая уборка', 'command': 'wash_start'},
    {'id': 'wet', 'name': 'Влажная уборка', 'command': 'wash_start'}
]

COMMAND_TO_SERVICE = {
    'go': {
        'service': 'drive',
        'url': 'http://localhost:%s/fwd/1/%s'
    },
    'follow_wall': {
        'service': 'drive',
        'url': 'http://localhost:%s/fwd/1/%s'  
    },
    'left': {
        'service': 'drive',
        'url': 'http://localhost:%s/lft/1/%s'
    },
    'right': {
        'service': 'drive',
        'url': 'http://localhost:%s/rgt/1/%s'
    },
    'back': {
        'service': 'drive',
        'url': 'http://localhost:%s/bck/1/%s'
    },
    'stop': {
        'service': 'drive',
        'url': 'http://localhost:%s/stop',
    },
    'drive_cmds_state': {
        'service': 'drive',
        'url': 'http://localhost:%s/state/get'
    },
    'localize_visual' : {
        'service': 'visual_nav',
        'url': 'http://localhost:%s/v_coords/calc'
    },
    'localize_lidar' : {
        'service': 'lidars',
        'url': 'http://localhost:%s/state/get'
    },
    'lights_on': {
        'service': 'lights',
        'url': 'http://localhost:%s/lights/1',
    },
    'lights_off': {
        'service': 'lights',
        'url': 'http://localhost:%s/ligths/0',
    },
    'l_on': {
        'service': 'lights',
        'url': 'http://localhost:%s/left_l_on', 
    },
    'l_off': {
        'service': 'lights',
        'url': 'http://localhost:%s/left_l_off',
    },
    'r_on': {
        'service': 'lights',
        'url': 'http://localhost:%s/right_l_on',
    },
    'r_off': {
        'service': 'lights',
        'url': 'http://localhost:%s/right_l_off',
    },
}