import numpy as np
import sys

from datetime import datetime
from flask import Flask, jsonify, request
from json import load, dumps, loads
from requests import get, exceptions, post
from threading import Thread
from time import sleep, time
import subprocess
import crontab

import settings
from settings import SERVICES, COMMAND_TO_SERVICE
# from flask_crontab import Crontab
# from descision_maker import DescisionMaker

# PATH = [
#     ('follow_left', 2500),
#     # ('go', 2000),
#     # ('right', 180),
#     # ('go', 2000),
#     # ('left', 180),
#     # ('go', 2000),
#     # ('right', 180),
#     # ('go', 2000),
#     # ('left', 180),
#     # ('go', 2000),
#     # ('right', 180),
#     # ('go', 2000),
#     # ('left', 180),
#     # ('go', 2000),
#     # ('right', 180),
#     # ('go', 2000),
#     # ('left', 180),
# ]

# The last cmds 
# python3 drv_process.py go 750 left 90 go 500 right 90 go 400 left 180 go 1500 right 180 go 1500 left 180 go 1500 right 180 go 1300 right 90 go 1900 left 90 back 600 back 1200

# l_angles = [89.93, 89.91, 89.92, 89.95]
# r_w_mv = [8.98, 6.15, 4.34, 9.25]
# l_wheel = [1049.39, 1095.44, 1023.07, 1095.30]
# l_coeff = np.median([ (l_wheel[i]-r_w_mv[i])/l_angles[i] for i in range(len(l_wheel)) ])
# r_angles = [90.0, 90., 89.36, 89.35]
# r_wheel = [1101.37, 1040.59, 1070.61, 1112.44]
# l_w_mv = [9.13, 7.54, 8.38, 19.66]
# r_coeff = np.median([ (r_wheel[i]-l_w_mv[i])/r_angles[i] for i in range(len(r_wheel)) ])

'''
Top level commands:
generate_route
draw_route
start_route
start_wash
stop_wash
lights_on
lights_off
'''

'''
Drive, wash:
'/fwd/1',
'/fwd/0',
'/lft/1',
'/lft/0',
'/rgt/1',
'/rgt/0',
'/exit/1', 
'/wash/1',
'/wash/0',
'/state/get',

Visual:
'/v_coords/calc'
'/state/get'

Lights:
'/lights/1',
'/ligths/0',
'/left_l_on', 
'/left_l_off',
'/right_l_on',
'/right_l_off',
'/state/get', TODO!

Lidars:
'/l_coords/calc'
'/state/get', TODO!

go 1000 localize go 1000

http://drive/go/1000
http://sensors/state
http://sensors/state
http://sensors/state
http://sensors/state
http://sensors/state
http://localize/do
http://sensors/state
http://sensors/state
http://sensors/state
http://drive/go/1000
http://sensors/state
http://sensors/state
http://sensors/state
http://sensors/state < obstackle!!!!
http://drive/left/30
http://sensors/state
http://sensors/state


Sensors:
'/state/get',
'''

class MiddleWareCollector(Flask):

    def __init__(self, application, debug=0):

        super().__init__(application)
        self.__do_debug = debug
        self.__cmds = []
        self.service_states = {}
        self.__drive_state = 'stop'
        self.__cmd_hist = []
        self.__time_to_continue = time()
        self.environment = {}
        self.inject_cmd = True
        self.cron = crontab.CronTab(user='pi')
        
        # self.dm = DescisionMaker()
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET',])
        self.add_url_rule('/exec/<command>', view_func=self.exec_cmd, methods=['GET',])
        self.add_url_rule('/wash_start', view_func=self.wash_start, methods=['GET',])
        self.add_url_rule('/wash_stop', view_func=self.wash_stop, methods=['GET',])
        self.add_url_rule('/environment', view_func=self.get_environment, methods=['GET'])
        self.add_url_rule('/parking/<state>', view_func=self.parking, methods=['GET',])
        self.add_url_rule('/schedule/wash_types', view_func=self.get_wash_types, methods=['GET',])
        self.add_url_rule('/schedule/add/<minute>/<hour>/<day>/<month>/<day_of_week>/<task>/<repeat>', view_func=self.add_schedule_job, methods=['GET', ])
        self.add_url_rule('/schedule/get', view_func=self.get_schedule, methods=['GET', ])
        self.add_url_rule('/schedule/delete/crontab/<minute>/<hour>/<day>/<month>/<day_of_week>', view_func=self.delete_cron_job, methods=['GET', ])
        self.add_url_rule('/schedule/delete/at/<index>', view_func=self.delete_at_job, methods=['GET', ])


        # self.w_cmds = json.load(open('/home/pi/robot-visual-nav/wash_commands.json'))

    def update_service_states(self):
        for service_name in SERVICES['ports'].keys():
            self.service_states.update({
                service_name: self.__get_service_state(service_name)
            })
        # r = self.get_environment() # update self.environment
        self.service_states.update({
            'environment': self.environment['environment'],
            'sensors': self.environment['sensors'],
            'lidar': self.environment['lidar'],
            'front_lidar': self.environment['front_lidar'],
            'coords': self.environment['coords']
        })


    def get_state(self):
        self.update_service_states()

        return jsonify(self.service_states), 200


    def get_wash_types(self):
        return jsonify(settings.WASH_TYPES), 200


    def get_environment(self):
        sensors_data = get(f'http://localhost:{SERVICES["ports"]["sensors"]}/state/get').json()
        lidar_data = get(f'http://localhost:{SERVICES["ports"]["lidars"]}/state/get').json()
        front_lidar_data = get(f'http://localhost:{SERVICES["ports"]["lidars_x2"]}/state/get').json()
        coords = get(f'http://localhost:{SERVICES["ports"]["drive"]}/state/get').json()
        self.environment.update({'environment': self.__collect_distances(), 'sensors': sensors_data,
                        'lidar': lidar_data,'front_lidar': front_lidar_data,'coords': coords})

        return jsonify({'environment': self.__collect_distances(), 'sensors': sensors_data,
                        'lidar': lidar_data,'front_lidar': front_lidar_data,'coords': coords}), 200


    # def get_points(self):
    #     res = get(f'http://localhost:{SERVICES["ports"]["lidars"]}/state/get').json()
    #
    #     return res

    def wash_start(self):
        # headers = {'content-type': 'application/json'}
        # try:
        #     # commands = request.get_json(force=True)
        #     commands = []
        #     print('[ INFO ] ', request, file=sys.stderr)
        #     assert commands is not None
        # except AssertionError:
        #     print('[ ERROR ] Commands is None!', file=sys.stderr)
        
        # if commands:
        # try:
        res = get(f'http://localhost:{SERVICES["ports"]["drive"]}/wash/1').text
        print(res, file=sys.stderr)
        # except KeyError as e:
        #     return jsonify({'command': 'None', 'sent': 0, 'error': e, 'time': datetime.now()}), 200
        # else:
            # return jsonify({'command': 'None', 'sent': 0, 'error': 'No commands found', 'time': datetime.now()}), 200
        return jsonify({'command': 'wash', 'sent': 1, 'time': datetime.now()}), 200


    def wash_stop(self):
        state = {'result': 'empty'}
        try:
            # 
            state = self.service_states
            res = get(f'http://localhost:{SERVICES["ports"]["drive"]}/wash/0')
        except Exception as e:
            res = get(f'http://localhost:{SERVICES["ports"]["drive"]}/wash/0')
            print(f'[ WARNING ] Not getting result from drv_process. {e}.', file=sys.stderr)

        return jsonify(state), 200


    def set_coords(self, coords):
        return jsonify({}), 200


    def __calc_angle(self, distances: list):
        return 5


    # def ride_by_wall(self, side, to_go, passed):
    #     distance_to_wall = 500
    #     ret = 1
    #     state = {'result': 'empty'}
    #     #res = get(f'http://localhost:{SERVICES["ports"]["lidars"]}/state/get').json()
    #     if side == 'left':
    #         o_cmds = ['right', '', 'left']
    #         distances = res['distances'][2:5]
    #     elif side == 'right':
    #         o_cmds = ['left', '', 'right']
    #         distances = res['distances'][8:11]
    #     else:
    #         print("[ ERROR ] Incorrect side value!", file=sys.stderr)
    #         return '', 0
    #     if distance_to_wall+100 >= distances[1] >= distance_to_wall-100: 
    #         return '', 0
    #     else:
    #         angle = ""
    #         if distances[1] < distance_to_wall:
    #             i = 0
    #             angle = self.__calc_angle(distances)
    #         elif distances[1] > distance_to_wall:
    #             i = 2
    #             angle = self.__calc_angle(distances)
    #         else:
    #             i = 1
    #             ret = 0
    #         ocmd = f'{o_cmds[i]} {angle}'

    #     lines = res['lines']
    #     try:
    #         state = self.service_states
    #     except Exception as e:
    #         print(f'[ WARNING ] Not getting result from drv_process. {e}.', file=sys.stderr)

    #     return ocmd, ret

    def exec_cmd(self, command):
        if command == 'collect_distances':
            return jsonify(self.__collect_distances()), 200
        # elif command == 'start_wash':
        #     return jsonify(self.wash_start()), 200
        # elif command == 'stop_wash':
        #     return jsonify(self.wash_stop()), 200
        elif command == 'take_shots':
            return jsonify(self.take_shots()), 200
        elif command == 'build_path':
            return jsonify({'command': 'build_path'}), 200
        elif command == 'shut_down':
            subprocess.call(['./shutdown.sh'])
            return jsonify({'command': 'shutdown'}), 200
        elif command == 'restart_services':
            open('/tmp/restart', 'w')
            return jsonify({'msg': 'restart init within 1 min'})
        else:
            return jsonify({'error': 'Unknown command!'}), 200


    def parking(self, state):
        print(f'[ Parking INFO ] MW receives state= {state}',file=sys.stderr)
        if state == '1':
            res = get(f'http://localhost:{SERVICES["ports"]["parking"]}/parking/1').json()
        elif int(state) == 0:
            res = get(f'http://localhost:{SERVICES["ports"]["parking"]}/parking/0').json()
        else:
            print("[ ERROR ] Unknown <state>",file=sys.stderr)
            res = {'state': '0'}

        return jsonify(res)


    def take_shots(self):
        res = get(f'http://localhost:{SERVICES["ports"]["visual_nav"]}/v_coords/calc')
        print(res.json())
        
        return res.json()


    def __get_service_state(self, service_name):
        port = SERVICES['ports'][service_name]
        try:
            state = get(f'http://localhost:{port}/state/get').json()
        except exceptions.ConnectionError as e:
            state = {'error': f"Can not get state from `{service_name}`. ConnectionError {e}."}
        except exceptions.JSONDecodeError as e:
            state = {'error': f"Can not get state from `{service_name}`. JSONDecodeError {e}."}

        return state


    # command MUST match with the one in settings.COMMAND_TO_SERVICE
    def __send_cmd(self, command, value=None):
        # low - to drv and other low-level processes
        low_command = self.__cmd_to_drv(command, value)
        service = COMMAND_TO_SERVICE[low_command]['service']
        port = SERVICES['ports'][service]
        try:
            url = COMMAND_TO_SERVICE[low_command]['url'] % (port, value)
        except TypeError:
            url = COMMAND_TO_SERVICE[low_command]['url'] % (port)
        res = get(url)
        
        state = command if res else 'error'

        return state, res


    def __cmd_to_drv(self, cmd, val=None):
        if cmd == 'follow_left' or cmd == 'follow_right':
            cmd = 'go'
        elif cmd == 'pause':
            self.__time_to_continue = time() + val
            cmd = 'stop'

        return cmd


    def __collect_distances(self):
        def to_dst(v):
            dst = 300 if v else 12000

            return dst


        def average_dst(sns_dst, ldr_dst, frn_ldr=None):
            if sns_dst == 12000:
                return ldr_dst
            elif ldr_dst == 12000:
                return sns_dst
            elif frn_ldr == 12000 or frn_ldr == None:
                sns_mult = 1
                ldr_mult = 2
                return (sns_dst*sns_mult + ldr_dst*ldr_mult)/(sns_mult+ldr_mult)
            else:
                sns_mult = 1
                ldr_mult = 2
                frn_ldr_mult = 1
                average = (sns_mult * sns_dst + ldr_mult * ldr_dst + frn_ldr_mult * frn_ldr)/(sns_mult + ldr_mult + frn_ldr_mult)

                return average

        ldr = self.__get_service_state('lidars')['distances']
        frn_ldr = self.__get_service_state('lidars_x2')['distances_x2']
        sns = self.__get_service_state('sensors')
        distances = {
            'frw': {
                'lft':average_dst(to_dst(sns['frw_lft_ik']),ldr[1], np.mean([frn_ldr[0], frn_ldr[1]])),
                'mdl':average_dst(np.mean([to_dst(sns['frw_lft_ik']), to_dst(sns['frw_rgt_ik'])]), ldr[0], np.mean([frn_ldr[2], frn_ldr[3]])),
                'rgt':average_dst(to_dst(sns['frw_rgt_ik']),ldr[11], np.mean([frn_ldr[4], frn_ldr[5]])),
            },
            'lft': {
                'frw':average_dst(to_dst(sns['lft_frw_ik']),ldr[2]),
                'mdl':average_dst(np.mean([to_dst(sns['lft_frw_ik']), to_dst(sns['lft_bkw_ik'])]),ldr[3]),
                'bkw':average_dst(to_dst(sns['lft_bkw_ik']),ldr[4]),
            },
            'rgt': {
                'frw':average_dst(to_dst(sns['rgt_frw_ik']),ldr[10]),
                'mdl':average_dst(np.mean([to_dst(sns['rgt_frw_ik']), to_dst(sns['rgt_bkw_ik'])]),ldr[9]),
                'bkw':average_dst(to_dst(sns['rgt_bkw_ik']),ldr[8]),
            },
            'bkw': {
                'lft':average_dst(to_dst(sns['bkw_lft_ik']),ldr[5]),
                'mdl':average_dst(np.mean([to_dst(sns['bkw_lft_ik']), to_dst(sns['bkw_rgt_ik'])]),ldr[6]),
                'rgt':average_dst(to_dst(sns['bkw_rgt_ik']),ldr[7]),
            },
        }

        return distances


    # def __update_state(self, drv):
    #     to_go = drv['to_go']
    #     passed = drv['passed']
    #     # if drv['drive_state'] == 'stop':
    #     #     break
    #     if self.__system_state == 'go' or self.__system_state == 'back':
    #         if abs(passed) >= to_go:
    #             self.__system_state = 'stop'
    #             self.__send_cmd('stop')
    #     elif self.__system_state == 'left':
    #         if drv['angle_z'] <= drv['z0']:
    #             self.__system_state = 'stop'
    #             self.__send_cmd('stop')
    #     elif self.__system_state == 'right':
    #         if drv['angle_z'] >= drv['z0']:
    #             self.__system_state = 'stop'
    #             self.__send_cmd('stop')

    def check_if_executed(self):
        # print('[ TEST ]', self.__system_state, file=sys.stderr)
        if self.__system_state != 'stop':
            drv = self.service_states['drive']
            if self.__system_state == 'go' or self.__system_state == 'back':
                to_go, passed = drv['to_go'], drv['passed']
                rez = abs(passed) >= to_go
            elif self.__system_state == 'left' or self.__system_state == 'right':
                angle_z, z0 = drv['angle_z'], drv['z0']
                if self.__system_state == 'left':
                    rez = angle_z <= z0
                elif self.__system_state == 'right':
                    rez = angle_z >= z0
            elif self.__system_state == 'pause':
                rez = time() > self.__time_to_continue
                print(rez, file=sys.stderr)
        else:
            rez = 0

        return rez


    def __update_cmd_hist(self, cmd, val):
        if len(self.__cmd_hist) > 10:
            self.__cmd_hist = self.__cmd_hist[1:]
            self.__cmd_hist += [(cmd, val)]
        else:
            self.__cmd_hist += [(cmd, val)]
        print(f'[ INFO ] Cmd history: {self.__cmd_hist}', file=sys.stderr)


    # cmd in PATH MUST match with the one in settings.COMMAND_TO_SERVICE
    def execute_path(self, path):
        n = 0
        # TODO: merge path and PATH !!
        force_stop = False
        for cmd, val in PATH:
            print('[ INFO ] COMMANDS', cmd, val, file=sys.stderr)
            if not force_stop:
                self.__system_state, _ = self.__send_cmd(cmd, val)
                self.__update_cmd_hist(cmd, val)
                while True:
                    self.update_service_states()
                    print('[ INFO ]', self.service_states, self.__system_state, file=sys.stderr)
                    sleep(0.1)
                    if self.check_if_executed():
                        self.__system_state, _ = self.__send_cmd('stop')
                        break
                    if self.inject_cmd == True:
                        for i, ocmd in enumerate([('back', 30), ('back', 100), ('back', 20)], start=1):
                            PATH.insert(n+i, ocmd)
                        self.inject_cmd = False



            #         if drv['drive_state'] == 'stop':
            #             break
            #         elif self.__force_stop_wash:
            #             force_stop = True
            #             self.__send_cmd('stop')
            #             self.__force_stop_wash = False
            #             break
            #         elif cmd == 'follow_left':
            #             ocmd, ret = self.ride_by_wall('left', to_go, passed)
            #             if ret: PATH.insert(n+1, ocmd)
            #             break
            #         elif cmd == 'follow_right':
            #             ocmd, ret = self.ride_by_wall('right', to_go, passed)
            #             if ret: PATH.insert(n+1, ocmd)
            #             break
            #         # TODO: implement check obstacle and bypass cmds
            #         # elif self.dm.check_obstackle(self.service_states['sensors']):
            #         #     # ==> break from loop, because stop cmd
            #         #     self.__send_cmd('stop')
            #         #     for ocmd in self.dm.get_bypass_commands(self.service_states['sensors'], to_go, passed)[::-1]:
            #         #         print(ocmd)
            #         #         PATH.insert(n+1, ocmd)
            #         #     break

            #     # ==> then n + 1, and pass obstackle cmds
            #     print('done')
                n += 1
        # TODO: stop thread by endpoint invoking!!!!!!

    def add_schedule_job(self, minute, hour, day, month, day_of_week, task, repeat):
        print("[ CRONTAB ] start of viewfunc", file=sys.stderr)
        for item in settings.WASH_TYPES:
            if item['id'] == task:
                command = item['command']
        if not command:
            print("[ CRONTAB ] Can\'t load washtypes from settings", file=sys.stderr)
            raise Exception('No such wash type exists')
        print("[ CRONTAB ] washtypes loaded from settings = ",command, file=sys.stderr)
        if repeat == 'never':
            print("[ CRONTAB ] repeat = ", repeat, file=sys.stderr)
            print(f"[ CRONTAB ] calling add_at_job({minute}, {hour}, {day}, {month}, {command})", file=sys.stderr)

            return self.add_at_job(minute, hour, day, month, command)
        else:
            print("[ CRONTAB ] repeat = ", repeat, file=sys.stderr)
            start_wash_string = f'curl -X get http://localhost:{SERVICES["ports"]["middleware"]}/{command}'
            job = self.cron.new(command=start_wash_string, comment=task)
            if repeat == 'daily':
                day, day_of_week = '*', '*'
            elif repeat == 'weekly':
                day = '*'
            elif repeat == 'monthly':
                day_of_week = '*'
            else:
                raise Exception("Can not add cron job")
            schedule_str = f'{minute} {hour} {day} * {day_of_week}'
            print("[ CRONTAB ] schedule_str = ",schedule_str, file=sys.stderr)
            job.setall(schedule_str)
            self.cron.write()

        return jsonify({"msg": f'{repeat} {command} has been planned on {schedule_str}'}), 200


    def add_at_job(self, minute, hour, day, month, command):
        print("[ CRONTAB ] start of add_at_job", file=sys.stderr)
        print(f"[ CRONTAB ] args: {minute}, {hour}, {day}, {month}, {command}", file=sys.stderr)
        months = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
        str_month = months[int(month) - 1]
        if len(minute) < 2: minute = "0" + minute
        print(f"[ CRONTAB ] minute: {minute}", file=sys.stderr)
        string = f"echo 'curl -X get http://localhost:{SERVICES['ports']['middleware']}/{command}' | at {hour}:{minute} {str_month} {day}"
        subprocess.run(string, shell=True)

        return jsonify({"msg": f'{command} has been planned on {hour}-{minute} {day}.{month}'}), 200


    def get_schedule(self):
        return list(self.get_at_jobs()[0] + self.get_cron_jobs()[0]), 200



    def get_at_jobs(self):
        months = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
        days_of_week = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"]
        result_atq = subprocess.run('atq', stdout=subprocess.PIPE, text=True)  # universal_newlines=True
        results = result_atq.stdout.split('\n')[:-1]
        at_jobs = []
        for i in results:
            s_job = i.split(' ')  # ['8\tThu', 'Jul', '13', '21:20:00', '2023', 'a', 'pi\n']
            index = s_job[0].split('\t')[0]
            result_at_c = subprocess.run(['at', '-c', index], stdout=subprocess.PIPE, text=True)
            at_command = result_at_c.stdout.split('/')[-1].strip()
            for i in settings.WASH_TYPES:
                wash_type = i['id'] if i['command'] == at_command else None
            at_jobs.append({"minute": s_job[3].split(':')[1],
                            "hour": s_job[3].split(':')[0],
                            "day": s_job[2],
                            "month": str(months.index(s_job[1]) + 1),
                            "day_of_week": str(days_of_week.index(s_job[0].split('\t')[1])),
                            "wash_type": wash_type,
                            "index": index})

        return at_jobs, 200

    def get_cron_jobs(self):
        cron_jobs = []
        for job in self.cron:
            s_job = str(job).split(' ')
            if s_job[0] == '@monthly':
                cron_jobs.append({"minute": "0",
                                  "hour": "0",
                                  "day": "1",
                                  "month": "*",
                                  "day_of_week": "*",
                                  "wash_type": s_job[-1]})
            elif s_job[0] == '@weekly':
                cron_jobs.append({"minute": "0",
                                  "hour": "0",
                                  "day": "*",
                                  "month": "*",
                                  "day_of_week": "0",
                                  "wash_type": s_job[-1]})
            elif s_job[0] == '@daily':
                cron_jobs.append({"minute": "0",
                                  "hour": "0",
                                  "day": "*",
                                  "month": "*",
                                  "day_of_week": "*",
                                  "wash_type": s_job[-1]})
            else:
                cron_jobs.append({"minute": s_job[0],
                                  "hour": s_job[1],
                                  "day": s_job[2],
                                  "month": s_job[3],
                                  "day_of_week": s_job[4],
                                  "wash_type": s_job[-1]})
        # print(cron_jobs)
        return cron_jobs, 200

    def delete_cron_job(self, minute, hour, day, month, day_of_week):
        # Find an existing job by schedule:
        # iter = self.cron.find_time("*/2 * * * *")
        job_to_delete = self.cron.find_time(f"{minute} {hour} {day} {month} {day_of_week}")
        self.cron.remove(job_to_delete)
        self.cron.write()

        return jsonify({'msg': 'job deleted successfully'}), 200

    def delete_at_job(self, index):
        result_atq = subprocess.run(['atrm', index], stdout=subprocess.PIPE, text=True)  # universal_newlines=True

        return jsonify({'msg': 'job deleted successfully'}), 200



    def __del__(self):
        pass



if __name__ == '__main__':
    
    mc_app = MiddleWareCollector(__name__)
    mc_app.config['JSON_AS_ASCII'] = False
    mc_app.run(host='0.0.0.0', port=4994, debug=False)
