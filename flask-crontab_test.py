from flask_crontab import Crontab
from flask import Flask
import subprocess
# from crontab import CronTab
import crontab

SERVICES = {'ports': {'middleware': 4990}}
# WASH_TYPES = {"dry": "wash_start",
#               "wet": "wash_start"}
WASH_TYPES = [
    {'id': 'dry', 'name': 'Сухая уборка', 'command': 'wash_start'},
    {'id': 'wet', 'name': 'Влажная уборка', 'command': 'wash_start'}
]


class TestFlask(Flask):
    def __init__(self, app, debug=False):
        super().__init__(app)
        self.debug = debug
        self.add_url_rule('/crontab/<minute>/<hour>/<day>/<month>/<day_of_week>/<task>/<repeat>', view_func=self.schedule_job, methods=['GET',])
        self.add_url_rule('/schedule/get', view_func=self.get_schedule, methods=['GET',])
        self.add_url_rule('/cron_job/delete/<minute>/<hour>/<day>/<month>/<day_of_week>', view_func=self.delete_cron_job, methods=['GET',])
        self.add_url_rule('/at_job/delete/<index>', view_func=self.delete_at_job, methods=['GET',])
        self.cron = crontab.CronTab(user='pi')


    def schedule_job(self, minute , hour, day, month, day_of_week, task, repeat):
        for item in WASH_TYPES:
            if item['id'] == task:
                command = item['command']

        if repeat == 'never':
            return self.planed_job(minute , hour, day, month, command)
        else:
            start_wash_string = f'curl -X get http://localhost:{SERVICES["ports"]["middleware"]}/{command}'
            job = self.cron.new(command=start_wash_string, comment = task)
            if repeat == 'daily':
                day, day_of_week ='*', '*'
            elif repeat == 'weekly':
                day =  '*'
            elif repeat == 'monthly':
                day_of_week = '*'
            else:
                raise Exception("Can not add cron job")
            schedule_str = f'{minute} {hour} {day} * {day_of_week}'
            job.setall(schedule_str)
            self.cron.write()
        # TODO: no time attributes for cronjob is being added now. Cron job is being added successfully though
        # TODO: test how job.setall() works
        # subprocess.call(['crontab', minute , hour, day, '*', day_of_week, start_wash_string])
        return f'{repeat} {command} has been planned on {schedule_str}', 200




    def planed_job(self, minute , hour, day, month, command):
        months = ["Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"]
        # days_of_week = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"]
        # str_day = days_of_week[day]
        str_month = months[int(month) -1]
        if len(minute) < 2: minute = "0" + minute
        string = f"echo 'curl -X get http://localhost:{SERVICES['ports']['middleware']}/{command}' | at {hour}:{minute} {str_month} {day}"
        subprocess.run(string,shell=True)
        return f'{command} has been planned on {hour}:{minute} {day}.{str_month}', 200

    def get_schedule(self):
        return list(self.get_at_jobs()[0] + self.get_cron_jobs()[0]), 200

    def get_at_jobs(self):
        months = ["Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"]
        days_of_week = ["Sun","Mon","Tue","Wed","Thu","Fri","Sat"]
        result_atq = subprocess.run('atq', stdout=subprocess.PIPE, text=True)  # universal_newlines=True
        # print(result_atq.stdout)
        results = result_atq.stdout.split('\n')[:-1]
        # print("RESULTS LENGTH", len(results))
        at_jobs =[]
        for i in results:
            s_job = i.split(' ') # ['8\tThu', 'Jul', '13', '21:20:00', '2023', 'a', 'pi\n']
            index = s_job[0].split('\t')[0]
            result_at_c = subprocess.run(['at', '-c', index], stdout=subprocess.PIPE, text=True)
            at_command = result_at_c.stdout.split('/')[-1].strip()
            # at_command = result_at_c.stdout.split('\n')[-1].split('/')[-1].split('\n')[0]
            # wash_type = list(WASH_TYPES.keys())[list(WASH_TYPES.values()).index(at_command)]
            for i in WASH_TYPES:
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
        # result = subprocess.run(['crontab','-l'], stdout=subprocess.PIPE, text=True)  # universal_newlines=True
        cron_jobs = []
        for job in self.cron:
            s_job = str(job).split(' ') # s_job = ['40', '20', '1', '*', '*', 'curl', '-X', 'get', 'http://localhost:4990/wash_start', '#', 'scheduled', 'wash']
            # print(s_job) # ['@monthly', 'curl', '-X', 'get', 'http://localhost:4990/wash_start', '#', 'wash_start']
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
                cron_jobs.append({"minute":s_job[0],
                                  "hour": s_job[1],
                                  "day": s_job[2],
                                  "month": s_job[3],
                                  "day_of_week": s_job[4],
                                  "wash_type": s_job[-1]})
        # print(cron_jobs)
        return cron_jobs, 200

    def delete_cron_job(self, minute, hour, day, month, day_of_month):
        # Find an existing job by schedule:
        # iter = self.cron.find_time("*/2 * * * *")
        job_to_delete = self.cron.find_time(f"{minute} {hour} {day} {month} {day_of_month}")
        self.cron.remove(job_to_delete)
        return '/schedule/refresh', 200

    def delete_at_job(self,index):
        result_atq = subprocess.run(['atrm', index], stdout=subprocess.PIPE, text=True)  # universal_newlines=True
        return '/schedule/refresh', 200



if __name__ == '__main__':

    app = TestFlask(__name__, debug = True)
    app.run(host='0.0.0.0', port=4990, debug=False)