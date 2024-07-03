import os, sys
from flask import Blueprint, render_template, request, redirect, flash, url_for, jsonify
from flask_login import login_required, current_user
from .models import User
from flask import current_app
from werkzeug.utils import secure_filename
import requests
import json

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from settings import SERVICES

api = Blueprint('api', __name__)

api_config = {
    'actions': [
        {'url': 'lights', 'type': 'btn'},
        {'url': 'wash', 'type': 'form'},
        {'url': 'backlights', 'type': 'btn'},
        {'url': 'fwdlights', 'type': 'btn'},
        {'url': 'fwd', 'type': 'btn'},
        {'url': 'lft', 'type': 'btn'},
        {'url': 'rgt', 'type': 'btn'},
        {'url': 'bck', 'type': 'btn'},

    ],
}

@api.route("/lights/<state>", methods=['POST', ])
@login_required
def lights(state):
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["lights"]}/lights/{state}')
        data = {
            'echo': True, 
            'action': 'lights', 
            'state': {
                'lights': state,
            }
        }

        return jsonify(data), 200
    else:
        return jsonify(message="Method Not Allowed"), 405


@api.route("/wash/<state>", methods=['POST', ])
@login_required
def wash(state): # 0 by default, to avoid issues with 'stop' command
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        # resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/wash/{state}, file=sys.stderr')
        commands = json.dumps({'path': [['go',100],['left',10]]})
        if state == '1':
            # headers = {'Content-type': 'application/json'}
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/wash_start')
            print('[ INFO ] WASH', file=sys.stderr)
        else:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/wash_stop')
            print('[ INFO ] STOP', file=sys.stderr) # force stop
        data = {
            'echo': True, 
            'action': 'wash', 
            'state': {
                'wash': state,
            }
        }
        return jsonify(data), 200
    else:
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/wash_stop/')
        print('[ INFO ] STOP', file=sys.stderr) # force stop
        return jsonify(message="Method Not Allowed"), 405


@api.route("/backlights", methods=['POST', ])
@login_required
def backlights():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        data = {'echo': True, 'action': 'backlights', 'state': {'lights': 1, 'fwd': 1,}}

        return jsonify(data), 200
    else:
        return jsonify(message="Method Not Allowed"), 405


@api.route("/fwdlights", methods=['POST', ])
@login_required
def fwdlights():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        data = {'echo': True, 'action': 'fwdlights', 'state': {'lights': 1, 'fwd': 1,}}

        return jsonify(data), 200
    else:
        return jsonify(message="Method Not Allowed"), 405


@api.route("/fwd/<state>", methods=['POST', ])
@login_required
def fwd(state): # 0 by default, to avoid issues with 'stop' command
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        distance = 1000
        if state == '1':
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/fwd/{state}/{distance}')
            print('[ INFO ]  FWD', file=sys.stderr)
        else:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/fwd/0')
            print('[ INFO ]  STOP', file=sys.stderr) # force stop
        data = {
            'echo': True, 
            'action': 'fwd', 
            'state': {
                'fwd': state,
            }
        }
        return jsonify(data), 200
    else:
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/fwd/0')
        print('[ INFO ]  STOP', file=sys.stderr) # force stop
        return jsonify(message="Method Not Allowed"), 405


@api.route("/lft/<state>", methods=['POST', ])
@login_required
def lft(state):
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        angle = 90
        if state == '1':
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/lft/{state}/{angle}')
            print('[ INFO ]  LFT', file=sys.stderr)
        else:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/lft/0/1')
            print('[ INFO ]  STOP', file=sys.stderr) # force stop
        
        data = {
            'echo': True, 
            'action': 'lft', 
            'state': {
                'lft': state,
            }
        }
        return jsonify(data), 200
    else:
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/lft/0/1')
        print('[ INFO ]  STOP', file=sys.stderr) # force stop
        return jsonify(message="Method Not Allowed"), 405


@api.route("/rgt/<state>", methods=['POST', ])
@login_required
def rgt(state, angle=0):
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        angle = 90
        if state == '1':
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/rgt/{state}/{angle}')
            print('[ INFO ]  RGT', file=sys.stderr)
        else:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/rgt/0/1')
            print('[ INFO ]  STOP', file=sys.stderr) # force stop

        data = {
            'echo': True, 
            'action': 'rgt', 
            'state': {
                'rgt': state,
            }
        }
        return jsonify(data), 200
    else:
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/rgt/0/1')
        print('[ INFO ]  STOP', file=sys.stderr) # force stop
        return jsonify(message="Method Not Allowed"), 405


@api.route("/bck/<state>", methods=['POST', ])
@login_required
def bck(state):
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        distance = 1000
        if state == '1':
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/bck/{state}/{distance}')
            print('[ INFO ]  bck', file=sys.stderr)
        else:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/bck/0')
            print('[ INFO ] STOP', file=sys.stderr) # force stop
        data = {
            'echo': True, 
            'action': 'bck', 
            'state': {
                'bck': state,
            }
        }
        return jsonify(data), 200
    else:
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/bck/0')
        print('[ INFO ] STOP', file=sys.stderr) # force stop
        return jsonify(message="Method Not Allowed"), 405


@api.route("/path/build", methods=['POST', ])
@login_required
def build_path():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/exec/build_path')
        print('[ INFO ] building path', file=sys.stderr)
        return jsonify({}), 200
    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405

@api.route("/state/env/get", methods=['POST', ])
@login_required
def get_environment():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/environment').json()
        return jsonify(resp), 200
        print('[ INFO ] getting environment data', file=sys.stderr)

    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405


@api.route("/system/state/get", methods=['GET', ])
# @login_required !!!!!!!!!!!!!!!!!!!!!!!
def get_state():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/state/get').json()
        return jsonify(resp), 200
        print('[ INFO ] getting environment data', file=sys.stderr)

    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405


@api.route("/parking/<state>", methods=['POST', ])
@login_required
def parking(state):
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/parking/{state}')
        try:
            state = resp.json()["state"]
        except:
            state = 0
            print(f'[ ERROR ] can\'t read response', file=sys.stderr)
        data = {
            'echo': True,
            'action': 'parking',
            'state': {
                'parking': state,
            }
        }
        return jsonify(data), 200
    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405

@api.route("/schedule/add/<minute>/<hour>/<day>/<month>/<weekday>/<task>/<repeat>", methods=['POST', ])
@login_required
def cron_schedule(minute, hour, day, month, weekday, task, repeat):
    print('[ INFO CRONTAB] api.py received GET request', file=sys.stderr)
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/schedule/add/{minute}/{hour}/{day}/{month}/{weekday}/{task}/{repeat}').json()
        print(f'[ INFO CRONTAB] api.py sent GET to middleware: /schedule/add/{minute}/{hour}/{day}/{month}/{weekday}/{task}/{repeat}', file=sys.stderr)
        return jsonify(resp), 200
    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405

@api.route("/schedule/delete/crontab/<minute>/<hour>/<day>/<month>/<day_of_week>", methods=['POST', ])
@login_required
def cron_delete(minute, hour, day, month, day_of_week):
    print('[ INFO CRONTAB] api.py received GET request', file=sys.stderr)
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/schedule/delete/crontab/{minute}/{hour}/{day}/{month}/{day_of_week}').json()
        print(f'[ INFO CRONTAB] api.py sent GET to middleware: /schedule/delete/{minute}/{hour}/{day}/{month}/{day_of_week}', file=sys.stderr)
        print("[ INFO CRONTAB] resp = ", resp)
        return jsonify(resp), 200
    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405

@api.route("/schedule/delete/at/<index>", methods=['POST', ])
@login_required
def at_delete(index):
    print('[ INFO CRONTAB] api.py received GET request', file=sys.stderr)
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/schedule/delete/at/{index}').json()
        print(f'[ INFO CRONTAB] api.py sent GET to middleware: /schedule/delete/at/{index}', file=sys.stderr)
        return jsonify(resp), 200
    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405

@api.route("/schedule/wash_types", methods=['POST', ])
@login_required
def get_wash_types():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/schedule/wash_types').json()
        print("resp= ", resp, file=sys.stderr)
        return jsonify(resp), 200

    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405

@api.route("/schedule/get", methods=['POST', ])
@login_required
def get_schedule():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/schedule/get').json()
        print("resp= ", resp, file=sys.stderr)
        return jsonify(resp), 200

    else:
        print('[ ERROR ] 405', file=sys.stderr)  # no path
        return jsonify(message="Method Not Allowed"), 405