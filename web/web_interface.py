import os, sys, requests
from flask import Blueprint, render_template, request, redirect, flash, url_for, jsonify
from flask_login import login_required, current_user
from .models import User
from flask import current_app
from werkzeug.utils import secure_filename
from .util import allowed_file, DEFAULT_MAP_FILENAME
from queue import Empty

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from settings import SERVICES

web_interface = Blueprint('web_interface', __name__)

@web_interface.route("/", methods=['GET', ])
@login_required
def index():
    params = {'whoami': current_user.name, 'actions': [
        {'title': 'Прямое управление', 'desc': 'Управление посредством кнопок вперёд/назад/вправо/влево.', 'link': url_for('web_interface.direct')},
        {'title': 'Обработка карт', 'desc': 'Загрузка и просмотр карты помещения.', 'link': url_for('web_interface.maps')},
        {'title': 'Контроль параметров', 'desc': 'Отображение параметров системы.', 'link': url_for('web_interface.system')}
    ]}

    return render_template('index.html', params=params)

@web_interface.route("/maps", methods=['GET', 'POST'])
@login_required
def maps():
    params = {
        'whoami': current_user.name, 
    }
    if request.method == 'POST':
        if 'svg_map' not in request.files:
            flash('No file part')
            return redirect(request.url)
        file = request.files['svg_map']
        # if user does not select file, browser also
        # submit an empty part without filename
        if file.filename == '':
            flash('No selected file')
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            print(filename, file=sys.stderr)
            full_filename = os.path.join(current_app.config['UPLOAD_FOLDER'], DEFAULT_MAP_FILENAME)
            print(full_filename, file=sys.stderr)
            try:
                file.save(f'web/{full_filename}')
                svg = open(f'web/{full_filename}').read()
                params['svg'] = svg
            except OSError as e:
                print(e, file=sys.stderr)
                flash('Error saving file, check if path web/media/uploads exists!')
            except FileNotFoundError as e:
                print(e, file=sys.stderr)
                flash('Error reading file')
            except Exception as e:
                print(e, file=sys.stderr)
                flash(f'Error {e}')

            # return redirect(url_for('web_interface.maps',
            #                         filename=filename))
            
            return render_template('maps.html', params=params)
    else:
        full_filename = os.path.join(current_app.config['UPLOAD_FOLDER'], DEFAULT_MAP_FILENAME)
        if os.path.isfile(f'web/{full_filename}'):
            svg = open(f'web/{full_filename}').read()
            params['svg'] = svg

        return render_template('maps.html', params=params)


@web_interface.route("/system", methods=['GET', ])
@login_required
def system():
    params = {
        'whoami': current_user.name, 
    }

    return render_template('system.html', params=params)

@web_interface.route("/state/get", methods=['GET', ])
@login_required
def get_state_drv():
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        try:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["drive"]}/state/get')
            j = resp.json()
            data = {
                'echo': True, 
                'action': 'state', 
                'state': j
            }
        except requests.exceptions.ConnectionError as e:
            data = {'error': f"Can not get state from `{service_name}`. ConnectionError {e}."}
        return jsonify(data), 200
    else:
        return jsonify(message="Method Not Allowed"), 405

@web_interface.route("/direct", methods=['GET', ])
@login_required
def direct():
    params = {
        'whoami': current_user.name, 
    }

    return render_template('direct.html', params=params)

@web_interface.route("/coords/get", methods=['POST', ])
@login_required
def robot_get_coords(last_coordinates=(0,0)):
    if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
        last_coordinates = (0,0)
        # current_app.coords_request.put({
        #     'action': 'get_coords',
        #     'last_coords': last_coordinates
        # })
        try:
            resp = requests.get(f'http://127.0.0.1:{SERVICES["ports"]["middleware"]}/state/get')
            data = resp.json()
        except requests.exceptions.ConnectionError as e:
            data = {'error': f"Can not get state from `{service_name}`. ConnectionError {e}."}
        # coords_xy = (int(coords[0]/100), int(coords[1]/100))
        return jsonify(data), 200
    else:
        return jsonify(message="Method Not Allowed"), 405

# @web_interface.route("/coords/get", methods=['POST', ])
# @login_required
# def robot_get_coords(last_coordinates=(0,0)):
#     if request.headers['X_REQUESTED_WITH'] == 'XMLHttpRequest':
#         coordinates = (0,0)
#         try:
#             cur_coords = current_app.coords_response.get()
#             coordinates = (int(coordinates[0]), int(coordinates[1]))
#         except Empty as e:
#             print('[ WARNING ] No coordinates!')
#             return jsonify([]), 200

#         return jsonify(coordinates), 200
#     else:
#         return jsonify(message="Method Not Allowed"), 405


