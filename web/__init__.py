import os
from getpass import getpass
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import exc as sql_exc
from flask_login import LoginManager
from werkzeug.security import generate_password_hash
from multiprocessing import Process, Queue

# init SQLAlchemy so we can use it later in our models
db = SQLAlchemy()
init_db = os.getenv('INIT_FLASK_DB_USER')
UPLOAD_FOLDER = 'media/uploads'


class FlaskProcess(Flask):

    def __init__(self, application, *args, **kwargs):
        super().__init__(application)
        self.coords_request = Queue()
        self.coords_response = Queue()

    def __del__(self):
        pass



def get_coords():
    coords = ()
    open('coords_list.txt', 'a').write(coords)

def create_app():
    app = FlaskProcess(__name__)

    # test_move_proc = Process(target=get_coords)
    # pid = test_move_proc.start()
    # print(pid)

    app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
    app.config['SECRET_KEY'] = 'secret-key-goes-here'
    app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///db.sqlite'

    db.init_app(app)
    print(init_db)
    # blueprint for auth routes in our app
    from .auth import auth as auth_blueprint
    app.register_blueprint(auth_blueprint)

    # blueprint for non-auth parts of app
    from .web_interface import web_interface as wi_blueprint
    app.register_blueprint(wi_blueprint)

    # blueprint for non-auth parts of app
    from .api import api as api_blueprint
    app.register_blueprint(api_blueprint)

    login_manager = LoginManager()
    login_manager.login_view = 'auth.login'
    login_manager.init_app(app)

    from .models import User

    def recreate_db(app):
        with app.app_context():
            
            db.drop_all()
            print('[ INFO ] Dropped db.')
            db.create_all()
            print('[ INFO ] Created db.')

    def create_new_user(app,  email, name, password, admin=False):
        with app.app_context():
            
            try:
                new_user = User(email=email, name=name, password=generate_password_hash(password, method='sha256'), admin=admin)
                # add the new user to the database
                db.session.add(new_user)
                db.session.commit()
                print(f'[ INFO ] Created new user with email {email} and name {name}.')
            except sql_exc.IntegrityError:
                print('[ ERROR ] User already exists')

        return new_user
    
    if init_db == '1':
        recreate_db(app=app)
        passwd = getpass()
        create_new_user(app=app, email='admin@asd.fg', name='Alex', password=passwd, admin=True)
        os.environ['INIT_FLASK_DB_USER'] = '0'

    @login_manager.user_loader
    def load_user(user_id):
        # since the user_id is just the primary key of our user table, use it in the query for the user
        return User.query.get(int(user_id))

    return app

