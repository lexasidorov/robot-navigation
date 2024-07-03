from flask import Flask, current_app, jsonify
from visual_navigation import VisualNav


class VisualCoords(Flask):

    def __init__(self, application, *args, **kwargs):
        super().__init__(application)
        self.add_url_rule('/v_coords/calc', view_func=self.get_coords, methods=['GET',])
        self.add_url_rule('/state/get', view_func=self.get_state, methods=['GET',])
        self.vn = VisualNav()
        # self.__create_visual_service()

    def __create_visual_service(self):
        self.q_cmd, self.q_result = Queue(), Queue()
        self.visual_proc = VisualNav(self.q_cmd, self.q_result)
        self.visual_proc.start()

    def get_coords(self):
        
        coords = self.vn.calculate_my_coords()
        return jsonify(coords), 200

    def get_state(self):

        return jsonify({'state': 'aaaa'})

    def __del__(self):
        pass


if __name__ == '__main__':

    visual_coords_app = VisualCoords(__name__)
    visual_coords_app.run(host='0.0.0.0', port=4997)

    visual_coords_app.stop()