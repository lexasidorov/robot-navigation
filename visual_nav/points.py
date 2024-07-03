from svg_pnt_actions import points_to_svg
import json

points = json.load(open('visual_nav/visual_metadata/points.json'))

if __name__ == '__main__':
    print(points_to_svg(points))

    