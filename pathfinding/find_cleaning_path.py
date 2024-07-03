import re
import subprocess
import sys

import cv2 as cv 
import numpy as np

from math import sin, cos, radians, degrees
from sympy import Line, Point, Circle, Ray

stroke = "blue"
style = ""
width = "0.2"
R = 200
ROBOT_RADIUS = 200
SCALE_COEFF = 1000

START_POS_CODE = 204
OBSTACLE_CODE = 0
PARKING_CODE = 218

MAP = 'map.png'

def xml_map_2_stl(xml_map):
    return 'g.stl'


def run_parser(stl_map_file):
    try:
        x = subprocess.run(["slic3r", "--load", "config.ini", stl_map_file])
        assert x.returncode == 0, ', '.join(["slic3r", "--scale", "20", "--load", "config.ini", stl_map_file])
        res = stl_map_file.replace(".stl",".gcode")
    except (FileNotFoundError, AssertionError) as e:
        print('error', e, file=sys.stderr)
        exit(1)
    return res


def extract_moves(gcode_map_file):
    x = [ x.strip() for x in open(gcode_map_file).readlines() ]
    y = [ x for x in x if x.startswith('G1 X') ]
    coordinates = []
    for move in y:
        p = [ float(x) for x in re.findall(r'.*?X(.*?)\sY(.*?) ', move)[0] ]
        coordinates += [p]

    coordinates_array = np.array(coordinates)
    delta = coordinates_array.min(axis=0)
    size = (coordinates_array - delta).max(axis=0)

    return coordinates_array - delta, size


def gcode_2_svg(gcode_map_file, start_pos):
    coordinates_array, size = extract_moves(gcode_map_file)
    p0 = np.array(start_pos)

    svg = f'''<svg version="1.1"
     baseProfile="full"
     width="{size[0]}" height="{size[1]}"
     xmlns="http://www.w3.org/2000/svg">
'''
    for p1 in coordinates_array:
        svg += f'''<line x1="{p0[0]}" y1="{p0[1]}" 
                        x2="{p1[0]}" y2="{p1[1]}"  
                        stroke="{stroke}" fill="transparent" 
                        style="{style}" stroke-width="{width}" />
'''
        p0 = p1

    svg += '''</svg>'''

    return svg

def get_coords_by_distance(distance, start_pos=(0,0), angle=0):
    coords = start_pos
    a_angle = 90 - angle
    r = Ray(Point(start_pos), angle=radians(a_angle))
    c = Circle(Point(start_pos), distance)
    stop_pos = r.intersection(c)
    coords = (float(stop_pos[0].x), float(stop_pos[0].y))

    return coords, angle

def get_turn_coords(angle, side, start_pos=(0,0), start_angle=0):
    coords = start_pos
    if side == 'right':
        a_angle = start_angle + 180 - angle
        c_center = (ROBOT_RADIUS, start_pos[1])
    else:
        c_center = (-ROBOT_RADIUS, start_pos[1])

    c = Circle(Point(c_center), ROBOT_RADIUS)
    r = Ray(Point(c_center), angle=radians(30))
    stop_pos = c.intersection(r)
    coords = (float(stop_pos[0].x), float(stop_pos[0].y))

    return coords, start_angle + angle


def cmd_to_svg(cmds, start_pos=(0,0), start_angle=0):

    # cmds = gcode_2_cmd(gcode_map_file, start_pos)

    cmds = [
        ('right', 60),
        ('go', 100),
        ('right', 60),
        ('go', 100),
        ('right', 60),
        ('go', 100),
        ('right', 60),
        ('go', 100),
        ('right', 60),
        ('go', 100),
        ('right', 60),
        ('go', 100),
        # ('left', 60),
        # ('go', 1000),
        # ('left', 90),
        # ('go', 1000),
        # ('left', 90),
        # ('go', 1000),
        # ('left', 60),
        # ('go', 1000),
        # ('left', 60),
        # ('go', 1000),
        # ('left', 60),
        # ('go', 1000),
    ]
    size = (10000, 10000)
    # svg = f'''<svg version="1.1"
    #  baseProfile="full"
    #  width="{size[0]}" height="{size[1]}"
    #  xmlns="http://www.w3.org/2000/svg">
    # '''
    svg = ''
    coords, angle = start_pos, start_angle
    for c in cmds:
        action, value = c[0], int(c[1])
        if action == 'go':
            coords, angle = get_coords_by_distance(distance=value, start_pos=coords, angle=angle)
            # print(f'Line from ({last_coordinates}) to ({coords}).')
            # line = f'''<line x1="{last_coordinates[0]+SCALE_COEFF}" y1="{last_coordinates[1]+SCALE_COEFF}" 
            #             x2="{coords[0]+SCALE_COEFF}" y2="{coords[1]+SCALE_COEFF}"  
            #             stroke="black" fill="transparent" />'''
            # svg += line
        else:
            coords, angle = get_turn_coords(angle=value, side=action, start_pos=coords, start_angle=angle)
            # print(f'Turn {action} from ({last_coordinates}) to ({coords}).')
            # curve_cp = (last_coordinates[0]+int((coords[0]-last_coordinates[0]) / 2),
            #         last_coordinates[1]+int((coords[1]-last_coordinates[1]) / 2))
            # line = f'''<path d="M {last_coordinates[0]+SCALE_COEFF},{last_coordinates[1]+SCALE_COEFF} 
            # Q {curve_cp[0]+SCALE_COEFF},{curve_cp[1]+SCALE_COEFF} 
            # {coords[0]+SCALE_COEFF},{coords[1]+SCALE_COEFF}" 
            # stroke="black" fill="transparent" />'''
            # svg += line
        print(coords, angle)

        # last_coordinates = coords


    # svg += '''</svg>'''

    # print(svg)

    return svg


def gcode_2_cmd(gcode_map_file, start_pos):
    coordinates_array, size = extract_moves(gcode_map_file)
    print(coordinates_array)
    r = R
    x0, y0 = start_pos
    z0, z1 = 0, 0

    rez = []

    A = Point(x0, y0)
    B = Point(x0 + sin(radians(z0)), y0 + cos(radians(z0)))
    ab = Line(A, B)
    pp = ab.perpendicular_line(A)
    me = Circle(A, R)

    for p in coordinates_array[1:]:
        x1, y1 = (p * 1000).astype(int)
        C = Point(x1, y1)
        if A.distance(C) > R:
            D, E = [ x for x in pp.intersection(me) ]
            l, r = D.distance(C), E.distance(C)
            f, b = B.distance(C), A.distance(C)

            if abs(l - r) > 0.1 or f > b :
                # F = sorted([D, E], key=lambda x: x.distance(C))[0]
                k = 'left' if l < r else 'right'
                F = D if l < r else E
                tc = Circle(F, R)
                commands = []
                for tl in tc.tangent_lines(C):
                    p = tl.p2
                    l = Line(p, F)
                    if k == 'right':
                        z1 = round(degrees(l.angle_between(pp)))
                    else:
                        z1 = 180 - round(degrees(l.angle_between(pp)))
                    commands += [(f'{k} {z1}', f'go {round(p.distance(C) / 2)}')]
                commands = commands[0] if k == 'right' else commands[-1]
                rez += [" ".join(commands)]
            else:
                rez += [f'go {round(A.distance(C) / 2)}']
        else:
            rez += []

        x0, y0 = x1, y1
        z0 = z1
        A = Point(x0, y0)
        B = Point(x0 + sin(radians(z0)), y0 + cos(radians(z0)))
        ab = Line(A, B)
        pp = ab.perpendicular_line(A)
        me = Circle(A, R)

    return rez

def _find_cleaning_path(xml_map, start_pos=(0,0)):
    stl_map_file = xml_map_2_stl(xml_map)
    gcode_map_file = run_parser(stl_map_file)
    svg_map = gcode_2_svg(gcode_map_file, start_pos)
    commands = gcode_2_cmd(gcode_map_file, start_pos)
    print(commands)
    return svg_map

# def create_cleaning_map():
#     svg = open('../web/media/uploads/test_map.svg', 'r').read()
#     sp = BS(svg, 'html.parser')
    
#     svg_new_lines = cmd_to_svg([])
    
#     original_tag = sp.svg
#     sp_new = BS(svg_new_lines, 'html.parser')
#     # original_tag.append("<g></g>")
#     group = original_tag.find_all('g')[-1]
#     [group.append(tag) for tag in sp_new.find_all()]
#     # print(sp_new)
#     original_tag.append(group)
#     print(original_tag)
#     output_svg = open('../web/media/uploads/test_map.1.svg', 'w').write(str(original_tag))

#     # return output_svg

def find_cleaning_path(map_file_name, robot_radius, start_pos_code, obstacle_code, parking_code):
    room_map = cv.imread(map_file_name, 0)


#print(image.shape)
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # apply thresholding to remove any noise in the image
        ret, thresh = cv2.threshold(image, 127, 255, 0)

        # find contours in the image and initialize the mask that will be used to remove the bad contours
        im2, contours, hierarchy = cv2.findContours(thresh, 1 , 2)

        # loop over all of the contours found in the image and only keep those that are large enough
        for i in range (0 , len (contours)) : 

            if cv2.contourArea (contours[i]) > 5000 : 

                rect = cv2.boundingRect (contours[i]) 



    return ''


if __name__ == '__main__':
    # gcode_2_svg('c.gcode')
    # xml_map = open('test.svg').read()
    # svg = find_cleaning_path('xml_map', start_pos=(300,300))
    # open('/tmp/test.svg','w').write(svg)
    # create_cleaning_map()
    # cmd_to_svg(None)
    path = find_cleaning_path(MAP, ROBOT_RADIUS, START_POS_CODE, OBSTACLE_CODE, PARKING_CODE)
    print(path)

