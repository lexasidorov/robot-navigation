import sys

import numpy as np
import cv2 as cv

ROBOT_RADIUS = 200
SCALE_COEFF = 1000

START_POS_CODE = 204
OBSTACLE_CODE = 0
PARKING_CODE = 218


def get_map(mapname='map.a.png'):
    im = cv.imread(mapname, 0)
    assert im is not None, '[ ERROR ] file "{f}" not found!'
    return im


def find_start_pos(a_map, start_pos_code=START_POS_CODE):
    y, x = [ (y[0], x[0]) for y, x in [np.where(a_map == start_pos_code)] ][0]
    return x, y


def find_contour_path(a_map, 
                      start_pos=(0,0), 
                      robot_radius=ROBOT_RADIUS, 
                      obstacle_code=OBSTACLE_CODE):
    r = round(robot_radius / 300)
    ccontours = []
    for i in range(20):
        ret, thresh = cv.threshold(a_map, 127, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(a_map, contours, -1, (0,0,0), r)
        ccontours += [contours]
    
    backtorgb = cv.cvtColor(a_map, cv.COLOR_GRAY2RGB)
    c = 0
    for contours in ccontours:
        cv.drawContours(backtorgb, contours, -1, (c,255,0), r)
        c = 255 if c == 0 else 0
    return ccontours, backtorgb
    

# imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
# 
# 
# contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# longest = sorted(contours, key=lambda x : cv.arcLength(x, 1))[-1]

# cv.drawContours(im, contours, -1, (0,0,0), 1)


# cv.imwrite('/tmp/out.png', im)
# cv.waitKey(0)

if __name__ == "__main__":
    try:
        fname = sys.argv[1]
        a_map = get_map(fname)
        start_pos = find_start_pos(a_map, START_POS_CODE)
        print(start_pos)
        contour_path, image = find_contour_path(a_map)
        print(contour_path)
        cv.imwrite(f'/tmp/c.{fname}', cv.resize(image, None, fx=10, fy=10, interpolation=0))
    except IndexError:        
        print(f'{sys.argv[0]} fname')


