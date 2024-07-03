import sys
import cv2 as cv
import numpy as np
from typing import Dict, List
from pprint import pprint
from glob import glob
from VisualCoordinateNavigatior import VisualCoordinateNavigatior

# DistanceLowT = DistanceHighT = Dict[str, int]

# DistanceRangeT = List[Dict[DistanceLowT, DistanceHighT]]

def get_distance_test(object_list: list, search_images: list):

    res = []
    vcn = VisualCoordinateNavigatior()
    nearest_objects = vcn.find_nearest_objects(object_list, (0,0), '')
    res += [vcn.recognize(i[0], nearest_objects) for i in search_images]
    # print(len(res))
    for i in range(len(res)):
        try:
            d = res[i][1][0][2]
            # pprint(res[i])
            assert res[i][0]["meta"]["min_distance"] < d < res[i][0]["meta"]["max_distance"]
            print(f'[ INFO ] Test on {res[i][0]["meta"]["path"]} passed!', file=sys.stderr)
            # print(d)
        
        except IndexError as e:
            print(f'[ WARNING ] No objects recognized on {search_images[i][1]}, {e}')

        except AssertionError as e:
            print(f'[ ERROR ] Test falied on {res[i][0]["meta"]["path"]}!', file=sys.stderr)
    
    return 0

if __name__ == '__main__':

    test_object_list = [
        {'path': 'test/mark.0.jpeg', 'h': 0.5, 'w': 1, 'x': 1, 'y': 2, 'min_distance': 1, 'max_distance': 2}, 
        {'path': 'test/mark.1.jpeg', 'h': 1.5, 'w': 1, 'x': 3, 'y': 4, 'min_distance': 1, 'max_distance': 2}, 
    ]

    search_images = [(cv.imread(x),x) for x in glob('test/out*')]
    
    get_distance_test(test_object_list, search_images)


