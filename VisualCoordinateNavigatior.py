import sys
import cv2 as cv
import numpy as np
import subprocess
from util import compare_imgs, draw_borders, CamError, MIN_MATCH_COUNT
from student import Student
from SVGParser import SVGParser
from math import tan, radians
from sympy import solve as sympy_solve
from sympy import symbols as sympy_symbols
from pprint import pprint
from glob import glob

MAX_DX = 1.276569083
MIN_KP_VALUE = 30

AX,BX,CX,DX = [-97.21329492, 573.47918779, 437.03542331, 537.20596175]
AY,BY,CY,DY = [-73.99297548, 453.36910223, 632.39898287, 313.42869025]

def my_print(obj):
    print('{')
    for k, v in obj.items():
        try:
            print(f"\t{k}:{f'{v[:20]}...' if len(v) > 20 else v}")
        except TypeError:
            print(f"\t{k}:{v}")
    print('}')

class VisualCoordinateNavigatior():

    description = dict(
            cameras = dict(
                cam_0 = dict(
                    A = radians(75.9),
                    B = radians(42.7),
                    angle = 0,
                    W = 1920,
                    H = 1080,
                    id = '0'
                ),
                cam_2 = dict(
                    A = radians(75.9),
                    B = radians(42.7),
                    angle = 90,
                    W = 1920,
                    H = 1080,
                    id = '2'
                ),
                cam_4 = dict(
                    A = radians(75.9),
                    B = radians(42.7),
                    angle = 180,
                    W = 1920,
                    H = 1080,
                    id = '4'
                ),
                cam_6 = dict(
                    A = radians(75.9),
                    B = radians(42.7),
                    angle = 270,
                    W = 1920,
                    H = 1080,
                    id = '6'
                ),
            ),
        )

    def __init__(self, test=False):
        self.test = test
        self.sift = cv.SIFT_create()
        self.matcher = cv.BFMatcher()
        self.A_cam, self.B_cam = self.description['cameras']['cam_0']['A'], self.description['cameras']['cam_0']['B']
        self.H_cam, self.W_cam = self.description['cameras']['cam_0']['H'], self.description['cameras']['cam_0']['W']

    def get_object_list(self):
        # 'path': 'folder/filename.ext' !!!
        self.object_list = [
            {'path': 'imgs/2_бабы.png', 'h': 0.1, 'w': 0.15, 'x': 1, 'y': 2},
            {'path': 'imgs/дирижабль.png', 'h': 0.1, 'w': 0.15, 'x': 3, 'y': 4}, 
            {'path': 'imgs/самолёт.png', 'h': 0.1, 'w': 0.15, 'x': 4, 'y': 5}
        ]
        self.test_object_list = [
            {'path': 'test/avrora.jpg', 'h': 600, 'w': 300, 'x': 0, 'y': 1520},
            {'path': 'test/stol.jpg', 'h': 900, 'w': 1300, 'x': 4000, 'y': 0}, 
            {'path': 'test/инструменты.jpeg', 'h': 650, 'w': 650, 'x': 8500, 'y': 2200}, 
            {'path': 'test/станок.jpeg', 'h': 1200, 'w': 500, 'x': 7000, 'y': 1000}, 
            # {'path': 'test/mark_2.jpeg', 'h': 0.4, 'w': 0.7, 'x': 2, 'y': 6}
        ]

        self.object_list = self.test_object_list #if self.test else self.object_list

        return self.object_list

    def get_frame(self, id, filename):
        # pprint(('get_frame',id))
        # cap = cv.VideoCapture(id)
        try:
            x = subprocess.run([
                "fswebcam", 
                "-d", f"/dev/video{id}",
                "--skip", "24", 
                "-r", "1920x1024",
                "--no-banner",
                "--rotate", "180", filename])
            assert x.returncode == 0
            frame = cv.imread(filename, 1)
            l = len(frame)
            frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        except TypeError as e:
            print(f'[ ERROR ] No frame from camera {id}, {e}!', file=sys.stderr)
            raise CamError(f'Can\'t get frame on camera {id}.')

        except AssertionError as e:
            print(f'[ ERROR ] No frame from camera {id}, {e}!', file=sys.stderr)
            raise CamError(f'Subprocess {x} falied with {x.returncode}.')

        return frame


    def find_nearest_by_map(self, xml_map, object_list) -> list:
        osm_parser = SVGParser(xml_map)
        # osm_parser.parse()
        filtered_objects = object_list # TODO: temp line, replace!

        return filtered_objects

    def find_nearest_objects(self, object_list, coords, xml_map) -> list:
        # pprint(('find_nearest_objects',object_list, coords, xml_map))

        nearest_objects = []
        # TODO: filter objects to find nearest by xml_map
        filtered_objects = self.find_nearest_by_map(xml_map, object_list)

        # TODO: change to filtered
        for object in object_list: # source images to find on frames
            # TODO: add w_o and h_o in object descriptions
            i = cv.imread(object['path'], 0)
            kp1, des1 = self.sift.detectAndCompute(i, None)
            # TODO: need to zapihyat all objects info to the list
            nearest_objects += [(kp1, des1, i, object['path'].split('/')[1].split('.')[0], object)]
        # pprint(('find_nearest_objects',len(nearest_objects)))

        return nearest_objects


    def xy_to_ab_radians(self, xy=[0,0], AB=[0,0], obj_id=None):
        # pprint(('xy_to_ab_radians', xy, AB, obj_id))

        Xo, Yo = xy[0] - self.W_cam / 2, self.H_cam / 2 - xy[1]
        Ao, Bo = AB[0] - 0.0017 * Xo, AB[1] + 0.0019 * Yo
        # pprint(('xy_to_ab_radians', np.rad2deg([Ao, Bo]), obj_id))

        return Ao, Bo


    def spher_to_cart(self, a, b, r1, r2, obj_id=None):
        # pprint(('spher_to_cart',a,b,r))

        x = (r1 * np.sin(b) * np.cos(a) + r2 * np.sin(b) * np.cos(a)) / 2
        y = (r1 * np.sin(b) * np.sin(a) + r2 * np.sin(b) * np.sin(a)) / 2
        z1 = r1 * np.cos(b)
        z2 = r2 * np.cos(b)
        # pprint(('spher_to_cart',x,y,z, obj_id))

        return x, y, z1, z2


    def get_spher_coords(self, dst, H, W, h_o, w_o, obj_id=None):
        # pprint(('get_spher_coords',dst, H, W, h_o, w_o))

        X1 = [dst[0][0][0], dst[2][0][0]]
        X2 = [dst[1][0][0], dst[3][0][0]]
        Y1 = [dst[0][0][1], dst[2][0][1]]
        Y2 = [dst[1][0][1], dst[3][0][1]]

        k1, b1 = np.polyfit(X1,Y1,1)
        k2, b2 = np.polyfit(X2,Y2,1)
        
        x = (b2 - b1) / (k1 - k2)
        y = k1 * x + b1
        a,b = self.xy_to_ab_radians([x,y],[radians(0),radians(0)],obj_id)
        # TODO: add actions after object diagonals checking
        try:

            p0, p1, p2, p3 = dst
            print('pppppppppppppppp', p0, p1, p2, p3)
            w = np.array([p2[0][0] - p1[0][0], p3[0][0] - p0[0][0]]).mean()
            h = np.array([p1[0][1] - p0[0][1], p2[0][1] - p3[0][1]]).mean()

            x = W/w
            y = H/h

            dist_x = AX*x**3 + BX*x**2 + CX*x + DX
            dist_y = AY*y**3 + BY*y**2 + CY*y + DY

            # print(dist, x, y, dist_x, dist_y)

            z = Student([dist_x, dist_y]*3)

        except ZeroDivisionError:
            z = 'n/a'

        # pprint(('get_spher_coords',a,b,z.X))

        return a,b,z

    def recognize(self, frame, nearest_object_list, cam_id):
        # pprint(('recognize', frame.shape, len(nearest_object_list)))

        objects_info = {}
        coordinates = []
        for data in nearest_object_list:
            kp1, des1, query_img, class_name, meta = data
            res_cmp_coords, len_of_kp = compare_imgs(frame, kp1, des1, query_img, self.sift, self.matcher, class_name, cam_id)
            try:
                H = max(res_cmp_coords[1][0][1],res_cmp_coords[2][0][1]) - min(res_cmp_coords[0][0][1],res_cmp_coords[3][0][1])
                W = max(res_cmp_coords[2][0][0],res_cmp_coords[3][0][0]) - min(res_cmp_coords[1][0][0],res_cmp_coords[0][0][0])
                try:
                    coeff = H/W
                    a,b,z = self.get_spher_coords(res_cmp_coords, H, W, meta['h'], meta['w'], class_name)
                    
                    assert len_of_kp > 30, f'{len_of_kp} <= 30'
                    assert 20000 > z.X > 0, f'not 20000 > {z.X} > 0'
                    assert z.Dx / z.X < 0.1, f'{z.Dx} / {z.X} = {z.Dx / z.X} > 0.1'

                    print(f'not 20000 > {z.X} > 0')
                    print(f'{z.Dx} / {z.X} = {z.Dx / z.X} > 0.1')

                    coordinates = [self.spher_to_cart(a,b,z.X-z.Dx,z.X+z.Dx)]
                    objects_info.update({
                        # obj_name
                        data[3]: {
                            # object,       tuple(x, y, z),        KeyPoints
                            "obj": data[4], "coords": coordinates, "kp": data[0]
                        }
                    })
                except (ZeroDivisionError, AssertionError) as e:
                    coeff = -1
                    print(f'[ WARNING ] {e}!', file=sys.stderr)
                    continue
                # print(coeff)
                # if 1 < coeff < 2:
                # res_all_coords += [res_cmp_coords]
            except IndexError:
                print('[ ERROR ] Not found src image on frame!', file=sys.stderr)
        # pprint(('recognize', objects_info))

        return objects_info

    def find_visible_objects(self, nearest_object_list):
        # pprint(('find_visible_objects',len(nearest_object_list)))

        cameras = self.description['cameras']
        cam_ids = [int(cameras[x]['id']) for x in cameras.keys()]
        # coordinates = []
        if self.test:
            objects = {}
            for img, key in [(cv.imread(x, 0), x) for x in glob('test/out*')]:
                for k,v in self.recognize(img, nearest_object_list, key).items():
                    v.update({'source': key})
                    objects.update({k:v})
            return objects
        
        objects_info = {}
        for cam_id in cam_ids:
            frame = self.get_frame(cam_id, f"/tmp/out_{cam_id}.jpg")
            # cv.imwrite(f'test_frame{cam_id}.jpg', frame)
            for k,v in self.recognize(frame, nearest_object_list, cam_id).items():
                v.update({'source': cam_id})
                objects_info.update({k:v})
            
        # pprint(('find_visible_objects',objects_info))

        return objects_info

    def calculate_intersect_pts(self, circle_center_xy1, circle_center_xy2, \
        radius1_top, radius2_top, radius1_btm, radius2_btm):
        res_top = []
        res_btm = []
        print(radius1_top, radius2_top, radius1_btm, radius2_btm)
        x, y = sympy_symbols('x, y')
        res_top += [sympy_solve([
                ((x - circle_center_xy1[0])**2 + (y - circle_center_xy1[1])**2)**0.5 - radius1_top,
                ((x - circle_center_xy2[0])**2 + (y - circle_center_xy2[1])**2)**0.5 - radius2_top
            ], 'x, y', dict=True)
        ]
        res_btm += [sympy_solve([
                ((x - circle_center_xy1[0])**2 + (y - circle_center_xy1[1])**2)**0.5 - radius1_btm,
                ((x - circle_center_xy2[0])**2 + (y - circle_center_xy2[1])**2)**0.5 - radius2_btm
            ], 'x, y', dict=True)
        ]

        return [{
            'x': res_top[0][0][x], 
            'y': res_top[0][0][y]
        },{
            'x': res_top[0][1][x], 
            'y': res_top[0][1][y]
        },{
            'x': res_btm[0][0][x], 
            'y': res_btm[0][0][y]
        },{
            'x': res_btm[0][1][x], 
            'y': res_btm[0][1][y]
        }]

    def filter_2_objects(self, objects):
        # from objects [
        #   {'path': 'imgs/2_бабы.png', 'h': 0.1, 'w': 0.15, 'x': 1, 'y': 2},
        #   list(tuple(x, y, z)),
        #   [<KeyPoint>, ... ],
        #   obj_name
        # ]
        # - z, n_kp
        # from nearest_object_list - x, y
        # new list {"name":name, "point":[x, y, z], "n_kp":n_kp}
        # sorted by n_kp
        # pprint(objects)

        # objects = sorted(objects, key=lambda x : len(x['kp']))
        res_obj_list = []
        for k,v in objects.items():
            if len(v['kp']) > MIN_KP_VALUE:
                res_obj_list += [v]

        # pprint(res_obj_list)
        
        return res_obj_list

    def get_obj_data(self, o, edge='top'):
        x, y = o['obj']['x'], o['obj']['y']
        try:
            if edge == 'top':
                # max (distance-z.Dx)
                z = o['coords'][0][3]
            else:
                # min (distance+z.Dx)
                z = o['coords'][0][2]
        except IndexError as e:
            print(f'[ ERROR ] {o}.')
            raise e
        
        return (x,y), z

    def get_ordered_list(self, points: list, point: tuple) -> list:
        # points.sort(key = lambda p: (p['x'] - point[0])**2 + (p['y'] - point[1])**2)

        return points

    def find_nearest_root(self, points: list, point):
        return self.get_ordered_list(points, point)


    def calculate_my_coords(self, cur_coords, objects, nearest_object_list):
        try:
            obj1, obj2 = self.filter_2_objects(objects)
            # pprint(obj1)
            circle_center_xy1, radius1_top = self.get_obj_data(obj1, edge='top')
            circle_center_xy1, radius1_btm = self.get_obj_data(obj1, edge='btm')
            circle_center_xy2, radius2_top = self.get_obj_data(obj2, edge='top')
            circle_center_xy2, radius2_btm = self.get_obj_data(obj2, edge='btm')

            res = self.find_nearest_root(
                self.calculate_intersect_pts(
                    circle_center_xy1, 
                    circle_center_xy2, 
                    radius1_top, 
                    radius2_top,
                    radius1_btm, 
                    radius2_btm
                ),
                cur_coords
            )
            # res = (0,0)
            c = open('/tmp/coords_list.txt', 'w').write(f'{res[1]["x"]},{res[1]["y"]}')
            return res
        except ValueError as e:
            print(f'[ ERROR ] Got only one object. {e}')

            return (None,None)


    def get_coords(self, coords=(0,0)):
        xml = '' # TODO: open map
        # TODO: move to external params
        # object_list = [
        #     {'path': 'test/mark.0.jpeg', 'h': 0.5, 'w': 1, 'x': 1, 'y': 2}, 
        #     {'path': 'test/mark.1.jpeg', 'h': 1.5, 'w': 1, 'x': 3, 'y': 4}, 
        # ]
        object_list = self.get_object_list()
        nearest_object_list = self.find_nearest_objects(object_list, coords, xml)
        recognized_objects = self.find_visible_objects(nearest_object_list)
        my_coords = self.calculate_my_coords(coords, recognized_objects, nearest_object_list)
     
        return my_coords


if __name__ == '__main__':
    try:
        assert sys.argv[1]
        command = 1

    except AssertionError:
        command = 0

    if command:
        if sys.argv[1] == 'test':
            vcn = VisualCoordinateNavigatior(test=True)
            print(vcn.get_coords())
        else:
            vcn = VisualCoordinateNavigatior(test=False)
            print(vcn.get_coords())
