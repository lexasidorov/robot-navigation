import cv2 as cv
import numpy as np
import subprocess
from student import Student


FILE_EXT = 'jpeg'
MIN_MATCH_COUNT = 30
FLANN_INDEX_KDTREE = 1

ax,bx,cx,dx = [-97.21329492, 573.47918779, 437.03542331, 537.20596175]
ay,by,cy,dy = [-73.99297548, 453.36910223, 632.39898287, 313.42869025]

H = 650
W = 1260

sift = cv.SIFT_create()
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

class VisualNav():


    def get_object_list(self):
        # 'path': 'folder/filename.ext' !!!
        self.object_list = [
            {'path': 'imgs/2_бабы.png', 'h': 0.1, 'w': 0.15, 'x': 1, 'y': 2},
            {'path': 'imgs/дирижабль.png', 'h': 0.1, 'w': 0.15, 'x': 3, 'y': 4}, 
            {'path': 'imgs/самолёт.png', 'h': 0.1, 'w': 0.15, 'x': 4, 'y': 5}
        ]
        self.test_object_list = [
            {'path': 'test/sample.jpeg', 'h': 650, 'w': 1260, 'x': 0, 'y': 1520},
            {'path': 'test/sample.2.jpeg', 'h': 900, 'w': 1300, 'x': 4000, 'y': 0}, 
            {'path': 'test/point.2.jpeg', 'h': 900, 'w': 1300, 'x': 4000, 'y': 0}, 
            {'path': 'test/sample.2.jpeg', 'h': 900, 'w': 1300, 'x': 4000, 'y': 0}, 
            {'path': 'test/sample.2.jpeg', 'h': 900, 'w': 1300, 'x': 4000, 'y': 0}, 
            {'path': 'test/sample.2.jpeg', 'h': 900, 'w': 1300, 'x': 4000, 'y': 0}, 
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
                # "--frames", "10",
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


    def get_distance_by_image(self, img_sample, cam_id):
        
        distance = None

        kp1, des1 = sift.detectAndCompute(img_sample,None)
        img = self.get_frame(cam_id, f'/tmp/out_{cam_id}.jpeg')
        kp2, des2 = sift.detectAndCompute(img, None)
        matches = flann.knnMatch(des1, des2, k=2)

        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h,w = img_sample.shape

            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv.perspectiveTransform(pts,M)
            img = cv.polylines(img,[np.int32(dst)],True,255,3, cv.LINE_AA)
            # print('>>>', dst, '<<<', dist)
            # cv.imwrite(f'/tmp/{dist}.{ext}', img)

            p0, p1, p2, p3 = dst
            w = np.array([p2[0][0] - p1[0][0], p3[0][0] - p0[0][0]]).mean()
            h = np.array([p1[0][1] - p0[0][1], p2[0][1] - p3[0][1]]).mean()

            x = W/w
            y = H/h

            dist_x = ax*x**3 + bx*x**2 + cx*x + dx
            dist_y = ay*x**3 + by*x**2 + cy*x + dy

            distance = Student([dist_x, dist_y]*3)

        return distance

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

if __name__ == '__main__':
    
    vn = VisualNav()
    try:
        cam_id = sys.argv[1]
        img_samples = [(cv.imread(x, 0), x) for x in glob('distance_measurement_and_calibration/*.jpeg')]
        for img_sample in img_samples:
            print(vn.get_distance_by_image(img_sample[0], int(cam_id)))

    except IndexError as e:
        print(e)

