import cv2 as cv
import numpy as np
import subprocess
import sys, os

from student import Student, NotEnoughValuesException
from points import points
from find_nearest import find_nearest

from uuid import uuid4
from itertools import product
from sympy import solve as sympy_solve
from sympy import symbols as sympy_symbols
from sympy import Point, Line, Circle
from time import sleep 

MIN_MATCH_COUNT = 10
FLANN_INDEX_KDTREE = 1

last_points_json = {
"0": {
        "6": {
            "file": "/home/pi/robot-visual-nav/visual_nav/samples/point_7_0_420_297.jpeg",
            "w": 420,
            "h": 297,
            "x": 0,
            "y": -2170
        },
        "4": {
            "file": "/home/pi/robot-visual-nav/visual_nav/samples/point_7_0_420_297.jpeg",
            "w": 420,
            "h": 297,
            "x": -1150,
            "y": 0
        }
    }
}

ax,bx = [1251.25160563, -17.28192787]
ay,by = [1280.69562257, -35.18519642]

ax,bx = [1329.04219563,  78.20686245]
ay,by = [1280.83119763, 113.16692996]

ax,bx,cx = [6.521940996950816e-06, 0.26822368188899237, 3.671421158909142]
ay,by,cy = [1.1025724309974525e-05, 0.37985729242098093, 2.1074272313293476]

ax,bx,cx = [1.0431395681815249e-05, 1.3625622437072178, 26.009892307641252 + 240]
ay,by,cy = [7.3294444981205065e-06, 1.9443238935590166,  8.168795035881312 + 330]

sift = cv.SIFT_create()
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

class CamError(Exception):

    def __init__(self, message):
        self.message = message


class VisualNav():

    def get_object_list(self):
        return None


    def get_frames_fswebcam(self, id, n=4):
        filenames = [ f'/tmp/vn-{uuid4()}.jpeg' for i in range(n) ]
        try:
            for filename in filenames:
                x = subprocess.run([
                    "fswebcam", 
                    "-d", f"/dev/video{id}",
                    "--skip", "3", 
                    "--frames", "10",
                    "-r", "1920x1024",
                    "--no-banner",
                    "--rotate", "180", filename])
                assert x.returncode == 0
                frame = cv.imread(filename, 0) 
                len(frame)
                yield frame 

        except TypeError as e:
            print(f'[ ERROR ] No frame from camera {id}, {e}!', file=sys.stderr)
            raise CamError(f'Can\'t get frame on camera {id}.')

        except AssertionError as e:
            print(f'[ ERROR ] No frame from camera {id}, {e}!', file=sys.stderr)
            raise CamError(f'Subprocess {x} falied with {x.returncode}.')


    def get_frames(self, id, n=4):
        id = int(id)
        cam = cv.VideoCapture(id)
        print(f"cam = cv.VideoCapture({id})")
        assert cam.isOpened()
        print(f"cam.isOpened()")

        cam.set(3, 1920)
        cam.set(4, 1080)

        for i in range(n):
            # frame = np.zeros((int(cam.get(4)*2),int(cam.get(3)*2)))
            # print(f"frame({(int(cam.get(4)*2),int(cam.get(3)*2))})")
            # frame[0::2,0::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
            # frame[1::2,0::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
            # frame[0::2,1::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
            # frame[1::2,1::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)

            img = cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
            print(f"frame({img.shape})")

            # img = np.array(frame.astype(np.uint8))
            
            cv.imwrite(f'/tmp/log_shoot_{id}_{uuid4()}.jpeg', img)

            yield img

        cam.release()

    def get_good_keypoints(self, kp1, des1, img):
        kp2, des2 = sift.detectAndCompute(img, None)
        matches = flann.knnMatch(des1, des2, k=2)

        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        return good, kp2

    
    def is_good_keypoints_present(self, good):
        return len(good) > MIN_MATCH_COUNT


    def calc_distance_by_keypoints(self, kp1, kp2, good, h_0, w_0, W_mm, H_mm, img):
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        pts = np.float32([ [0,0],[0,h_0-1],[w_0-1,h_0-1],[w_0-1,0] ]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts,M)
        img = cv.polylines(img,[np.int32(dst)],True,255,3, cv.LINE_AA)

        cv.imwrite(f'/tmp/log_frame_{W_mm}_{H_mm}_{uuid4()}.jpeg', img)

        p0, p1, p2, p3 = dst
        w_i = np.array([p2[0][0] - p1[0][0], p3[0][0] - p0[0][0]]).mean()
        h_i = np.array([p1[0][1] - p0[0][1], p2[0][1] - p3[0][1]]).mean()

        x, y = W_mm * w_0/w_i, H_mm * h_0/h_i
        dist_x = ax*x*x + bx*x + cx
        dist_y = ay*y*y + by*y + cy
        print(22222222222222222, W_mm, H_mm, w_0, h_0, w_i, h_i, x, y, dist_x, dist_y)
                                # 420   297 1258  837  554  324 952 765  265    299
        return dist_x, dist_y


    def get_distance_by_image(self, img_sample, cam_id, W, H):
        
        distance = None
        measurements = []

        kp1, des1 = sift.detectAndCompute(img_sample,None)
        
        for img in self.get_frames(cam_id, 3):
            good, kp2 = self.get_good_keypoints(kp1, des1, img)
            if self.is_good_keypoints_present(good):
                sample_h, sample_w = img_sample.shape
                dist_x, dist_y = self.calc_distance_by_keypoints(kp1, kp2, good, sample_h, sample_w, W, H, img)
                measurements += [dist_x, dist_y]
        
        distance = Student(measurements)

        return distance


    def calculate_intersect_pts(self, circles):
        circle_a = circles.__next__()[0]
        circle_b = circles.__next__()[0]
        print(circle_a, circle_b)
        c1_x, c1_y = circle_a['c']
        r1_min = circle_a['r_min']
        r1_max = circle_a['r_max']
        c2_x, c2_y = circle_b['c']
        r2_min = circle_b['r_min']
        r2_max = circle_b['r_max']
        res_top = []
        res_btm = []
        
        rez = []
        x, y = sympy_symbols('x, y')

        for r1, r2 in product([r1_min, r1_max], [r2_min, r2_max]):
            s = sympy_solve([
                    ((x - c1_x)**2 + (y - c1_y)**2)**0.5 - r1,
                    ((x - c2_x)**2 + (y - c2_y)**2)**0.5 - r2,
                ], 'x, y', dict=True)
            try:
                float(s[0][x])
                rez += s
            except IndexError:
                print(f'[ ERROR ]: index [0][{x}] not in {s}', file=sys.stderr)

            except TypeError:
                print('[ WARNING ]: imaginary root found', file=sys.stderr)

        return rez


    def find_visible_objects(self, point):
        point = str(point)
        for cam_id in points[point]:
            cam_id = str(cam_id)
            fname = points[point][cam_id]['file']
            # print(os.path.dirname(os.path.realpath(__file__)), file=sys.stderr)
            # open(fname, 'rb')
            img = cv.imread(fname, 0)
            W, H, X, Y = [ points[point][cam_id][k] for k in ['w', 'h', 'x', 'y'] ]
            distance = self.get_distance_by_image(img, cam_id, W, H)
            yield {'fname':fname, 'W':W, 'H':H, 'X':X, 'Y':Y, 'distance':distance}
        

    def draw_circles(self, objects):
        for obj in objects:
            r_min = obj['distance'].X - obj['distance'].Dx
            r_max = obj['distance'].X + obj['distance'].Dx
            c = (obj['X'],obj['Y'])
            circle = [{
                'c': c,
                'r_min': r_min,
                'r_max': r_max,
            }]
            yield circle


    def calculate_my_coords(self):
        try:
            x, y = sympy_symbols('x, y')
            my_coords = {x:10000, y:2000}  # test!!!
            # point = int(sys.argv[1])
            point = 0
            coords = self.calculate_intersect_pts(self.draw_circles(self.find_visible_objects(point)))
            p1, p2, p3 = [ Point(v[x],v[y]) for v in find_nearest(my_coords,coords) ]
            c = Circle(p1, p2, p3)
            return [ float(a) for a in [c.center.x, c.center.y, c.radius]]
        except (NotEnoughValuesException, ValueError) as e:
            print(f'[ ERROR ] Got only one object. {e}', file=sys.stderr)

            return ('None','None','None')

if __name__ == '__main__':
    
    vn = VisualNav()
    print(vn.calculate_my_coords())
    # c1 = Circle(Point(0, 0), 5)

    # print(vn.calculate_intersect_pts({'c':(0,0),'r_min':1,'r_max':2}, {'c':(1,1),'r_min':1,'r_max':2}))
    # x = vn.get_frames(0, 20)
    # cv.imwrite('/tmp/out.jpeg', x)
