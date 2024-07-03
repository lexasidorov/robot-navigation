import cv2 as cv
import numpy as np
import sys

from requests import get
from requests.exceptions import JSONDecodeError
from time import time
from uuid import uuid4

MIN_MATCH_COUNT = 20
FLANN_INDEX_KDTREE = 1
FNAME = '/home/pi/robot-visual-nav/parking/park.png'
PARKING_CAM_ID = 6
LENGTH_TO_LIDAR = 700
MIN_PARKING_DISTANCE = 150
PARKING_MOVE_DISTANCE = 300

sift = cv.SIFT_create()
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

# camera calibration
PTS_X = [1802.8830403645834, 1783.5368977864584, 1671.155887858073, 1011.5810770670573, 575.7378784179688, 286.5060668945313, 108.87550735473633]
PTS_Z = [-20,-17,-15,0,9,16,22]

ax,bx,cx = [-0.0009236470765215104, 3.986458583968185, -316.2816196847409]
ay,by,cy = [-0.00020817205800883348, 2.5861520885206897, -288.9747693325901]


class CamError(Exception):

    def __init__(self, message):
        self.message = message


class MotionControl():

    def control_move(self, dir, min_distance, wanted_ir_state, timeout=10):
        get('http://localhost:4998/state/get')
        t0 = time()
        rez = 'ok'
        while 1:
            if dir in ['left', 'right', 'back']:
                get_distance = self.get_bck_distance
                get_ir_state = self.get_bck_ir_state
            else:
                get_distance = self.get_fwd_distance
                get_ir_state = self.get_fwd_ir_state
            distance = get_distance()
            ir_state = get_ir_state()
            state = get('http://localhost:4998/state/get').json()['drive_state']
            print(f'distance = {distance}')
            print(f'ir_state = {ir_state}')
            print(f'state = {state}')
            if state == 'stop': 
                rez = 'stop_from_drive'
                break
            if distance < min_distance or \
               ir_state == wanted_ir_state:
                rez = 'stop_from_environment'
                print(f'{distance} < {min_distance}, {ir_state} == {wanted_ir_state}')
                self.stop()
                break
            if time() - t0 > timeout:
                rez = 'stop_from_timeout'
                print(f'{time()} - {t0} > {timeout}')
                self.stop()
                break
        return rez
            

    def back(self, distance):
        print('back', distance)
        get(f'http://localhost:4998/bck/1/{distance}').json()
        return 'back'


    def go(self, distance):
        print('go', distance)
        get(f'http://localhost:4998/fwd/1/{distance}').json()
        return 'go'


    def left(self, angle):
        print('left', angle)
        get(f'http://localhost:4998/bck/lft/1/{angle}').json()
        return 'left'
        

    def right(self, angle):
        print('right', angle)
        get(f'http://localhost:4998/bck/rgt/1/{angle}').json()
        return 'right'


    def stop(self):
        print(get('http://localhost:4998/stop').json())
        return 'stop'


    def get_bck_distance(self):
        try:
            rez = get('http://localhost:4996/state/get').json()
            distance = rez['distances'][6] - LENGTH_TO_LIDAR
        except Exception as e:
            print(f'[ ERROR ] line 92 {e}')
            distance = 0
        return distance


    def get_fwd_distance(self):
        try:
            rez = get('http://localhost:4996/state/get').json()
            distanceA = rez['distances'][1] - LENGTH_TO_LIDAR
            rez = get('http://localhost:4993/environment').json()
            distanceB = rez['env']['frw']['mdl']
            distance = min(distanceA, distanceB)
        except Exception as e:
            print(f'[ ERROR ] line 105 {e}')
            distance = 0
        return distance


    def get_bck_ir_state(self):
        try:
            rez = get('http://localhost:4995/state/get').json()
            bkw_lft_ik = rez['bkw_lft_ik']
            bkw_rgt_ik = rez['bkw_rgt_ik']
        except Exception as e:
            print(f'[ ERROR ] line 131 {e}')
            bkw_lft_ik = 1
            bkw_rgt_ik = 1
        return bkw_lft_ik + bkw_rgt_ik


    def get_fwd_ir_state(self):
        try:
            rez = get('http://localhost:4995/state/get').json()
            bkw_lft_ik = rez['frw_lft_ik']
            bkw_rgt_ik = rez['frw_rgt_ik']
        except Exception as e:
            print(f'[ ERROR ] line 143 {e}')
            bkw_lft_ik = 1
            bkw_rgt_ik = 1
        return bkw_lft_ik + bkw_rgt_ik


class VisualPark():

    def __init__(self, do_debug=False):
        self.__mc = MotionControl()
        self.__debug = do_debug

    def __get_frame(self, id):
        cam = cv.VideoCapture(id)
        try:
            assert cam.isOpened(), f'Camera {id} is not opened!'

            cam.set(3, 1920), cam.set(4, 1080)
            for x in range(10):
                cam.read()
            img = cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
            if self.__debug: cv.imwrite(f'/tmp/log_shoot_{id}_{uuid4()}.jpeg', img)
            
            cam.release()
        except AssertionError as e:
            print(f'[ ERROR ] line 168 {e}',file=sys.stderr)

        return img


    def __get_good_keypoints(self, kp1, des1, kp2, des2):
        good = []
        try:
            matches = flann.knnMatch(des1, des2, k=2)

            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)

        except cv.error as e:
            print(f'[ ERROR ] OpenCV error {e}', file=sys.stderr)
            raise CamError('[ ERROR ] OpenCV error {e}')

        return good


    def __is_good_keypoints_present(self, good):
        return len(good) > MIN_MATCH_COUNT


    def __calc_distance_by_keypoints(self, kp1, kp2, good, h_0, w_0, W_mm, H_mm, img):
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        # matchesMask = mask.ravel().tolist()

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


    def __calc_angle_by_keypoints(self, parking_img, station_img):
        f = np.poly1d(np.polyfit(PTS_X, PTS_Z, 1))
        
        parking_kp1, parking_des1 = sift.detectAndCompute(parking_img, None)
        station_kp2, station_des2 = sift.detectAndCompute(station_img, None)

        good = self.__get_good_keypoints(parking_kp1, parking_des1, station_kp2, station_des2)
        if self.__is_good_keypoints_present(good):
            X = np.array([ station_kp2[x.trainIdx].pt[0] for x in good ])
            angle = f(np.median(X))
            # distance = __calc_distance_by_keypoints(self, parking_kp1, station_kp2, good, h_0, w_0, W_mm, H_mm, img):
            if self.__debug:
                draw_params = dict(matchColor = (0,255,0), )
                img = cv.drawMatches(parking_img,parking_kp1,station_img,station_kp2,good,None,**draw_params)
                cv.imwrite(f'/tmp/log_recognize_{uuid4()}.jpeg', img)
                np.savetxt(f'/tmp/log_recognize_{uuid4()}.csv', X, fmt='%i', delimiter=",")
        else:
            print(f'[ ERROR ] not enough kp {len(good)}', file=sys.stderr)
            raise CamError(f'[ ERROR ] not enough kp {len(good)}')

        return angle

    def __calculate_parking_angle_distance(self):
        parking_img = cv.imread(FNAME, 0)
        station_img = self.__get_frame(PARKING_CAM_ID)
        angle = self.__calc_angle_by_keypoints(parking_img, station_img)
        angle = angle / 3 if angle > 4 else angle
        # distance = self.__calc_dist_by_keypoints(parking_img, station_img)
        return round(angle), 0 # round(distance)


    def __calculate_parking_move(self, lidar_distance, ir_state):
        if ir_state == 1:
            distance = MIN_PARKING_DISTANCE // 4
        elif lidar_distance > MIN_PARKING_DISTANCE:
            distance = int(lidar_distance / 1.15)
        else:
            distance = MIN_PARKING_DISTANCE
        return round(distance)


    def do_park(self):
        direction = ''
        lidar_distance, ir_state = None, None
        while 1:
            try:
                angle, distance = self.__calculate_parking_angle_distance()
                lidar_distance = self.__mc.get_bck_distance()
                ir_state = self.__mc.get_bck_ir_state()
                
                if abs(angle) > 1:
                    if angle > 0: direction = self.__mc.left(abs(angle))
                    if angle < 0: direction = self.__mc.right(abs(angle))
                if lidar_distance >= MIN_PARKING_DISTANCE:
                    direction = self.__mc.back(self.__calculate_parking_move(lidar_distance, ir_state))
                if lidar_distance < MIN_PARKING_DISTANCE or ir_state == 2:
                    break
            except CamError:
                direction = self.__mc.go(PARKING_MOVE_DISTANCE)
                angle = 0
            except JSONDecodeError:
                break
            control_state = self.__mc.control_move(direction, MIN_PARKING_DISTANCE, 2, timeout=5)
            if self.__debug:
                print('angle', angle)
                print('lidar_distance', lidar_distance)
                print('ir_state', ir_state)
                print('control_state', control_state)
            if control_state == 'stop_from_environment':
                break

        self.__mc.stop()


if __name__ == '__main__':
    vp = VisualPark(do_debug=True)

    print(vp.do_park())
