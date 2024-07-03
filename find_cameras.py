import cv2 as cv
import numpy as np
import traceback as tb
import sys
from glob import glob

FLANN_INDEX_KDTREE = 1

def get_frames(id, n=4, w=640, h=480):
    cam = cv.VideoCapture(id)
    assert cam.isOpened()
    assert cam.read()[0] != False

    cam.set(3, 1920)
    cam.set(4, 1080)


    # frame = np.zeros((int(cam.get(4)*2),int(cam.get(3)*2)))
    frame = cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
    # for i in range(n):
    #     frame[0::2,0::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
    #     frame[1::2,0::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
    #     frame[0::2,1::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
    #     frame[1::2,1::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
    print(f'[ INFO ] Camera {id} is working!')
    # return (frame / n).astype(np.uint8)
    return frame

def compare(img1, img2):
    sift = cv.SIFT_create()
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv.FlannBasedMatcher(index_params, search_params)

    print(f'[ INFO ] {len(img1)}, {len(img2)}')

    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    matches = flann.knnMatch(des1, des2, k=2)

    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    return len(good)

def process_result(res):

    r = {}
    values = set()
    try:
        for k,v in res.items():
            assert len(v) > 0
            r[k] = sorted(v, key=lambda x: x[1], reverse=True)[0][0]
            values.update(r[k])
        assert len(values) == 4
    except AssertionError:
        print('[ ERROR ] Can not determine cameras!')

    return r

if __name__ == '__main__':
    photos = glob('/home/pi/robot-visual-nav/imgs.conf/*.jpeg')
    res = {
        "l": [],
        "r": [],
        "f": [],
        "b": []
    }
    for c in glob('/dev/video*'):
        try:
            c_id = c.split('video')[-1]
            img = get_frames(int(c_id), 10, 640, 480)
            for side in ["l", "r", "f", "b"]:
                print(f'[ INFO ] {side}, {c_id}')
                fname = f'imgs.conf/{side}.jpeg'
                matches = compare(cv.imread(fname, 0).astype(np.uint8), img)
                res[side] += [(c_id, matches)]
            
            print(process_result(res))

        except AssertionError:
            print(f'Cam on device id `{c_id}` not found!')
        
        # cv.imwrite(f'/tmp/out_{c_id}_{w}_{h}.jpeg', img)


