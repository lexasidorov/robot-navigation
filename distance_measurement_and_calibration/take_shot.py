import cv2 as cv
import numpy as np
import sys

def get_frames(id, n=4, w=640, h=480):
    cam = cv.VideoCapture(id)
    assert cam.isOpened()

    cam.set(3, 1920)
    cam.set(4, 1080)

    frame = np.zeros((int(cam.get(4)*2),int(cam.get(3)*2)))
    for i in range(n):
        frame[0::2,0::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
        frame[1::2,0::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
        frame[0::2,1::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
        frame[1::2,1::2] += cv.cvtColor(cam.read()[1], cv.COLOR_BGR2GRAY)
    
    return (frame / n).astype(np.uint8)

if __name__ == '__main__':
    try:
        id = int(sys.argv[1])
        w = int(sys.argv[2])
        h = int(sys.argv[3])
    except IndexError:
        id = 0
        w = 640
        h = 480
    img = get_frames(id, 10, w, h)
    print(f'/tmp/out_{id}_{w}_{h}.jpeg written')
    cv.imwrite(f'/tmp/out_{id}_{w}_{h}.jpeg', img)


