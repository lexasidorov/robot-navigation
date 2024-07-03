import cv2 as cv
import numpy as np
from glob import glob

FILE_EXT = 'jpeg'
MIN_MATCH_COUNT = 30
FLANN_INDEX_KDTREE = 1

ax,bx,cx,dx = [-97.21329492, 573.47918779, 437.03542331, 537.20596175]
ay,by,cy,dy = [-73.99297548, 453.36910223, 632.39898287, 313.42869025]

H = 450
W = 940

sift = cv.SIFT_create()
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

if __name__ == '__main__':
    img_sample = cv.imread('sample.2.jpeg', 0)
    kp1, des1 = sift.detectAndCompute(img_sample,None)

    for name, ext in [ x.split('.') for x in glob(f'stage2/out_2_*.{FILE_EXT}') ]:
        img = cv.imread(f'{name}.{ext}')
        kp2, des2 = sift.detectAndCompute(img, None)
        matches = flann.knnMatch(des1, des2, k=2)

        dist = int(name.split('/')[1].replace('out_2_',''))

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

            print(dist, x, y, (dist_x + dist_y) / 2)


