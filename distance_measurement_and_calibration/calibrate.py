import cv2 as cv
import numpy as np
from glob import glob
from scipy.optimize import curve_fit

def foo(x, a, b):
    return a * 1/x + b

FILE_EXT = 'jpeg'
MIN_MATCH_COUNT = 10
FLANN_INDEX_KDTREE = 1

H_mm = 300 # mm
W_mm = 210
X = []
W = []
H = []

sift = cv.SIFT_create()
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

if __name__ == '__main__':
    img_sample = cv.imread('/tmp/aa.jpeg', 0)
    kp1, des1 = sift.detectAndCompute(img_sample,None)

    for dist, ext in [ x.split('.') for x in glob(f'/tmp/imgs/*.{FILE_EXT}') ]:
        img = cv.imread(f'{dist}.{ext}')
        kp2, des2 = sift.detectAndCompute(img, None)
        matches = flann.knnMatch(des1, des2, k=2)

        dist = int(dist.split('/')[-1]) # * 10 # mm

        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        # print(dist, len(good))
        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h_0,w_0 = img_sample.shape # px

            pts = np.float32([ [0,0],[0,h_0-1],[w_0-1,h_0-1],[w_0-1,0] ]).reshape(-1,1,2)
            dst = cv.perspectiveTransform(pts,M)
            img = cv.polylines(img,[np.int32(dst)],True,255,3, cv.LINE_AA)
            # print('>>>', dst, '<<<', dist)
            cv.imwrite(f'/tmp/{dist}.{ext}', img)

            p0, p1, p2, p3 = dst
            w_i = np.array([p2[0][0] - p1[0][0], p3[0][0] - p0[0][0]]).mean()
            h_i = np.array([p1[0][1] - p0[0][1], p2[0][1] - p3[0][1]]).mean()
            x, y = W_mm * w_0/w_i, H_mm * h_0/h_i
            # print(dist, W_mm * w_i/w_0, H_mm * h_i/h_0, W_mm, w_i, w_0, H_mm, h_i, h_0)
            print(W_mm, H_mm, w_0, h_0, w_i, h_i, x, y, dist)
            X += [dist]
            W += [x] # mm / px
            H += [y]
            

    # pars_w, cov = curve_fit(f=foo, xdata=X, ydata=W, p0=[0, 0], bounds=(-np.inf, np.inf))
    # pars_h, cov = curve_fit(f=foo, xdata=X, ydata=H, p0=[0, 0], bounds=(-np.inf, np.inf))
    aw, bw, cw = np.polyfit(W,X,2)
    ah, bh, ch = np.polyfit(H,X,2)
    print(X, W, H)
    print(aw, bw, cw)
    print(ah, bh, ch)
    for d, w, h in zip(X, W, H):
        print(d, aw*w**2 + bw*w + cw, ah*h**2 + bh*h + ch)

