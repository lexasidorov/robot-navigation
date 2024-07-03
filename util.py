import numpy as np
import cv2 as cv
import time
import traceback as tb
from time import sleep
from glob import glob
import math
from pprint import pprint 
from datetime import datetime

MIN_MATCH_COUNT = 30

class CamError(Exception):

    def __init__(self, message):
        self.message = message

def compare_imgs(train_img, kp1, des1, query_img, sift, matcher, class_name, cam_id):
    
    pprint(('compare >', train_img.shape, query_img.shape))

    kp2, des2 = sift.detectAndCompute(train_img,None)
    matches = matcher.knnMatch(des1, des2, k=2)

    good = []
    for m,n in matches:
        if m.distance < 0.8*n.distance:
            good.append(m)
    res_coords = []

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = query_img.shape

        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts,M)
        res_coords = dst
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matchesMask, # draw only inliers
                        flags = 2)
        # img3 = cv.drawMatches(query_img,kp1,train_img,kp2,good,None,**draw_params)
        # img3 = draw_borders(img3, np.array(pts, dtype=np.int32))
        img3 = cv.polylines(train_img,[np.int32(dst)],True,255,3, cv.LINE_AA)

        # final_img = draw_borders(frame, np.array(res_all_coords, dtype=np.int32))
        # final_img = cv.resize(final_img, (1000,650))
        # cv.imshow("res", final_img)
        # if cv.waitKey(0):
        #     cv.destroyAllWindows()
        cv.imwrite(f'/tmp/{class_name}_{cam_id}.jpeg', img3)
        print(f'совпадение: {len(good)}; {class_name};')
    else:
        print( "мало кт - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None

    return res_coords, len(good)

def draw_borders(train_img, coords):
    train_img = cv.polylines(train_img,coords,True,255,1, cv.LINE_AA)
    return train_img

