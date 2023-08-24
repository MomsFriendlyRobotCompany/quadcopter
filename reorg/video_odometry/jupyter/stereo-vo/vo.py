
import numpy as np
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

import cv2
from opencv_camera import bgr2gray, gray2bgr


class Stereo:
    def __init__(self):
        block = 11
        #emperical values from P1, P2 as suggested in Ocv documentation
        P1 = block * block * 8
        P2 = block * block * 32

        self.disparityEngine = cv2.StereoSGBM.create(
            minDisparity=0,
            numDisparities=32, 
            blockSize=block, 
            P1=P1, 
            P2=P2
        )
        
        # thresh = dict(threshold=15, nonmaxSuppression=True);
        # self.detector = cv2.FastFeatureDetector.create(**thresh)
        
        # MAX_FEATURES = 1000
        # self.detector = cv2.ORB.create(MAX_FEATURES)
        
        self.fast = cv2.FastFeatureDetector.create()
        
        # self.matcher = cv2.DescriptorMatcher.create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
        
        lk_params = dict( 
            winSize = (15, 15),
            maxLevel = 3,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 0.03))
        
        
    def getDisp(self, imgL, imgR):
        disparity = self.disparityEngine.compute(imgL, imgR).astype(np.float32)
        disparityA = np.divide(disparity, 16.0)
        
        return disparityA
    
    def matchDisp(self, a, da, b, db):
        """
        a, ad - oldleft, disparity oldleft
        b, bd - left, disparity left
        """
        # fa = self.detector.detect(a)
        # fb = self.detector.detect(b)
        
        # p0, descriptors0 = self.detector.detectAndCompute(a, None) # mask = None
        # kp2, descriptors2 = self.detector.detectAndCompute(b, None)
        
        # params for ShiTomasi corner detection
        feature_params = dict( maxCorners = 500,
                               qualityLevel = 0.2,
                               minDistance = 1,
                               # blockSize = 7 
                             )
        p0 = cv2.goodFeaturesToTrack(a, mask = None, **feature_params)

#         # Match features.
#         # matcher = cv2.DescriptorMatcher.create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
#         matches = matcher.match(descriptors1, descriptors2, None) # query train
#         matches = list(matches)

#         # Sort matches by score
#         matches.sort(key=lambda x: x.distance, reverse=False)

#         # key = 12
#         # print(matches[key].distance)

#         # Remove not so good matches
#         GOOD_MATCH_PERCENT = 1.0
#         numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
#         matches = matches[:numGoodMatches]

        
        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize  = (15,15),  # size of the search window at each pyramid level
                          maxLevel = 2,   #  0, pyramids are not used (single level), 
                                          # if set to 1, two levels are used, and so on
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        p1, st, err = cv2.calcOpticalFlowPyrLK(a, b, p0, None, **lk_params)
        
        # Select good points
        good_old = p0[st==1]
        good_new = p1[st==1]
        
        # print([x.response for x in kp1])
        # print(matches)
        
        # fa = kp1[:matches]
        # fb = kp2[:matches]
        
        # print(type(p0), p0.shape)
        # print(type(p1), p1.shape)
        # print(type(da), da.shape)
        # print(type(good_old), good_old.shape)
        # print(good_old)
        
        # print(descriptors1.shape)
        # print(descriptors2.shape)
        
        
        # print(f"Found: {len(matches)}")
        # print(f"Distance imgIdx queryIdx trainIdx")
        # for m in matches:
        #     print(m.distance, m.imgIdx, m.queryIdx, m.trainIdx)
        
#         wa = [[v,u,da[v,u],1.] for u,v in good_old.astype(int)]
#         wa = np.array([x for x in wa if x[2] > 0])
        
#         wb = [[v,u,db[v,u],1.] for u,v in good_new.astype(int)]
#         wb = np.array([x for x in wb if x[2] > 0])
        
        wa, wb = [], []
        for aa, bb in zip(good_old, good_new):
            au,av = aa.astype(int)
            bu,bv = bb.astype(int)
            daa = da[av,au]
            dbb = db[bv,bu]
            
            Tx = 0.0311 # m
            cx = 200
            cy = 200
            f = 300
            if daa > 0.0 and dbb > 0.0:
                # wa.append([av,au,daa,1.0])
                # wb.append([bv,bu,dbb,1.0])
                h = -Tx/daa
                wa.append([h*(av-cx),h*(au-cy), -h*f])
                h = -Tx/dbb
                wb.append([h*(bv-cx),h*(bu-cy), -h*f])
        wa = np.array(wa)
        wb = np.array(wb)
        
        # print(wa)
        # print(wa.shape)
        # print(wb)
        # print(wb.shape)
        # print("")
        print(wb - wa)
        print("")
            
        # raise Exception
        
        return wa, wb