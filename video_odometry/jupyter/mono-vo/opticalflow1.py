# https://github.com/simondlevy/OpenCV-Python-Hacks/blob/master/optical_flow/__init__.py
import cv2
import numpy as np
from wernstrom.utils import rotz
from opencv_camera import bgr2gray, gray2bgr

import time
import math

class OpticalFlowCalculator:
    '''
    A class for optical flow calculations using OpenCV
    '''

    def __init__(self, 
                 # frame_width,
                 # frame_height,
                 # scaledown=1,
                 # perspective_angle=0,
                 # move_step=16,
                 # window_name=None,
                 draw_flow=False,
                 flow_color_rgb=(0, 255, 0)):
        '''
        Creates an OpticalFlow object for images with specified width, height.
        Optional inputs are:
          perspective_angle - perspective angle of camera, for reporting flow
                              in meters per second
          move_step         - step size in pixels for sampling the flow image
          window_name       - window name for display
          flow_color_rgb    - color for displaying flow
        '''

        # self.move_step = move_step
        self.mv_color_bgr = (flow_color_rgb[2],
                             flow_color_rgb[1],
                             flow_color_rgb[0])

        # self.perspective_angle = perspective_angle

        # self.window_name = window_name
        self.draw_flow = draw_flow
        
        self.flow_image = None

        # self.size = (int(frame_width/scaledown), int(frame_height/scaledown))

        self.prev_gray = None
        # self.prev_time = None

#     def processBytes(self, rgb_bytes, distance=None, timestep=1):
#         '''
#         Processes one frame of RGB bytes, returning summed X,Y flow.
#         Optional inputs are:
#           distance - distance in meters to image (focal length) for returning
#           flow in meters per second timestep - time step in seconds for
#           returning flow in meters per second
#          '''

#         frame = np.frombuffer(rgb_bytes, np.uint8)
#         frame = np.reshape(frame, (self.size[1], self.size[0], 3))
#         return self.processFrame(frame, distance, timestep)

    def processFrame(self, frame2, distance=None, timestep=1):
        '''
        Processes one image frame, returning summed X,Y flow and frame.
        Optional inputs are:
          distance - distance in meters to image (focal length) for returning
          flow in meters per second timestep - time step in seconds for
          returning flow in meters per second
        '''

        # frame2 = cv2.resize(frame, self.size)
        # frame2 = frame.copy()

        if len(frame2.shape) == 3:
            gray = bgr2gray(frame2)
        else:
            gray = frame2.copy()
            frame2 = gray2bgr(frame2)

        xsum, ysum = 0, 0
        xvel, yvel = 0, 0

        flow = None

        if self.prev_gray is None:
            self.prev_gray = gray.copy()
            
        # flow = cv2.calcOpticalFlowFarneback(self.prev_gray,
        #                                     gray,
        #                                     flow,
        #                                     pyr_scale=0.5,
        #                                     levels=5,
        #                                     winsize=13,
        #                                     iterations=10,
        #                                     poly_n=5,
        #                                     poly_sigma=1.1,
        #                                     flags=0)
        
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray,
            gray,
            flow,
            pyr_scale=0.5,
            levels=5,
            winsize=13,
            iterations=5,
            poly_n=5,
            poly_sigma=1.1,
            flags=0)

        # draw flow
        # for y in range(0, flow.shape[0], self.move_step):     # row
        #     for x in range(0, flow.shape[1], self.move_step): # col
        #         fx, fy = flow[y, x] # backwards???? row, col
        #         xsum += fx
        #         ysum += fy

#                 if self.draw_flow:
#                     cv2.line(frame2, (x, y), (int(x+fx), int(y+fy)), self.mv_color_bgr)
#                     cv2.circle(frame2, (x, y), 2, self.mv_color_bgr, -1)
#                     self.flow_image = frame2
        # print(flow.shape)
        # xsum,ysum = np.sum(np.sum(flow,axis=1),axis=0) #np.sum(flow, axis=1)
        # xsum,ysum = np.add.reduce(flow,axis=(1,0))
        
        # yy = list(range(0, flow.shape[0], self.move_step))
        # xx = list(range(0, flow.shape[1], self.move_step))
        # f = flow[yy,xx]
        
        move_step = 16
        f = flow[
                    0:flow.shape[0]:move_step, 
                    0:flow.shape[1]:move_step,
                    :
                ]
        # I think this is reversed, rows, cols = y, x
        ysum,xsum = np.add.reduce(f,axis=(1,0))
        
        # draw flow
        if self.draw_flow:
            for y in range(0, flow.shape[0], move_step):     # row
                for x in range(0, flow.shape[1], move_step): # col
                    fx, fy = flow[y, x] # backwards???? row, col

                    if self.draw_flow:
                        cv2.line(frame2, (x, y), (int(x+4*fx), int(y+4*fy)), self.mv_color_bgr)
                        cv2.circle(frame2, (x, y), 1, self.mv_color_bgr, -1)
                        self.flow_image = frame2
        
        
        # if xsum != xxsum or ysum != yysum:
        #     print(xsum, xxsum)
        #     print(ysum, yysum)
        #     # return None
        #     raise Exception

        # Default to system time if no timestep
        # curr_time = time.time()
        # if not timestep:
        #     timestep = ((curr_time - self.prev_time)
        #                 if self.prev_time else 1)
        # self.prev_time = curr_time
        timestep = 1

        count = (flow.shape[0] * flow.shape[1]) / move_step**2
        xvel = xsum / count / timestep
        yvel = ysum / count / timestep

        self.prev_gray = gray.copy()

        # if self.window_name:
        #     cv2.imshow(self.window_name, frame2)
        #     if cv2.waitKey(1) & 0x000000FF == 27:  # ESC
        #         return None

        # Return x,y velocities and new image with flow lines
        # return xvel, yvel, frame2
        return np.array([xvel, yvel])

#     def _get_velocity(self, count, sum_velocity_pixels, dimsize_pixels, timestep_seconds):

#         # count = (flow.shape[0] * flow.shape[1]) / self.move_step**2

#         avg_vel_px_per_sec = (sum_velocity_pixels / count / timestep_seconds)
        
#         return avg_vel_px_per_sec

        # return (self._velocity_meters_per_second(
        #             average_velocity_pixels_per_second,
        #             dimsize_pixels, distance_meters)
        #         if self.perspective_angle and distance_meters
        #         else average_velocity_pixels_per_second)
        
#         if self.perspective_angle and distance_meters:
#             vel = self._velocity_meters_per_second(
#                         average_velocity_pixels_per_second,
#                         dimsize_pixels, distance_meters)
#         else:
#             vel = average_velocity_pixels_per_second
            
#         return vel

#     def _velocity_meters_per_second(self, velocity_pixels_per_second,
#                                     dimsize_pixels, distance_meters):

#         distance_pixels = ((dimsize_pixels/2) /
#                            np.tan(self.perspective_angle/2))

#         pixels_per_meter = distance_pixels / distance_meters

#         return velocity_pixels_per_second / pixels_per_meter













class VideoOdometry:
    """
    VideoOdometry
    """
    def __init__(self, K=None, yaw=0):
        """
        Args:
            K: camera matrix
            yaw: initial starting heading
        """
        # self.K = K
        self.R_f = np.eye(3) #rotz(yaw)
        self.t_f = np.zeros((3,1))
        # self.detector = self.featureDetection()
        self.preFeature = None
        self.preImage = None
        self.find_features = 0
        self.avg_features = []
        self.flow_image = None
        self.deepF = cv2.optflow.createOptFlow_DeepFlow()
        self.errors = []
        self.error_count = 0
        
    def featureTracking(self, img_1, img_2, p1):
        lk_params = dict( 
            # winSize  = (21,21),
            winSize  = (35,35),
            maxLevel = 0,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        p2, st, err = cv2.calcOpticalFlowPyrLK(img_1, img_2, p1, None, **lk_params)
        st = st.reshape(st.shape[0])
        
        # find good ones
        newp1 = p1[st==1]
        newp2 = p2[st==1]

        return newp1, newp2
    
    def featureDetection(self):
        thresh = dict(threshold=25, nonmaxSuppression=True);
        fast = cv2.FastFeatureDetector_create(**thresh)
        return fast
    
    def draw(self, img, good_new, good_old):
        frame = gray2bgr(self.preImage)
        color = (0,255,0)
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            # if not (i % 20 == 0):
            #     continue
            
            # start - old location in frame
            a, b = old.ravel()
            a, b = int(a), int(b)
            
            # end - new location in frame
            # print(new.ravel(), " -> ", old.ravel())
            c, d = new.ravel()
            c, d = int(c), int(d)
            
            scale = 4
            da = scale*(c-a)
            db = scale*(d-b)
            cv2.line(frame, (a, b), (a+da, b+db), color, 1)
            cv2.circle(frame, (a, b), 2, color, -1)
        self.flow_image = frame
        
    def processFrame3(self, curImage, timestep=1):
        MIN_NUM_FEAT=100
        if self.preImage is None:
            self.preImage = curImage.copy()
            
        if (self.preFeature is None) or (len(self.preFeature) < MIN_NUM_FEAT):
            # feature = self.detector.detect(self.preImage)
            # # self.preFeature = np.array([e.pt for e in feature], dtype='float32')
            # self.preFeature = cv2.KeyPoint.convert(feature)
            feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
            self.preFeature = cv2.goodFeaturesToTrack(self.preImage, mask = None,
                             **feature_params)
            
        
        if (self.preFeature is None) or (len(self.preFeature) < MIN_NUM_FEAT):
            self.error_count += 1
            self.preImage = curImage
            return self.t_f.ravel()
            
        # fc = self.K[0,0]
        # pp = (self.K[0,2], self.K[1,2]) # (x,y)
        fc = 320 #self.K[0,0]
        w, h = curImage.shape[:2]
        pp = (h//2, w//2) #(self.K[0,2], self.K[1,2]) # (x,y)

        # kp1 = self.detector.detect(curImage);
        self.preFeature, curFeature = self.featureTracking(self.preImage, curImage, self.preFeature)
        
        # try:
        E, mask = cv2.findEssentialMat(curFeature, self.preFeature, fc, pp, cv2.RANSAC,0.999,1.0)
        if E is None:
            self.error_count += 1
            self.preImage = curImage
            return self.t_f.ravel()
            
        E = E[:3,:3]
        _, R, t, mask = cv2.recoverPose(E, curFeature, self.preFeature, focal=fc, pp = pp)
        # except:
        #     print(E)
        #     self.preImage = curImage
        #     # return self.t_f[:2]
        #     E = E[:3,:3]
       

        # absolute_scale = 1.0
        if True:
            self.draw(curImage, curFeature, self.preFeature)
        
        # t_f = None
        # R_f = None
        # t = t.ravel()
            
        # if absolute_scale > 0.1:
        # FIXME: y-axis down ... subtract looks better?
        # self.t_f = self.t_f - absolute_scale*self.R_f.dot(t)
        self.t_f = self.t_f - self.R_f.dot(t)
        self.R_f = R.dot(self.R_f)
        # else:
        #     print("crap ... bad scale:", absolute_scale)

        self.preImage = curImage
        self.preFeature = curFeature
        
        self.avg_features.append(len(curFeature))
        self.errors.append(self.error_count)
        
        # print(self.t_f)
        
        return self.t_f.ravel()
        
    def processFrame2(self, curImage):
        if self.preImage is None:
            self.preImage = curImage.copy()
            
        if (self.preFeature is None) or (len(self.preFeature) < 400):
            feature    = self.detector.detect(self.preImage)
            self.preFeature = np.array([e.pt for e in feature], dtype='float32')
            
        fc = 320 #self.K[0,0]
        w, h = curImage.shape[:2]
        pp = (h//2, w//2) #(self.K[0,2], self.K[1,2]) # (x,y)

        kp1 = self.detector.detect(curImage);
        self.preFeature, curFeature = self.featureTracking(self.preImage, curImage, self.preFeature)
        
        if True:
            self.draw(curImage, curFeature, self.preFeature)
            
        try:
            E, mask = cv2.findEssentialMat(curFeature, self.preFeature, fc, pp, cv2.RANSAC,0.999,1.0)
        except:
            print(E)
            self.preImage = curImage
            return self.t_f
        
        _, R, t, mask = cv2.recoverPose(E, curFeature, self.preFeature, focal=fc, pp = pp)

        absolute_scale = 1.0
            
        if absolute_scale > 0.1:
            # FIXME: y-axis down ... subtract looks better?
            self.t_f = self.t_f - absolute_scale*self.R_f.dot(t)
            self.R_f = R.dot(self.R_f)
        else:
            print("crap ... bad scale:", absolute_scale)

        self.preImage = curImage
        self.preFeature = curFeature
        
        # return self.R_f, self.t_f
        return self.t_f
    
    def processFrame(self, curImage, timestep=1):
        if self.preImage is None:
            self.preImage = curImage.copy()
            
        if (self.preFeature is None) or (len(self.preFeature) < 400):
            # print("--new features")
            self.find_features += 1
            feature = self.detector.detect(self.preImage)
            # self.preFeature = np.array([e.pt for e in feature], dtype='float32')
            self.preFeature = cv2.KeyPoint.convert(feature)
            # print(self.preFeature)
            
        preFeature, curFeature = self.featureTracking(self.preImage, curImage, self.preFeature)

        if True:
            self.draw(curImage, curFeature, preFeature)

        # calc avg change in pixels
        t = curFeature - preFeature
        a = np.mean(np.arctan2(t[:,1],t[:,0]))
        
        R = np.array([
            [np.cos(a), -np.sin(a)],
            [np.sin(a), np.cos(a)]
        ])
        t = np.mean(t,axis=0)
        t = R @ t
        
        self.avg_features.append(len(curFeature))

        self.preImage = curImage
        self.preFeature = curFeature
        
        return t #self.t_f.T.ravel()
    
    def calc_angl_n_transl(self,img, flow, step=8):
        '''
        input:
            - img - numpy array - image
            - flow - numpy array - optical flow
            - step - int - measurement of sparsity
        output:
            - angles - numpy array - array of angles of optical flow lines to the x-axis
            - translation - numpy array - array of length values for optical flow lines
            - lines - list - list of actual optical flow lines (where each line represents a trajectory of 
            a particular point in the image)
        '''

        angles = []
        translation = []

        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)

        for (x1, y1), (x2, y2) in lines:
            angle = math.atan2(- int(y2) + int(y1), int(x2) - int(x1)) #* 180.0 / np.pi
            length = math.hypot(int(x2) - int(x1), - int(y2) + int(y1))
            translation.append(length)
            angles.append(angle)

        return np.array(angles), np.array(translation) #, lines
    
    def estimate_motion(self,angles, translation):
        '''
        Input:
            - angles - numpy array - array of angles of optical flow lines to the x-axis
            - translation - numpy array - array of length values for optical flow lines
        Output:
            - ang_mode - float - mode of angles of trajectories. can be used to determine the direction of movement
            - transl_mode - float - mode of translation values 
            - ratio - float - shows how different values of translation are across a pair of frames. allows to 
            conclude about the type of movement
            - steady - bool - show if there is almost no movement on the video at the moment
        '''
        from scipy.stats import mode

        # Get indices of nonzero opical flow values. We'll use just them
        nonzero = np.where(translation > 0)

        # Whether non-zero value is close to zero or not. Should be set as a thershold
        steady = np.mean(translation) < 0.5

        translation = translation[nonzero]
        transl_mode = mode(translation)[0][0]

        angles = angles[nonzero]
        ang_mode = mode(angles)[0][0]

        # cutt off twenty percent of the sorted list from both sides to get rid off outliers
        ten_percent = len(translation) // 10
        translations = sorted(translation)
        translations = translations[ten_percent: len(translations) - ten_percent]

        # cluster optical flow values and find out how different these cluster are
        # big difference (i.e. big ratio value) corresponds to panning, otherwise - trucking
        inliers = [tuple([inlier]) for inlier in translations]
        k_means = KMeans(n_clusters=3, random_state=0).fit(inliers)
        centers = sorted(k_means.cluster_centers_)
        ratio = centers[0] / centers[-1]

        return ang_mode, transl_mode, ratio, steady
    
#     def processFrame2(self, frame):
#         frame2 = frame

#         if len(frame2.shape) == 3:
#             curImage = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
#         else:
#             curImage = frame2.copy()
#             frame2 = cv2.cvtColor(frame2, cv2.COLOR_GRAY2BGR)

#         xsum, ysum = 0, 0
#         xvel, yvel = 0, 0

#         flow = None

#         if self.preImage is None:
#             self.preImage = curImage.copy()
            
#         # flow = cv2.calcOpticalFlowFarneback(
#         #     self.preImage,
#         #     curImage,
#         #     flow,
#         #     pyr_scale=0.5,
#         #     levels=5,
#         #     winsize=13,
#         #     iterations=10,
#         #     poly_n=5,
#         #     poly_sigma=1.1,
#         #     flags=0)
        
#         flow = cv2.optflow.calcOpticalFlowSparseToDense(
#             self.preImage,
#             curImage,
#             grid_step=15, 
#             sigma=0.5)
        
#         a, t = self.calc_angl_n_transl(self.preImage, flow)
#         a,t,_,_ = self.estimate_motion(a,t)
        
#         # flow = self.deepF.calc(self.preImage, curImage, None)

#         # draw flow
#         self.move_step = 15
#         for y in range(0, flow.shape[0], self.move_step):     # row
#             for x in range(0, flow.shape[1], self.move_step): # col
#                 fx, fy = flow[y, x] # backwards???? row, col
#                 xsum += fx
#                 ysum += fy

#                 if True:
#                     self.flow_image = frame2
#                     color = (0,255,0)
#                     cv2.line(frame2, (x, y), (int(x+fx), int(y+fy)), color)
#                     cv2.circle(frame2, (x, y), 2, color, -1)

#         timestep = 1/30

#         count = (flow.shape[0] * flow.shape[1]) / self.move_step**2

#         xvel = (xsum / count / timestep)
#         yvel = (ysum / count / timestep)

#         self.preImage = curImage
#         self.avg_features.append(len(flow))

#         return np.array([xvel, yvel])
    
#     def processFrame(self, curImage, MIN_NUM_FEAT=50):
#         if self.preImage is None:
#             self.preImage = curImage.copy()
            
#         if (self.preFeature is None) or (len(self.preFeature) < MIN_NUM_FEAT):
#             feature = self.detector.detect(self.preImage)
#             self.preFeature = np.array([e.pt for e in feature], dtype='float32')
            
#         # fc = self.K[0,0]
#         # pp = (self.K[0,2], self.K[1,2]) # (x,y)
#         fc = 40
#         pp = (curImage.shape[0]//2,curImage.shape[1]//2)

#         kp1 = self.detector.detect(curImage);
#         self.preFeature, curFeature = self.featureTracking(self.preImage, curImage, self.preFeature)
#         E, mask = cv2.findEssentialMat(curFeature, self.preFeature, fc, pp, cv2.RANSAC,0.999,1.0)
#         _, R, t, mask = cv2.recoverPose(E, curFeature, self.preFeature, focal=fc, pp = pp)

#         absolute_scale = 1.0
            
#         if absolute_scale > 0.1:
#             # FIXME: y-axis down ... subtract looks better?
#             self.t_f = self.t_f - absolute_scale*self.R_f.dot(t)
#             self.R_f = R.dot(self.R_f)
#         else:
#             print("crap ... bad scale:", absolute_scale)

#         self.preImage = curImage
#         self.preFeature = curFeature
        
#         return self.t_f.T.ravel()