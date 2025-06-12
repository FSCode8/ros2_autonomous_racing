from VisionCalculation import *

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import numpy as np
import torch
import cv2

import math

class LaneDetector():
    def __init__(self, vision_obj=None, model_path='./fastai_model.pth'):
        self.vo = vision_obj
        self.grid = self.vo.grid_coordinates
        self.cut_v = self.vo.cut_v

        if torch.cuda.is_available():
            self.device = "cuda"
            torch.cuda.empty_cache()
            self.model = torch.load(model_path, weights_only=False).to(self.device)
        else:
            self.model = torch.load(model_path, weights_only=False, map_location=torch.device("cpu"))
            self.device = "cpu"
        self.model.eval()

    def create_path(self, image, time_stamp):
        left_poly, right_poly, _, _ = self.get_fit_and_probs(image)

        distances = [0.7, 1.2, 1.8]

        path_msg = Path()
        path_msg.header.stamp = time_stamp
        path_msg.header.frame_id = "odom"

        # Create the path
        path_msg.poses = []
        for i, dist in enumerate(distances):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = "odom"

            left_y = left_poly(dist)
            right_y = right_poly(dist)
            
            # smooth the drive if neccesarry
            if (left_y == 0 and right_y != 0) or (left_y != 0 and right_y == 0): # XOR - only one is missing
                division_val = 4.0/3.0
            else:
                division_val = 2.0

            middle_y = (left_y + right_y)/division_val


            pose.pose.position.x = dist
            pose.pose.position.y = middle_y 
            pose.pose.position.z = 0.0
            
            if i > 0:
                dx = pose.pose.position.x - path_msg.poses[i-1].pose.position.x
                dy = pose.pose.position.y - path_msg.poses[i-1].pose.position.y
                  
            else:
                dx = pose.pose.position.x 
                dy = pose.pose.position.y
            
            # Calculate yaw angle
            yaw = math.atan2(dy, dx)

            pose.pose.orientation.w = yaw  # No rotation
            path_msg.poses.append(pose)

        return path_msg
        

    def get_fit_and_probs(self, img):
        _, left, right = self.detect(img)
        left_poly = self.fit_poly(left)
        right_poly = self.fit_poly(right)
        return left_poly, right_poly, left, right

    def _predict(self, img):
        with torch.no_grad():
            image_tensor = img.transpose(2,0,1).astype('float32')/255
            x_tensor = torch.from_numpy(image_tensor).to(self.device).unsqueeze(0)
            model_output = torch.softmax(self.model.forward(x_tensor), dim=1).cpu().numpy()
        return model_output

    def detect(self, img_array):
        model_output = self._predict(img_array)
        class_map = np.argmax(model_output[0], axis=0)  # Shape: (960, 1280)

        # Convert to uint8 for display (scale class IDs to visible range)
        class_map_vis = (class_map * 255 // (model_output.shape[1] - 1)).astype(np.uint8)

        cv2.imshow('Segmentation Classes', class_map_vis)
        cv2.waitKey(1)

        background, left, right = model_output[0,0,:,:], model_output[0,1,:,:], model_output[0,2,:,:] 
        return background, left, right

    def fit_poly(self, probs):
        min_points_for_fit = 15
        poly_deg = 3

        probs_flat = np.ravel(probs[self.cut_v:, :])
        mask = probs_flat > 0.3
        if mask.sum() > min_points_for_fit:
            coeffs = np.polyfit(self.grid[:,0][mask], self.grid[:,1][mask], deg=3, w=probs_flat[mask])
        else:
            coeffs = np.array([0.,0.,0.,0.])

        return np.poly1d(coeffs)

    def create_path_oldschool(self, image, time_stamp):
        # region of interest is one pixel under the middle for pitch angle = 0  
        h, w = image.shape[:2]
        cx, cy = w // 2, h // 2
        x1, y1 = 0, h // 2 
        x2, y2 = w, h 
        roi = image[y1:y2, x1:x2]

        h_e, w_e, _ = roi.shape

        # convert image, apply the Canny Edge Detection and find the contours to get the lane markings
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges = cv2.Canny(blurred, 150, 200)

        # paint over car part
        cv2.rectangle(edges, ((w_e//2)-(w_e//4), h_e-20), ((w_e//2)+(w_e//4), h_e-1), (0,0,0), -1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Find the contours which start closest to the bottom middle of the image
        # They are most likely to be the lane markings of interest
        h_e, w_e = edges.shape
        target = np.array([w_e // 2, h_e - 1])
        target_l = np.array([target[0]-((target[0]//4)*3), target[1]-50])
        target_r = np.array([target[0]+((target[0]//4)*3), target[1]-50])

        left_min = float('inf')
        right_min = float('inf')
        left_cnt, left_point = None, None
        right_cnt, right_point = None, None

        y_max_point_array = []
        for cnt in contours:
            idx = np.argmax(cnt[:, 0, 1])
            y_max_point = cnt[idx, 0, :]
            y_max_point_array.append(y_max_point)

            if len(cnt)>100 and y_max_point[1]>target[1]//3: # filter out small or wrongcontours
                if y_max_point[0] < target[0]:
                    left_dist = np.linalg.norm(target_l-y_max_point)
                    if left_dist < left_min:
                        left_min = left_dist
                        left_point = y_max_point
                        left_cnt = cnt
                else:
                    right_dist = np.linalg.norm(target_r-y_max_point)
                    if right_dist < right_min:
                        right_min = right_dist
                        right_point = y_max_point
                        right_cnt = cnt

        # setup the arrays with the x values of the left and right lane
        if left_point is None and right_point is None:
            x_left = -1
            x_right = -1
        else:
            if left_point is None: 
                x_left = -1
            else:
                xl, yl = left_cnt[:, 0, 0], left_cnt[:, 0, 1]

                idx_left = np.argmin(np.abs(yl - target[1]))     # Find closest index in yl to target y
                x_left = xl[idx_left]   # Get corresponding x positions

            if right_point is None:
                x_right = -1
            else: 
                xr, yr = right_cnt[:, 0, 0], right_cnt[:, 0, 1]

                idx_right = np.argmin(np.abs(yr - target[1]))    # Find closest index in yr to target y
                x_right = xr[idx_right]     # Get corresponding x positions

        # Create points
        current_position = [0.0, 0.0, 0.0]  # X, Y, Z in world frame

        path_msg = Path()
        path_msg.header.stamp = time_stamp
        path_msg.header.frame_id = "odom"

        contours_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if left_cnt is not None:
            cv2.drawContours(contours_img, [left_cnt], -1, (0,255,0), thickness=2)
        if right_cnt is not None:
            cv2.drawContours(contours_img, [right_cnt], -1, (0,0,255), thickness=2)

        # Create the path
        path_msg.poses = []
        for i in range(6):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = "odom"

            pixel_y = h_e - (i+2) * 60  

            # NOTE: Rotation is not considered yet 

            if x_left == -1 and x_right == -1:
                pixel_x = int(w_e / 2.0)  # Default to center if both sides are missing
            elif x_left == -1:
                pixel_x = 0
            elif x_right == -1:
                pixel_x = int(w_e - 1)

            else:
                idx_left = np.argmin(np.abs(yl - pixel_y))     # Find closest index in yl to y_target
                x_left = xl[idx_left]   # Get corresponding x positions

                idx_right = np.argmin(np.abs(yr - pixel_y))    # Find closest index in yr to y_target
                x_right = xr[idx_right]     # Get corresponding x positions

                pixel_x = int((x_left + x_right) / 2.0)

            cv2.circle(contours_img, (pixel_x, pixel_y), 5, (0, 255, 0), -1)

            pose.pose.position.x = current_position[0] + self.vo.grid_coordinates[y1 + pixel_y, x1 + pixel_x][0]  # x coordinate in world frame
            pose.pose.position.y = current_position[1] + self.vo.grid_coordinates[y1 + pixel_y, x1 + pixel_x][1]  # y coordinate in world frame 
            pose.pose.position.z = current_position[2]  # z coordinate in world frame
            
            if i> 0:
                dx = pose.pose.position.x - path_msg.poses[i-1].pose.position.x
                dy = pose.pose.position.y - path_msg.poses[i-1].pose.position.y
                  
            else:
                dx = pose.pose.position.x - current_position[0]
                dy = pose.pose.position.y - current_position[1]
            
            # Calculate yaw angle
            yaw = math.atan2(dy, dx)

            pose.pose.orientation.w = yaw  # No rotation
            path_msg.poses.append(pose)

        cv2.imshow("Contours", contours_img)
        cv2.waitKey(1)

        return path_msg