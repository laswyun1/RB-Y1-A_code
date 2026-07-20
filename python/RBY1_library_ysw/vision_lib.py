# Vision Depth Camera Library for RBY1-A
# This Library contains Intel Realsense D435i depth camera
# This should be imported in "rby1_lib_ysw.py"
# 
# Usage example:
#       from vision_lib import *
#
# Copyright (c) 2026 Precision Mechatronics Laboratory, KAIST. All rights reserved.
# Author: Seungwon Yun



# =============================================================================
# Info
# =============================================================================
"""
When viewed from the front of the D435i,
    Origin: camera's center
    Coordinate: 
        Right-to-Left = X-axis, 
        Top-to-Bottom = Y-axis,
        Orthogonal from camera = Z-axis (or Depth "z")  
"""


# =============================================================================
# Import
# =============================================================================
import rby1_sdk as rby
import robot_info as RI
import utils as UTIL
import logging
import numpy as np
import time
from typing import Union
import signal
import sys
import cv2
import pyrealsense2 as rs



# =============================================================================
# Code
# =============================================================================
class D435i:
    def __init__(self, rgb_on: bool=True, depth_on: bool=True, pixel: tuple=(640, 480), fps: int=30):
        self.rgb_on = rgb_on
        self.depth_on = depth_on
        self.pixel = pixel
        self.fps = fps
        
        self.pipeline = rs.pipeline()
        self.config = None
        self.align = None
        self.profile = None   
        self.is_started = False
        
        self.clicked_point = None
    
    
    def initialize(self, align_enable=True):
        self.config = rs.config()
        
        if (self.rgb_on):
            self.config.enable_stream(rs.stream.color, self.pixel[0], self.pixel[1], rs.format.bgr8, self.fps)
        if (self.depth_on):
            self.config.enable_stream(rs.stream.depth, self.pixel[0], self.pixel[1], rs.format.z16, self.fps)
        if (self.rgb_on and self.depth_on and align_enable):
            self.align = rs.align(rs.stream.color)
            
            
    def startPipeline(self):
        try:
            self.profile = self.pipeline.start(self.config)
            self.is_started = True
            logging.info("D435i pipeline started")
            return True
        
        except RuntimeError as e:
            logging.error(f"Failed to start D435i pipeline: {e}")
            self.is_started = False
            return False
        
        
    def stopPipeline(self):
        if self.is_started:
            self.pipeline.stop()
            self.is_started = False
    
    
    def getRawFrames(self, timeout_ms: int=10000):
        """
        Get raw frames.
        
        return:
            color_frame
            depth_frame
        """
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=timeout_ms)
        except RuntimeError as e:
            logging.error(f"Frame didn't arrive: {e}")
            return None, None

        if self.rgb_on and self.depth_on and self.align is not None:
            frames = self.align.process(frames)

        color_frame = None
        depth_frame = None

        if self.rgb_on:
            color_frame = frames.get_color_frame()
        if self.depth_on:
            depth_frame = frames.get_depth_frame()

        return color_frame, depth_frame
    
    
    def frameToImage(self, color_frame=None, depth_frame=None):
        """
        Get the pixel image once
        Before use this function, you need to execute "getRawFrames"
        
        return:
            color_data: (w,h,3) -> BGR sequence
            depth_data: (w,h,1)
        """
        color_image = None
        depth_image = None

        if color_frame is not None:
            color_image = np.asanyarray(color_frame.get_data())

        if depth_frame is not None:
            depth_image = np.asanyarray(depth_frame.get_data())

        return color_image, depth_image
            
            
    def convertDepthToColorMap(self, depth_image, alpha=0.3):
        """
        Make raw depth_image data be visible
        
        args:
            depth_image: 2nd output of function "getRawData" 
            alpha: transparency
            
        return:
            colored depth_image data 
        """
        depth_colorMap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=alpha),
            cv2.COLORMAP_JET
        )
        return depth_colorMap
        
        
    def imshow(self, name, image):
        """
        args:
            name (str)
            image (w,h,3) 
        """
        cv2.imshow(name, image)
        
    
    def waitKey(self, waiting_time):
        """
        Waiting keyboard input during "wainit_time"
        
        args:
            waiting_time: unit=[ms]
            
        return:
            key: keyboard input
        """
        key = cv2.waitKey(waiting_time)
        return key
    

    def destroyAllWindows(self):
        """
        Close all windows of OpenCV
        """
        cv2.destroyAllWindows()


    def getDepth(self, depth_frame, pixel=(0, 0)):
        """
        Get distance(or depth) in [m] unit
        
        args:
            depth_frame: 2nd value obtained from "getRawFrames"
            pixel: (u,v), u: left-to-right, v: top-to-bottom
            
        return:
            z = depth distance value [m]
        """
        u, v = pixel
        depth = depth_frame.get_distance(u, v)
        return depth
    
    
    def get3DPos(self, depth_frame, pixel=(0, 0)):
        """
        Calcuate 3D position(x,y,z) from pixel(u,v) and depth(z) information
        X = (u - ppx)/fx * Z
        Y = (v - ppy)/fy * Z
        Z = z
        
        args:
            depth_frame: 2nd value obtained from "getRawFrames"
            pixel: (u,v), u: left-to-right, v: top-to-bottom
            
        return:
            3D position: [X,Y,Z]
        """
        u, v = pixel    # pixel
        z = self.getDepth(depth_frame, pixel)    # depth
        
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        
        pos = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u,v], z)
    
        return pos  # [X,Y,Z]
    
    
    def openWindowWithMouse(self, image_name):
        """
        Open the image window including mouse left click interrupt
        """
        def mouse_leftClick_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.clicked_point = (x,y)
                print(f"Clicked pixel: u={x}, v={y}")
    
        cv2.namedWindow(image_name)
        cv2.setMouseCallback(image_name, mouse_leftClick_callback)
            
    
    def detectColorObjectContour(self, color_image, depth_frame, mask_list, minimum_area):
        """
        Detect colored objects using contour and calculate 2D bounding box.

        args:
            color_image: BGR image
            depth_frame: RealSense depth frame
            mask_list:
                ex)
                mask_list = [
                    (np.array([0, 80, 80]), np.array([10, 255, 255])),
                    (np.array([170, 80, 80]), np.array([180, 255, 255]))
                ]

            minimum_area: detect objects larger than this value

        return:
            detected_objects: list of detected object dicts
            color_image: image with visualization
            mask_total: binary mask
        """
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)      # Convert BGR to HSV
        
        # Masking (pixel in range->white(255), pixel out of range->black(0))
        mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        for lower, upper in mask_list:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = cv2.bitwise_or(mask_total, mask)       # Append
            
        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove outside small white noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # remove inside small black noise
        
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,          # Only find outmost contour
            cv2.CHAIN_APPROX_SIMPLE     # compress and save contour points 
        )
        
        detected_objects = []
        
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if area < minimum_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            u = x + w // 2
            v = y + h // 2
            
            X, Y, Z = self.get3DPos(depth_frame, [u, v])    
            
            # If depth measurment failed
            if Z <= 0:
                continue         

            detected_objects.append({
                "id": idx,
                "area": area,
                "bbox": [x, y, w, h],
                "pixel": [u, v],
                "pos": [X, Y, Z]
            })
            
            # Bounding Box
            cv2.rectangle(
                color_image,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )
        
            # Center point
            cv2.circle(
                color_image,
                (u, v),
                5,
                (0, 0, 255),
                -1
            )
            
            cv2.putText(
                color_image,
                f"id:{idx} Z:{Z:.3f}m",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
        
        cv2.putText(
            color_image,
            f"Detected red objects: {len(detected_objects)}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )
        
        return detected_objects, color_image, mask_total

    
    
    def detectColorObjectContour3D(self, color_image, depth_frame, mask_list, minimum_area, maximum_depth=None, sampling: int=3):
        """
        Detect colored objects using contour and calculate 3D bounding box.
        
        args:
            color_image
            depth_frame
            mask_list
            minimum_area
            maximum_depth:
                If not None, points with depth >= maximum_depth are ignored.(Ex. floor) 
                unit: [m]c=
            sampling:
                how many sample steps skip for reducing calculation load
                [Ex] If sampling=3, 1st sample, 4th sample, 7th sample... are used

        return:
            detected_objects
            color_image
            mask_total
        """

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)      # Convert BGR to HSV

        # Masking (pixel in range->white(255), pixel out of range->black(0))
        mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in mask_list:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = cv2.bitwise_or(mask_total, mask)   # Append

        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)   # remove outside small white noise
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)  # remove inside small black noise

        contours, _ = cv2.findContours(
            mask_total,
            cv2.RETR_EXTERNAL,          # Only find outmost contour
            cv2.CHAIN_APPROX_SIMPLE     # compress and save contour points 
        )

        detected_objects = []
        height, width = mask_total.shape

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < minimum_area:
                continue

            # Create mask for each object contour
            object_mask = np.zeros(mask_total.shape, dtype=np.uint8)
            cv2.drawContours(object_mask, [contour], -1, 255, thickness=-1)     # fill in the contour with white color

            # Get pixels inside contour
            ys, xs = np.where(object_mask == 255)

            points_3d = []

            # Sampling for fast calculation
            sample_step = sampling

            for u, v in zip(xs[::sample_step], ys[::sample_step]):
                if u < 0 or u >= width or v < 0 or v >= height:
                    continue
                
                depth = depth_frame.get_distance(int(u), int(v))
                if depth <= 0:
                    continue

                X, Y, Z = self.get3DPos(depth_frame, [u, v])    

                points_3d.append([X, Y, Z])

            if len(points_3d) == 0:
                continue

            points_3d = np.array(points_3d)

            # 3D center position (Mean)
            center_3d = np.mean(points_3d, axis=0)

            # 3D center position (Median -> recommended)
            center_3d_median = np.median(points_3d, axis=0)
            
            # object 중심 depth가 maximum_depth 이상이면 object 자체를 무시(Ex. floor)
            if maximum_depth is not None and center_3d_median[2] >= maximum_depth:
                continue


            # Axis-aligned 3D bounding box
            min_3d = np.min(points_3d, axis=0)
            max_3d = np.max(points_3d, axis=0)
            size_3d = max_3d - min_3d

            Xc, Yc, Zc = center_3d_median

            object_id = len(detected_objects)

            # 2D contour centroid for drawing
            M = cv2.moments(contour)

            if M["m00"] != 0:       # If 3D bbox is allowable
                u_center = int(M["m10"] / M["m00"])
                v_center = int(M["m01"] / M["m00"])
            else:                   # If 3D bbox is not allowable
                x, y, w, h = cv2.boundingRect(contour)
                u_center = x + w // 2
                v_center = y + h // 2

            detected_objects.append({
                "id": object_id,
                "area_2d": area,
                "pixel_center": [u_center, v_center],
                "points_3d": points_3d,
                "center_3d": center_3d_median.tolist(),
                "bbox_3d_min": min_3d.tolist(),
                "bbox_3d_max": max_3d.tolist(),
                "bbox_3d_size": size_3d.tolist(),
            })

            # Draw contour, not just 2D bbox
            cv2.drawContours(color_image, [contour], -1, (255, 0, 0), 2)

            cv2.circle(
                color_image,
                (u_center, v_center),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                color_image,
                f"id:{object_id} center=({Xc:.3f},{Yc:.3f},{Zc:.3f})",
                (u_center + 10, v_center - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                2
            )

        cv2.putText(
            color_image,
            f"Detected objects: {len(detected_objects)}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        return detected_objects, color_image, mask_total
    
    
    









def average_detectColorObjectContour3D(samples):
    """
    Average the samples obtained by "detectColorObjectContour3D" function
    """

    if len(samples) == 0:
        return None

    area_2d_list = np.array([s["area_2d"] for s in samples])
    pixel_center_list = np.array([s["pixel_center"] for s in samples])
    center_3d_list = np.array([s["center_3d"] for s in samples])
    bbox_3d_size_list = np.array([s["bbox_3d_size"] for s in samples])
    bbox_3d_min_list = np.array([s["bbox_3d_min"] for s in samples])
    bbox_3d_max_list = np.array([s["bbox_3d_max"] for s in samples])

    result = {
        "num_samples": len(samples),

        "area_2d_avg": float(np.mean(area_2d_list)),
        "area_2d_median": float(np.median(area_2d_list)),

        "pixel_center_avg": np.mean(pixel_center_list, axis=0),
        "pixel_center_median": np.median(pixel_center_list, axis=0),

        "center_3d_avg": np.mean(center_3d_list, axis=0),
        "center_3d_median": np.median(center_3d_list, axis=0),

        "bbox_3d_size_avg": np.mean(bbox_3d_size_list, axis=0),
        "bbox_3d_size_median": np.median(bbox_3d_size_list, axis=0),

        "bbox_3d_min_avg": np.mean(bbox_3d_min_list, axis=0),
        "bbox_3d_min_median": np.median(bbox_3d_min_list, axis=0),

        "bbox_3d_max_avg": np.mean(bbox_3d_max_list, axis=0),
        "bbox_3d_max_median": np.median(bbox_3d_max_list, axis=0),
    }

    return result


    
 
            
def convertPos_vision_to_linkhead2(pos_vision):
    """
    This function converts 3D position value from vision camera frame to "link_head_2" frame
    
    args:
        3D position based on D435i frame: pos_vision = np.array([pos_x, pos_y, pos_z])
        
    return:
        3D position based on "link_head_2" frame
    """
    pos_vision = np.asarray(pos_vision, dtype=np.float64)
    
    pos_x = pos_vision[0]
    pos_y = pos_vision[1]
    pos_z = pos_vision[2]
    
    X = pos_z
    Y = -pos_x + RI.D435I_X_OFFSET_FROM_D435I_CENTER
    Z = -pos_y + RI.D435I_Z_OFFSET_FROM_LINK_HEAD_2
    
    return np.array([X, Y, Z], dtype=np.float64)



def calculateEEpos_forGrasp(object_pos_from_vision, object_3Dsize, manual_offset: float=0):
    """
    This function calculates the position of final end-effector(NOT "ee_finger_r1" or "ee_finger_l1") based on "link_head_2"
    It is only applicable if the end-effector orientation(rpy) is matched with "link_head_2" frame
        [Ex: Right EE]
            link_head_2 frame => Rz(+90) -> Rx(-90) => Right EE frame
            EE frame x = link_head_2 frame y
            EE frame y = link_head_2 frame -z
            EE frame z = link_head_2 frame -x
        [Ex: Left EE]
            link_head_2 frame => Rx(+90) -> Ry(-90) => Left EE frame
            EE frame x = link_head_2 frame -y
            EE frame y = link_head_2 frame z
            EE frame z = link_head_2 frame -x
    
    args:
        object_pos_from_vision: object's 3D center position obtained by D435i
        object_3Dsize: object's 3D size (w,h,d)
        manual_offset: 
            Manual offset for fine tuning(theoretically it should be zero)
            For pick&place demo, I use 0.03m 
            direction: same as proceeding direction of arm or end-effector
            (+): go forward slightly more to proceeding direction
            (-): backward
    
    return:
        object_pos_from_linkhead2_to_ee_before_open_gripper: 
            3D position of end-effector(Not "ee_finger_l1/r1") relative to "link_head_2" before open the gripper
        object_pos_from_linkhead2_to_ee_after_open_gripper: 
            3D position of end-effector(Not "ee_finger_l1/r1") relative to "link_head_2" after open the gripper (final destination)
    """
    object_pos_from_linkhead2 = convertPos_vision_to_linkhead2(object_pos_from_vision)
    
    object_maxDim = max(object_3Dsize)
    
    # small distanted offset from object's nearest part
    x_offset = object_maxDim / 2 + RI.GRIPPER_OFFSET_BEFORE_GRASP
    
    # "link_head_2" to "ee_finger_l1/r1" position considering finger length
    object_pos_from_linkhead2_to_ee_before_open_gripper = object_pos_from_linkhead2 + [-x_offset , 0, 0]
    object_pos_from_linkhead2_to_ee_after_open_gripper  = object_pos_from_linkhead2 + [+manual_offset, 0, 0]
    
    return object_pos_from_linkhead2_to_ee_before_open_gripper, object_pos_from_linkhead2_to_ee_after_open_gripper
            
    

def calculateEEpos_forPlace(box_pos_from_vision, box_3Dsize, object_3Dsize, pos, head_y_angle):
    """
    This function calculates the position of "ee_finger_r1" or "ee_finger_l1" based on "link_head_2"
    It is only applicable if the end-effector orientation(rpy) is matched with "link_head_2" frame
        [Ex: Right EE]
            link_head_2 frame => Rz(+90) -> Rx(-90) => Right EE frame
            EE frame x = link_head_2 frame y
            EE frame y = link_head_2 frame -z
            EE frame z = link_head_2 frame -x
        [Ex: Left EE]
            link_head_2 frame => Rx(+90) -> Ry(-90) => Left EE frame
            EE frame x = link_head_2 frame -y
            EE frame y = link_head_2 frame z
            EE frame z = link_head_2 frame -x
    
    args:
        box_pos_from_vision: box's 3D center position obtained by D435i
        box_3Dsize: object's 3D size (w,h,d)
        pos: get_FK(robot, reference_link="link_head_2", target_link="ee_finger_11/r1")["position"]
        rpy: get_FK(robot, reference_link="link_head_2", target_link="ee_finger_11/r1")["rpy"]
        
    return:
        box_pos_from_linkhead2_to_ee_before_open_gripper: 3D position of "ee_finger_l1/r1" relative to "link_head_2" before open the gripper
        box_pos_from_linkhead2_to_ee_after_open_gripper: 3D position of "ee_finger_l1/r1" relative to "link_head_2" after open the gripper (final destination)
    """
    box_pos_from_linkhead2 = convertPos_vision_to_linkhead2(box_pos_from_vision)
    
    box_maxDim = max(box_3Dsize)
    object_maxDim = max(object_3Dsize)
    
    # small distanted offset from object's nearest part
    z_offset_box = box_maxDim / 2
    
    # total distance from ee_finger_l1/r1 frame to object's end
    pos_from_ee_finger_to_object = np.array([0, 0, RI.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER - object_maxDim])
    
    
    
    
    
    
    z_offset_ee = RI.GRIPPER_OFFSET_BEFORE_GRASP
    
    # "link_head_2" to "ee_finger_l1/r1" position considering finger length
    object_pos_from_linkhead2_to_ee_before_open_gripper = object_pos_from_linkhead2 + [RI.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER -x_offset , 0, 0]
    object_pos_from_linkhead2_to_ee_after_open_gripper  = object_pos_from_linkhead2 + [RI.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER , 0, 0]
    
    return object_pos_from_linkhead2_to_ee_before_open_gripper, object_pos_from_linkhead2_to_ee_after_open_gripper  