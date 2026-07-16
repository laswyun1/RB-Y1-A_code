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
        Detect colored objects using contour.

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

    
    
    
    
    
    
    
    