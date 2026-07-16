# Test D435i vision sensor
# 
# Usage example:
#       (Remotely connect to Real RBY1-A robot UPC)
#       Before use this code, Robot should be powered ON
#       If you want to terminate the test, Enter ESC button
#       python3 test_vision.py
#
# Copyright (c) 2026 Precision Mechatronics Laboratory, KAIST. All rights reserved.
# Author: Seungwon Yun


# =============================================================================
# Import
# =============================================================================
import sys
from pathlib import Path
# folder_a(parent folder)/folder_b - file.py(current file)  ->   0: folder_b / 1: folder_a
sys.path.append(str(Path(__file__).resolve().parents[1]))
import rby1_lib_ysw as ysw
import time
import cv2


# =============================================================================
# Code
# =============================================================================
ysw.setupLogging()

robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)
robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_ONLY)

print("[D435i Test] Starts")

### --- Select------------------------ ###
while True:
    answer = int(input("Select the number to test\n"
                       " 1. Show color and depth images\n"
                       " 2. Get [X,Y,Z] 3D position value(clicked point)\n"
    ))
    if answer not in [1,2]:
        print("Please enter valid number")
    else:
        break
### ---------------------------------- ###

d435i = ysw.D435i(rgb_on=True, depth_on=True, pixel=(640,480), fps=30)
d435i.initialize()

if (answer == 1):
    d435i.startPipeline()
    
    try:
        while True:
            color_frame, depth_frame = d435i.getRawFrames()
            color_image, depth_image = d435i.frameToImage(color_frame, depth_frame)
            depth_colorMap = d435i.convertDepthToColorMap(depth_image, alpha=0.03)
            if (d435i.rgb_on):
                d435i.imshow("color_image", color_image)
            if (d435i.depth_on):
                d435i.imshow("depth_image", depth_colorMap)
            key = d435i.waitKey(1)
            if key == 27:       # ESC key
                break
    finally:
        d435i.stopPipeline()
        d435i.destroyAllWindows()
        
elif (answer == 2):
    d435i.openWindowWithMouse("color_image")
    d435i.startPipeline()
    
    try:
        while True:
            color_frame, depth_frame = d435i.getRawFrames()

            if color_frame is None or depth_frame is None:
                continue

            color_image, depth_image = d435i.frameToImage(color_frame, depth_frame)
            depth_colorMap = d435i.convertDepthToColorMap(depth_image, alpha=0.03)

            if d435i.clicked_point is not None:
                u, v = d435i.clicked_point

                X, Y, Z = d435i.get3DPos(depth_frame, (u,v))

                cv2.circle(color_image, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(
                    color_image,
                    f"X:{X:.3f}, Y:{Y:.3f}, Z:{Z:.3f} m",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                print(f"Pixel=({u}, {v}), Camera 3D=({X:.3f}, {Y:.3f}, {Z:.3f}) m")

            d435i.imshow("color_image", color_image)
            d435i.imshow("depth_image", depth_colorMap)

            key = d435i.waitKey(1)
            if key == 27:       # ESC key
                break
    finally:
        d435i.stopPipeline()
        d435i.destroyAllWindows()
        

print("[D435i Test] Ends")