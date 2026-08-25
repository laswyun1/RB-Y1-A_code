"""
    This is the Demo File for 260826

    [Scenario]
    (0) In 1st Floor Laboratory, the robot move forward to Table 

    (1) Black Box is in the Left side, Red Box is in the Right side on the table
        - Box location can be changed
        
    (2) 4 Black Objects(25x25x25) and 4 Red Objects(25x25x25) are located between two Boxes
        - Objects location and number can be changed

    (3) Check the Locations of "Black Box" and all "Black Objects" -> calculate IK

    (4) Pick the [Black objects] (near the center has lower index=higher priority to pick) using the Black Box located arm 
        - Ex: If Black Box is in the left side of robot, using left arm
        
    (5) Move the picking arm to Black Box and lay the Black Objects
        
    (6) Likewise, Do same works for [Red Box and Red Objects]
        
    (7) Robot grasp the [Black Box's outer side]
    
    (8) Robot grasp the [Red Box's outer side]
    
    (9) Robot goes box-transfer ready posture

    (10) The robot slightly move backward, and turn right 180 degrees

    (11) Check the shelf distance by vision camera
    
    (12) Move forward to shelf and Lay two Boxes 
    
    
    
    **********************************************************
        position control이나 mobile 관련 함수 실행 중 에러나서 멈추면,   
        - python3 poweroff.py                               
        - python3 initialize.py 
        실행 후 demoFile_260826.py 실행                        
    ***********************************************************
"""

# ==========================================================================================================
# Import Libraries
# ==========================================================================================================
import rby1_sdk as rby
import numpy as np
import logging
import time
import pyrealsense2 as rs
import cv2

import sys
from pathlib import Path
# Path.cwd()           : folder "260826", 
# Path.cwd().parents[0]: folder "Demonstration", 
# Path.cwd().parents[1]: folder "RBY1_library_ysw"
sys.path.append(str(Path.cwd().parents[1]))                  
import rby1_lib_ysw as ysw

np.set_printoptions(suppress=True, precision=4, linewidth=300)  

# In .ipynb, show logging 
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
    # format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
    force=True,
)




# ==========================================================================================================
# Function Definitions
# ==========================================================================================================

# Vision-based Colored Box Detection 
def check_colored_box_and_set_grasp_arm(
    depthCamObj: ysw.RealSenseDepthCamera,
    boxColor,
    hsv_mask,
    record_duration,
    box_minimum_area,
    box_maximum_distance,
    pixel_roi_x=None,
    pixel_roi_y=None,
    verbose=True,
):
    """
    Detect one colored box for a certain duration and decide grasp arm.

    Args:
        depthCamObj:
            RealSenseDepthCamera object
            Ex) [RealSenseDepthCamera] Class

        boxColor:
            string for print/result label
            Ex) "black", "red", "green"

        hsv_mask:
            HSV mask list
            Ex) ysw.HSV_MASK_BLACK_BOX, ysw.HSV_MASK_RED_BOX

        record_duration:
            recording duration [sec]

        box_minimum_area:
            minimum contour area [pixel^2]

        box_maximum_distance:
            maximum depth distance [m]

        pixel_roi_x/y:
            ROI range in pixel coordinate
            Ex) pixel_roi_x = [x_min, x_max]
                pixel_roi_y = [y_min, y_max]

    Return:
        result = {
            "boxColor": boxColor,
            "box_samples": box_samples,
            "box_sample_avg": box_sample_avg,
            "grasp_arm": grasp_arm,
        }
    """

    # ============================================================
    # Get the Information about Colored Box
    # ============================================================
    box_samples = []

    rv = depthCamObj.startPipeline()
    if rv is False:
        raise RuntimeError("Failed to start pipeline.")

    try:
        start_time = time.time()

        while True:
            elapsed_time = time.time() - start_time

            if elapsed_time >= record_duration:
                break

            color_frame, depth_frame = depthCamObj.getRawFrames()

            if color_frame is None or depth_frame is None:
                continue

            color_image, depth_image = depthCamObj.frameToImage(color_frame, depth_frame)

            if color_image is None or depth_image is None:
                continue

            # Detection용 원본 이미지 보존
            color_image_raw = color_image.copy()

            detected_boxes, color_image_forBox, mask_box = depthCamObj.detectColorObjectContour3D(
                color_image=color_image_raw.copy(),
                depth_frame=depth_frame,
                mask_list=hsv_mask,
                minimum_area=box_minimum_area,
                maximum_depth=box_maximum_distance,
                pixel_roi_x=pixel_roi_x,
                pixel_roi_y=pixel_roi_y,
            )

            if len(detected_boxes) > 0:
                largest_box = max(detected_boxes, key=lambda obj: obj["area_2d"])
                
                # [Largest area] sample
                box_samples.append({
                    "area_2d":      float(largest_box["area_2d"]),
                    "pixel_center": np.array(largest_box["pixel_center"], dtype=np.float64),
                    "center_3d":    np.array(largest_box["center_3d"], dtype=np.float64),
                    "bbox_3d_size": np.array(largest_box["bbox_3d_size"], dtype=np.float64),
                    "bbox_3d_min":  np.array(largest_box["bbox_3d_min"], dtype=np.float64),
                    "bbox_3d_max":  np.array(largest_box["bbox_3d_max"], dtype=np.float64),
                })

    finally:
        depthCamObj.stopPipeline()
        depthCamObj.destroyAllWindows()


    # =========================
    # Average Box
    # =========================
    box_sample_avg = None

    if len(box_samples) > 0:
        box_sample_avg = ysw.average_detectColorObjectContour3D(box_samples)


    # =========================
    # Set grasp arm
    # =========================
    grasp_arm = None

    if box_sample_avg is not None:
        if box_sample_avg["center_3d_avg"][0] > 0:
            grasp_arm = "right"
        else:
            grasp_arm = "left"


    # =========================
    # Print Result
    # =========================
    if verbose:
        print(f"\n========== {boxColor.capitalize()} Box Detection Result ==========")

        if box_sample_avg is None:
            print(f"No {boxColor.capitalize()} Box detected.")
        else:
            print(f"\n[{boxColor.capitalize()} Box]")
            print(f"num_samples             : {box_sample_avg['num_samples']}")
            print(f"area_2d_avg             : {box_sample_avg['area_2d_avg']:.1f}")
            print(f"area_2d_median          : {box_sample_avg['area_2d_median']:.1f}")
            print(f"center_3d_avg           : {np.round(box_sample_avg['center_3d_avg'], 4)} m")
            print(f"center_3d_median        : {np.round(box_sample_avg['center_3d_median'], 4)} m")
            print(f"bbox_3d_size_avg        : {np.round(box_sample_avg['bbox_3d_size_avg'], 4)} m")
            print(f"bbox_3d_size_median     : {np.round(box_sample_avg['bbox_3d_size_median'], 4)} m")

        print(f"\ngrasp_arm_{boxColor}Obj: {grasp_arm}")

    result = {
        "boxColor": boxColor,
        "box_samples": box_samples,
        "box_sample_avg": box_sample_avg,
        "grasp_arm": grasp_arm,
    }
    return result



# Vision-based Colored Objects Detection (sort: near the center = lower index)
def check_colored_objects_sorted_by_center(
    depthCamObj: ysw.RealSenseDepthCamera,
    objectColor,
    hsv_mask,
    record_duration,
    object_minimum_area,
    object_maximum_area=None,
    maximum_depth=None,
    pixel_roi_x=None,
    pixel_roi_y=None,
    max_match_distance_pixel=40.0,   # Cluster option
    min_rank_sample_ratio=0.4,       # Sample filtering option
    min_rank_num_samples=10,
    verbose=True,
):
    """
    Detect all objects of one color for a certain duration.
    Objects are clustered by "pixel_center" after recording to neglect noise or wrong samplesd.
    Final object ranks are sorted by distance from image/ROI center.

    The nearest object from image/ROI center has rank 0.

    Args:
        depthCamObj:
            RealSenseDepthCamera object

        objectColor:
            string for print/result label
            Ex) "black", "red", "green"

        hsv_mask:
            HSV mask list
            Ex) ysw.HSV_MASK_BLACK_BOX, ysw.HSV_MASK_RED

        record_duration:
            recording duration [sec]

        object_minimum_area:
            minimum contour area [pixel^2]

        object_maximum_area:
            maximum contour area [pixel^2]

        maximum_depth:
            maximum depth distance [m]

        pixel_roi_x/y:
            ROI range in pixel coordinate
            Ex) pixel_roi_x = [x_min, x_max]
                pixel_roi_y = [y_min, y_max]
            If None, whole image range is used.

        max_match_distance_pixel:
            Pixel distance threshold for clustering objects.
            If two detections are closer than this value, they are treated as same object.

        min_rank_sample_ratio:
            Ignore clusters whose sample count is smaller than
            max_sample_count * min_rank_sample_ratio.

        min_rank_num_samples:
            Ignore clusters whose sample count is smaller than this value.

    Return:
        result = {
            "objectColor": objectColor,
            "all_object_samples": all_object_samples,
            "object_clusters": object_clusters,
            "valid_object_clusters": valid_object_clusters_sorted,
            "ignored_object_cluster_info": ignored_object_cluster_info,
            "object_sample_avg_list": object_sample_avg_list,
            "max_sample_count": max_sample_count,
        }
    """

    # ============================================================
    # Helper: get image or ROI center pixel
    # ============================================================
    def get_roi_center_pixel(width, height, pixel_roi_x=None, pixel_roi_y=None):
        if pixel_roi_x is None:
            x_min = 0
            x_max = width
        else:
            x_min, x_max = pixel_roi_x
            x_min = int(np.clip(x_min, 0, width - 1))
            x_max = int(np.clip(x_max, 0, width))

        if pixel_roi_y is None:
            y_min = 0
            y_max = height
        else:
            y_min, y_max = pixel_roi_y
            y_min = int(np.clip(y_min, 0, height - 1))
            y_max = int(np.clip(y_max, 0, height))

        u_center = (x_min + x_max) // 2
        v_center = (y_min + y_max) // 2

        return np.array([u_center, v_center], dtype=np.float64)


    # ============================================================
    # Helper: sample dictionary
    # ============================================================
    def make_object_sample(obj, frame_idx, image_center_pixel):
        pixel_center = np.array(obj["pixel_center"], dtype=np.float64)

        center_distance_pixel = float(
            np.linalg.norm(pixel_center - image_center_pixel)
        )

        sample = {
            "frame_idx": frame_idx,
            "center_distance_pixel": center_distance_pixel,
            "area_2d":      float(obj["area_2d"]),
            "pixel_center": pixel_center,
            "center_3d":    np.array(obj["center_3d"], dtype=np.float64),
            "bbox_3d_size": np.array(obj["bbox_3d_size"], dtype=np.float64),
            "bbox_3d_min":  np.array(obj["bbox_3d_min"], dtype=np.float64),
            "bbox_3d_max":  np.array(obj["bbox_3d_max"], dtype=np.float64),
        }

        return sample


    # ============================================================
    # Helper: cluster samples by pixel center
    # ============================================================
    def cluster_samples_by_pixel_center(
        samples,
        max_match_distance_pixel=40.0,
    ):
        """
        Group object samples by pixel_center proximity.
        This is a simple greedy clustering method.

        Static objects 기준:
            - object가 고정되어 있고
            - 카메라도 고정되어 있고
            - pixel 위치가 크게 흔들리지 않는 경우
        에 적합함.
        """

        clusters = []

        for sample in samples:
            p = np.asarray(sample["pixel_center"], dtype=np.float64)

            best_cluster_idx = None
            best_dist = None

            for i, cluster in enumerate(clusters):
                cluster_center = cluster["pixel_center_mean"]
                dist = np.linalg.norm(p - cluster_center)

                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_cluster_idx = i

            # ----------------------------------------------------
            # Add to existing cluster
            # ----------------------------------------------------
            if best_cluster_idx is not None and best_dist <= max_match_distance_pixel:
                cluster = clusters[best_cluster_idx]
                cluster["samples"].append(sample)

                pixel_centers = np.array(
                    [s["pixel_center"] for s in cluster["samples"]],
                    dtype=np.float64
                )

                cluster["pixel_center_mean"] = np.mean(pixel_centers, axis=0)
                cluster["pixel_center_median"] = np.median(pixel_centers, axis=0)

            # ----------------------------------------------------
            # Create new cluster
            # ----------------------------------------------------
            else:
                clusters.append({
                    "samples": [sample],
                    "pixel_center_mean": p.copy(),
                    "pixel_center_median": p.copy(),
                })

        return clusters


    # ============================================================
    # 1. Get colored objects from camera
    # ============================================================
    all_object_samples = []
    frame_idx = 0
    image_center_pixel_ref = None

    rv = depthCamObj.startPipeline()
    if rv is False:
        raise RuntimeError("Failed to start pipeline.")

    try:
        start_time = time.time()

        while True:
            elapsed_time = time.time() - start_time

            if elapsed_time >= record_duration:
                break

            color_frame, depth_frame = depthCamObj.getRawFrames()

            if color_frame is None or depth_frame is None:
                continue

            color_image, depth_image = depthCamObj.frameToImage(
                color_frame,
                depth_frame
            )

            if color_image is None or depth_image is None:
                continue

            height, width = color_image.shape[:2]

            image_center_pixel = get_roi_center_pixel(
                width=width,
                height=height,
                pixel_roi_x=pixel_roi_x,
                pixel_roi_y=pixel_roi_y,
            )

            image_center_pixel_ref = image_center_pixel.copy()

            # Detection용 원본 이미지 보존
            color_image_raw = color_image.copy()

            detected_objects, color_image_forObj, mask_obj = depthCamObj.detectColorObjectContour3D(
                color_image=color_image_raw.copy(),
                depth_frame=depth_frame,
                mask_list=hsv_mask,
                minimum_area=object_minimum_area,
                maximum_area=object_maximum_area,
                maximum_depth=maximum_depth,
                pixel_roi_x=pixel_roi_x,
                pixel_roi_y=pixel_roi_y,
            )

            if len(detected_objects) == 0:
                frame_idx += 1
                continue

            # ----------------------------------------------------
            # Save all detections without rank
            # rank는 여기서 정하지 않음
            # ----------------------------------------------------
            for obj in detected_objects:
                sample = make_object_sample(
                    obj=obj,
                    frame_idx=frame_idx,
                    image_center_pixel=image_center_pixel,
                )

                all_object_samples.append(sample)

            frame_idx += 1

    finally:
        depthCamObj.stopPipeline()
        depthCamObj.destroyAllWindows()


    # ============================================================
    # 2. Cluster all samples after recording
    # ============================================================
    object_clusters = cluster_samples_by_pixel_center(
        samples=all_object_samples,
        max_match_distance_pixel=max_match_distance_pixel,
    )


    # ============================================================
    # 3. Filter unstable clusters
    # ============================================================
    valid_object_clusters = []
    ignored_object_cluster_info = []

    if len(object_clusters) > 0:
        max_sample_count = max(
            len(cluster["samples"]) for cluster in object_clusters
        )
    else:
        max_sample_count = 0

    for cluster_idx, cluster in enumerate(object_clusters):
        samples = cluster["samples"]
        sample_count = len(samples)

        # [Ignore case 1]
        if sample_count < min_rank_num_samples:
            ignored_object_cluster_info.append({
                "cluster_idx": cluster_idx,
                "num_samples": sample_count,
                "reason": f"num_samples < min_rank_num_samples({min_rank_num_samples})",
            })
            continue

        if max_sample_count > 0:
            sample_ratio = sample_count / max_sample_count
        else:
            sample_ratio = 0.0

        # [Ignore case 2]
        if sample_ratio < min_rank_sample_ratio:
            ignored_object_cluster_info.append({
                "cluster_idx": cluster_idx,
                "num_samples": sample_count,
                "sample_ratio": sample_ratio,
                "reason": f"sample_ratio < min_rank_sample_ratio({min_rank_sample_ratio})",
            })
            continue

        # For valid clusters
        cluster["cluster_idx"] = cluster_idx
        cluster["num_samples"] = sample_count
        cluster["sample_ratio"] = sample_ratio

        valid_object_clusters.append(cluster)


    # ============================================================
    # 4. Sort valid clusters by distance from image/ROI center
    # ============================================================
    if image_center_pixel_ref is None:
        # No valid frame was acquired
        valid_object_clusters_sorted = []
    else:
        valid_object_clusters_sorted = sorted(
            valid_object_clusters,
            key=lambda cluster: np.linalg.norm(
                cluster["pixel_center_mean"] - image_center_pixel_ref
            )
        )


    # ============================================================
    # 5. Average objects
    # ============================================================
    object_sample_avg_list = []

    for rank_idx, cluster in enumerate(valid_object_clusters_sorted):
        samples = cluster["samples"]

        sample_avg = ysw.average_detectColorObjectContour3D(samples)

        # rank/object info
        sample_avg["rank"] = rank_idx
        sample_avg["objectColor"] = objectColor

        # cluster info
        sample_avg["cluster_idx"] = cluster["cluster_idx"]
        sample_avg["num_samples"] = cluster["num_samples"]
        sample_avg["sample_ratio"] = cluster["sample_ratio"]
        sample_avg["pixel_center_mean"] = cluster["pixel_center_mean"]
        sample_avg["pixel_center_median"] = cluster["pixel_center_median"]

        # center distance avg/median
        center_distances = np.array(
            [sample["center_distance_pixel"] for sample in samples],
            dtype=np.float64
        )

        sample_avg["center_distance_pixel_avg"] = float(
            np.mean(center_distances)
        )
        sample_avg["center_distance_pixel_median"] = float(
            np.median(center_distances)
        )

        object_sample_avg_list.append(sample_avg)


    # ============================================================
    # 6. Print result
    # ============================================================
    if verbose:
        print(f"\n========== {objectColor.capitalize()} Object Detection Result ==========")
        print("Objects are clustered by pixel_center after recording.")
        print("Final ranks are sorted by distance from image/ROI center.\n")

        print(f"Total raw detections     : {len(all_object_samples)}")
        print(f"Total clusters           : {len(object_clusters)}")
        print(f"Valid clusters           : {len(valid_object_clusters_sorted)}")
        print(f"Ignored clusters         : {len(ignored_object_cluster_info)}")
        print(f"Max sample count         : {max_sample_count}")
        print(f"Cluster distance thresh  : {max_match_distance_pixel:.1f} pixel")
        print(f"Sample filter            : ratio >= {min_rank_sample_ratio}, "
              f"num_samples >= {min_rank_num_samples}")

        if len(object_sample_avg_list) == 0:
            print(f"\nNo valid {objectColor.capitalize()} Object detected.")

        else:
            for sample_avg in object_sample_avg_list:
                rank_idx = sample_avg["rank"]

                print(f"\n[{objectColor.capitalize()} Object rank {rank_idx}]")
                print(f"cluster_idx                  : {sample_avg['cluster_idx']}")
                print(f"num_samples                  : {sample_avg['num_samples']}")
                print(f"sample_ratio                 : {sample_avg['sample_ratio']:.2f}")
                print(f"area_2d_avg                  : {sample_avg['area_2d_avg']:.1f}")
                print(f"area_2d_median               : {sample_avg['area_2d_median']:.1f}")
                print(f"center_distance_pixel_avg    : {sample_avg['center_distance_pixel_avg']:.1f}")
                print(f"center_distance_pixel_median : {sample_avg['center_distance_pixel_median']:.1f}")
                print(f"pixel_center_mean            : {np.round(sample_avg['pixel_center_mean'], 1)}")
                print(f"pixel_center_median          : {np.round(sample_avg['pixel_center_median'], 1)}")
                print(f"center_3d_avg                : {np.round(sample_avg['center_3d_avg'], 4)} m")
                print(f"center_3d_median             : {np.round(sample_avg['center_3d_median'], 4)} m")
                print(f"bbox_3d_size_avg             : {np.round(sample_avg['bbox_3d_size_avg'], 4)} m")
                print(f"bbox_3d_size_median          : {np.round(sample_avg['bbox_3d_size_median'], 4)} m")

        if len(ignored_object_cluster_info) > 0:
            print("\n[Ignored clusters]")
            for info in ignored_object_cluster_info:
                print(info)


    # ============================================================
    # 7. Return result
    # ============================================================
    result = {
        "objectColor": objectColor,

        # raw samples
        "all_object_samples": all_object_samples,

        # clusters
        "object_clusters": object_clusters,
        "valid_object_clusters": valid_object_clusters_sorted,
        "ignored_object_cluster_info": ignored_object_cluster_info,

        # final ranked object avg list [Important !!]
        "object_sample_avg_list": object_sample_avg_list,

        # parameters/info
        "max_sample_count": max_sample_count,
        "max_match_distance_pixel": max_match_distance_pixel,
        "min_rank_sample_ratio": min_rank_sample_ratio,
        "min_rank_num_samples": min_rank_num_samples,
        "image_center_pixel_ref": image_center_pixel_ref,
    }

    return result



# Calculate IK for colored box and objects
def compute_IK_for_colored_object_and_box(
    robot,
    objectColor,
    object_sample_avg,
    boxColor,
    box_sample_avg,
    grasp_arm,
    object_manual_offset_downToGrasp=-0.008,   # can be manually changed (theoretically it should be zero)
    object_pregrasp_z_offset=+0.1,             # can be manually changed 
    object_grasp_x_offset=0.0,                 # can be manually changed (theoretically it should be zero)
    box_pregrasp_z_offset=0.05,      # can be manually changed, {N1}: before opening gripper, above box outer side-top [m]
    box_grasp_z_offset=-0.01,        # can be manually changed, {N2}: after opening gripper, down to  box outer side-top [m]
    box_grasp_x_offset=-0.04,        # can be manually changed, To grasp front part of outer-side of box
    box_grasp_y_offset=0.05,         # can be manually changed, absolute value (theoretically it should be zero)
    use_median=False,
    gripper_z_offset=ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER,
    verbose=True,
):
    """
    Compute target transforms for one colored object and one collecting box.

    Frame notation:
        {V}: vision frame
        {H}: link_head_2 frame
        {S}: base frame
        {M}: object goal frame
        {M1}: object goal frame before opening gripper
        {M2}: object goal frame after opening gripper
        {N}: collecting box goal frame
        {N1}: collecting box (outer side to grasp) goal frame before opening gripper
        {N2}: collecting box (outer side to grasp) goal frame after opening gripper
        {C}: ee_finger_l1/r1 frame
        {D}: final fingertip frame, same orientation as {C}

    Args:
        objectColor:
            Ex) "black", "red", "green"

        object_sample_avg:
            output of average_detectColorObjectContour3D() for object

        boxColor:
            Ex) "black", "red", "green"

        box_sample_avg:
            output of average_detectColorObjectContour3D() for collecting box

        grasp_arm:
            "right" or "left"

        use_median:
            False -> use center_3d_avg / bbox_3d_size_avg
            True  -> use center_3d_median / bbox_3d_size_median

    Return:
        result dict
    """

    # ------------------------------------------------------------
    # 0. Basic check
    # ------------------------------------------------------------
    if object_sample_avg is None:
        raise ValueError(f"{objectColor} object_sample_avg is None.")

    if box_sample_avg is None:
        raise ValueError(f"{boxColor} box_sample_avg is None.")

    if grasp_arm not in ["right", "left"]:
        raise ValueError(f"Invalid grasp_arm: {grasp_arm}")

    center_key = "center_3d_median" if use_median else "center_3d_avg"
    size_key = "bbox_3d_size_median" if use_median else "bbox_3d_size_avg"

    # ------------------------------------------------------------
    # 1. Get object / box positions in vision frame {V}
    # ------------------------------------------------------------
    object_pos = np.asarray(
        object_sample_avg[center_key],
        dtype=np.float64
    )

    object_3Dsize = np.asarray(
        object_sample_avg[size_key],
        dtype=np.float64
    )

    box_pos = np.asarray(
        box_sample_avg[center_key],
        dtype=np.float64
    )

    box_3Dsize = np.asarray(
        box_sample_avg[size_key],
        dtype=np.float64
    )

    # ------------------------------------------------------------
    # 2. Convert the position based on {V} to the position based on {H}
    # ------------------------------------------------------------
    P_HM_object = ysw.convertPos_vision_to_linkhead2(object_pos)
    P_HN_box = ysw.convertPos_vision_to_linkhead2(box_pos)

    # ------------------------------------------------------------
    # 3. Get T_SH: base -> link_head_2
    # ------------------------------------------------------------
    fk_SH = ysw.get_FK(robot, "base", "link_head_2")

    if fk_SH is None:
        raise RuntimeError("Failed to get FK: base -> link_head_2")

    T_SH = np.asarray(fk_SH["T"], dtype=np.float64)
    R_SH = np.asarray(fk_SH["rotation"], dtype=np.float64)
    P_SH = np.asarray(fk_SH["position"], dtype=np.float64)

    # ------------------------------------------------------------
    # 4. Convert the position based on {H} to the position based on {S}
    # ------------------------------------------------------------
    def transform_posVector(T, P):
        P_ext = np.append(np.asarray(P, dtype=np.float64), 1.0)
        P_out = T @ P_ext
        return P_out[:3]

    P_SM_object = transform_posVector(T_SH, P_HM_object)
    P_SN_box = transform_posVector(T_SH, P_HN_box)

    # ------------------------------------------------------------
    # 5. Desired gripper orientation (You can Change This!!)
    # ------------------------------------------------------------
    def get_desired_gripper_rotation(grasp_arm):
        if grasp_arm == "right":
            R1 = ysw.rotMat_z(90.0 * ysw.DEG2RAD)
            R2 = ysw.rotMat_x(-40.0 * ysw.DEG2RAD)
            return R1 @ R2

        elif grasp_arm == "left":
            R1 = ysw.rotMat_z(-90.0 * ysw.DEG2RAD)
            R2 = ysw.rotMat_x(40.0 * ysw.DEG2RAD)
            return R1 @ R2

        else:
            raise ValueError(f"Invalid grasp_arm: {grasp_arm}")

    R_SC_desired = get_desired_gripper_rotation(grasp_arm)

    # ------------------------------------------------------------
    # 6-1. Box center target transform {N}
    # ------------------------------------------------------------
    R_SN_box = R_SC_desired
    T_SN_box = ysw.make_HT(R_SN_box, P_SN_box)

    # ------------------------------------------------------------
    # 6-2. Box outer-side target transform {N1}, {N2}
    # ------------------------------------------------------------
    #   base x direction ≈ vision z size
    #   base y direction ≈ vision x size
    #   base z direction ≈ vision y size
    box_size_x_approx = box_3Dsize[2]
    box_size_y_approx = box_3Dsize[0]
    box_size_z_approx = box_3Dsize[1]

    half_x = 0.5 * box_size_x_approx
    half_y = 0.5 * box_size_y_approx
    half_z = 0.5 * box_size_z_approx
    
    if grasp_arm == "right":
        box_side_y_sign = -1.0
        box_side_y_offset = box_grasp_y_offset
    elif grasp_arm == "left":
        box_side_y_sign = +1.0
        box_side_y_offset = -box_grasp_y_offset
            
    # {N_side_top}: box side edge/corner 기준점, box top surface
    P_SN_side_top = P_SN_box + np.array([0.0, box_side_y_sign * half_y + box_side_y_offset, half_z], dtype=np.float64)
    
    # {N1}: before opening gripper, box outer-side 위쪽
    P_SN1_box = P_SN_side_top + np.array([box_grasp_x_offset, 0.0, box_pregrasp_z_offset], dtype=np.float64)

    # {N2}: after opening gripper, N1 아래로 살짝 내려온 위치
    P_SN2_box = P_SN_side_top + np.array([box_grasp_x_offset, 0.0, box_grasp_z_offset], dtype=np.float64)
            

    R_SN1_box = R_SC_desired
    R_SN2_box = R_SC_desired

    T_SN1_box = ysw.make_HT(R_SN1_box, P_SN1_box)
    T_SN2_box = ysw.make_HT(R_SN2_box, P_SN2_box)
    
    # ------------------------------------------------------------
    # 7. Object pre-grasp / grasp target transforms
    # ------------------------------------------------------------
    P_SM1_object = P_SM_object + np.array([0.0, 0.0, object_pregrasp_z_offset])
    R_SM1_object = R_SC_desired
    T_SM1_object = ysw.make_HT(R_SM1_object, P_SM1_object)

    P_SM2_object = P_SM_object + np.array([object_grasp_x_offset, 0.0, object_manual_offset_downToGrasp])
    R_SM2_object = R_SC_desired
    T_SM2_object = ysw.make_HT(R_SM2_object, P_SM2_object)

    # ------------------------------------------------------------
    # 8. ee_finger frame {C} -> fingertip frame {D}
    # ------------------------------------------------------------
    P_CD = np.array([0.0, 0.0, gripper_z_offset], dtype=np.float64)
    T_CD = ysw.make_HT(np.eye(3), P_CD)
    T_DC = np.linalg.inv(T_CD)

    # Target of {C} = target of {D} * inv(T_CD)
    T_SC_target_before_open_gripper = T_SM1_object @ T_DC
    T_SC_target_after_open_gripper = T_SM2_object @ T_DC
    
    # Box center target
    T_SC_target_for_box = T_SN_box @ T_DC
    
    # Box side targets
    T_SC_target_for_box_side_before_open_gripper = T_SN1_box @ T_DC
    T_SC_target_for_box_side_after_open_gripper = T_SN2_box @ T_DC

    # ------------------------------------------------------------
    # 9. Print
    # ------------------------------------------------------------
    if verbose:
        print(f"======== For {objectColor.capitalize()} Object ========")
        print(f"Before open gripper: \n{T_SC_target_before_open_gripper}\n")
        print(f"After open gripper: \n{T_SC_target_after_open_gripper}\n")
        print(f"Grasping Arm: {grasp_arm}\n")

        print(f"======== For {boxColor.capitalize()} Box ========")
        print(f"For {boxColor.capitalize()} Box: \n{T_SC_target_for_box}\n")
        
        print(f"======== For {boxColor.capitalize()} Box Side {{N1}}, {{N2}} ========")
        print(f"\nBefore opening gripper {{N1}}: \n{T_SC_target_for_box_side_before_open_gripper}\n")
        print(f"After opening gripper {{N2}}: \n{T_SC_target_for_box_side_after_open_gripper}\n")

    # ------------------------------------------------------------
    # 10. Return all useful values
    # ------------------------------------------------------------
    result = {
        # labels
        "objectColor": objectColor,
        "boxColor": boxColor,
        "grasp_arm": grasp_arm,

        # vision frame values
        "object_pos_V": object_pos,
        "box_pos_V": box_pos,

        "object_3Dsize": object_3Dsize,
        "box_3Dsize": box_3Dsize,

        # head frame values
        "P_HM_object": P_HM_object,
        "P_HN_box": P_HN_box,

        # base frame values
        "T_SH": T_SH,
        "R_SH": R_SH,
        "P_SH": P_SH,

        "P_SM_object": P_SM_object,
        "P_SN_box": P_SN_box,
        
        # box side frame values
        "P_SN1_box": P_SN1_box,
        "P_SN2_box": P_SN2_box,

        "box_size_x_approx": box_size_x_approx,
        "box_size_y_approx": box_size_y_approx,
        "box_size_z_approx": box_size_z_approx,

        "box_pregrasp_z_offset": box_pregrasp_z_offset,
        "box_grasp_z_offset": box_grasp_z_offset,

        # desired rotation
        "R_SC_desired": R_SC_desired,

        # intermediate target transforms
        "T_SM1_object": T_SM1_object,
        "T_SM2_object": T_SM2_object,
        "T_SN_box": T_SN_box,
        "T_SN1_box": T_SN1_box,
        "T_SN2_box": T_SN2_box,

        # fingertip offset
        "P_CD": P_CD,
        "T_CD": T_CD,

        # final ee_finger target transforms
        "T_SC_target_before_open_gripper": T_SC_target_before_open_gripper,
        "T_SC_target_after_open_gripper": T_SC_target_after_open_gripper,

        # final ee_finger target transforms for box center
        "T_SC_target_for_box": T_SC_target_for_box,
        
        # final ee_finger target transforms for box side
        "T_SC_target_for_box_side_before_open_gripper": T_SC_target_for_box_side_before_open_gripper,
        "T_SC_target_for_box_side_after_open_gripper": T_SC_target_for_box_side_after_open_gripper,
    }

    return result



# Select used-arm
def get_target_ee(grasp_arm):
    if grasp_arm == "right":
        return "ee_finger_r1"
    elif grasp_arm == "left":
        return "ee_finger_l1"
    else:
        raise ValueError(f"Invalid grasp_arm: {grasp_arm}")

# Select used-gripper
def get_target_gripper_id(grasp_arm):
    if grasp_arm == "right":
        return ysw.GRIPPER_ID_R
    elif grasp_arm == "left":
        return ysw.GRIPPER_ID_L
    else:
        raise ValueError(f"Invalid grasp_arm: {grasp_arm}")  
    
# Fix some torso joints for stability
def set_fixed_joint_targets(robot):
    """
        Fix current torso_0 ~ torso_3 joint positions.
        It can be changed by USER
    """
    current_joint_position = ysw.getState(robot, verbose=False).position

    fixed_joint_targets = {
        "torso_0": current_joint_position[ysw.ALL_JOINT_INDEX["torso_0"]],
        "torso_1": current_joint_position[ysw.ALL_JOINT_INDEX["torso_1"]],
        "torso_2": current_joint_position[ysw.ALL_JOINT_INDEX["torso_2"]],
        "torso_3": current_joint_position[ysw.ALL_JOINT_INDEX["torso_3"]],
    }

    return fixed_joint_targets

# Basic Position control using T
def positionCtrl_ee_by_T(
    robot,
    T_SC_target,
    grasp_arm,
    minimum_time=8.5,
    control_hold_time=0.5,
    fix_torso=True,
    get_return=True,
):
    """
        Move selected ee finger frame to target transform.

        Args:
            T_SC_target:
                4x4 transform of ee_finger_l1/r1 target w.r.t. base frame.
            grasp_arm:
                "right" or "left"
    """

    T_SC_target = np.asarray(T_SC_target, dtype=np.float64)

    pos = T_SC_target[:3, 3].copy()
    ori = ysw.rotMat_to_rpy(T_SC_target[:3, :3])

    target_ee_link = get_target_ee(grasp_arm)

    cartesian_position_targets_sample = [
        {
            "reference_link": "base",
            "target_link": target_ee_link,
            "pos": pos,
            "rpy": ori,
        },
    ]

    if fix_torso:
        joint_targets = set_fixed_joint_targets(robot)
    else:
        joint_targets = None

    rv = ysw.positionCtrl_cartesian_wholeBody(
        robot,
        position_targets=cartesian_position_targets_sample,
        joint_targets=joint_targets,
        minimum_time=minimum_time,
        control_hold_time=control_hold_time,
        get_return=get_return,
    )
    return rv


# 위치만 바꾸고 orientation은 기존 T에서 가져오는 함수
def positionCtrl_by_pos_and_T_orientation(
    robot,
    pos,
    T_SC_orientation_source,
    grasp_arm,
    minimum_time=8.5,
    control_hold_time=0.5,
    fix_torso=True,
    get_return=True,
):
    """
        Move selected ee finger frame to given position,
        while maintaining orientation from T_SC_orientation_source.
    """

    T_SC_orientation_source = np.asarray(T_SC_orientation_source, dtype=np.float64)

    pos = np.asarray(pos, dtype=np.float64).copy()
    ori = ysw.rotMat_to_rpy(T_SC_orientation_source[:3, :3])

    target_link = get_target_ee(grasp_arm)

    cartesian_position_targets_sample = [
        {
            "reference_link": "base",
            "target_link": target_link,
            "pos": pos,
            "rpy": ori,
        },
    ]

    if fix_torso:
        joint_targets = set_fixed_joint_targets(robot)
    else:
        joint_targets = None

    rv = ysw.positionCtrl_cartesian_wholeBody(
        robot,
        position_targets=cartesian_position_targets_sample,
        joint_targets=joint_targets,
        minimum_time=minimum_time,
        control_hold_time=control_hold_time,
        get_return=get_return,
    )
    return rv
    
    
# Calculate the gripper opening distance
def compute_desired_gripper_open_pos_mm(
    object_3Dsize,
    offset_gripper_open_m,
    max_margin_mm=8.0,
):
    """
        Compute desired gripper opening width from object size.
    """

    object_3Dsize = np.asarray(object_3Dsize, dtype=np.float64)

    object_meanDim = np.mean(object_3Dsize)                  # [m]
    desiredGripperPos = object_meanDim + offset_gripper_open_m # [m]
    desiredGripperPos_mm = desiredGripperPos * 1000.0        # [mm]

    if desiredGripperPos_mm >= ysw.GRIPPER_POS_RANGE_LINEAR:
        desiredGripperPos_mm = ysw.GRIPPER_POS_RANGE_LINEAR - max_margin_mm

    return desiredGripperPos_mm


# Gripper Open by position control
def open_gripper_for_grasp(
    gripper,
    grasp_arm,
    desiredGripperPos_mm,
    current_ampere_mag=0.02,
    sleep_sec=1.0,
    disable_after_open=True,
):
    """
        Open selected gripper using current-based position control.
        이게 가끔 안열리는 경우가 있어서, safety 추가
    """

    dxl_id = get_target_gripper_id(grasp_arm)
    
    # # Manual safety checker
    # while gripper.disable(dxl_id) is False:
    #     time.sleep(0.1)
        
    gripper.initialize_setting(dxl_id=dxl_id, operation_mode=ysw.DXL_CURRENT_BASED_POSITION_CONTROL_MODE)
        
    while (gripper.current_based_position_control_linear(dxl_id, current_ampere_mag=current_ampere_mag, linear_pos=desiredGripperPos_mm) != True):
        logging.warning(f"[DXL ID:{dxl_id}] Retry gripper open command")
        time.sleep(0.1)
        
    time.sleep(sleep_sec)

    # if disable_after_open:
    #     while gripper.disable(dxl_id) is False:
    #         time.sleep(0.1)


# Gripper Close by current control
def close_gripper_for_grasp(
    gripper,
    grasp_arm,
    grasp_current_obj,
    sleep_sec=1.0,
):
    """
        Close selected gripper using current control.
    """

    dxl_id = get_target_gripper_id(grasp_arm)

    gripper.initialize_setting(dxl_id=dxl_id, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
    gripper.write_current_ampere(dxl_id, grasp_current_obj)
    time.sleep(sleep_sec)
    
    
# Pick one Object and place it to Box
def pick_object_and_place_to_box(
    robot,
    gripper,
    target_result,
    offset_gripper_open_m,
    grasp_current_obj,
    object_floating_z_afterGrasp,
    position_control_minimum_time=8.5,
    control_hold_time=0.5,
    fix_torso=True,
    verbose=True,
):
    """
    Pick one object and place it into the target box using target_result.

    target_result should be output of "compute_IK_for_colored_object_and_box()".

    Required keys in target_result:
        "grasp_arm"
        "object_3Dsize"
        "T_SC_target_before_open_gripper"
        "T_SC_target_after_open_gripper"
        "T_SC_target_for_box"
    """

    if target_result is None:
        print("[ERROR] target_result is None.")
        return False

    grasp_arm = target_result["grasp_arm"]
    objectColor = target_result.get("objectColor", "object")
    boxColor = target_result.get("boxColor", "box")

    object_3Dsize = target_result["object_3Dsize"]

    T_SC_target_before_open_gripper = np.asarray(
        target_result["T_SC_target_before_open_gripper"],
        dtype=np.float64,
    )

    T_SC_target_after_open_gripper = np.asarray(
        target_result["T_SC_target_after_open_gripper"],
        dtype=np.float64,
    )

    T_SC_target_for_box = np.asarray(
        target_result["T_SC_target_for_box"],
        dtype=np.float64,
    )

    # ============================================================
    # 1. Move to pre-grasp pose
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print(f"# Move to Pre-grasp pose: {objectColor}")
        print("# ======================================")

    rv = positionCtrl_ee_by_T(
        robot=robot,
        T_SC_target=T_SC_target_before_open_gripper,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.6,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to move to pre-grasp pose: {objectColor}")
        return False

    # ============================================================
    # 2. Open gripper
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print("# Open Gripper")
        print("# ======================================")

    desiredGripperPos_mm = compute_desired_gripper_open_pos_mm(
        object_3Dsize=object_3Dsize,
        offset_gripper_open_m=offset_gripper_open_m,
    )

    if verbose:
        print(f"desiredGripperPos_mm: {desiredGripperPos_mm:.2f} mm")

    open_gripper_for_grasp(
        gripper=gripper,
        grasp_arm=grasp_arm,
        desiredGripperPos_mm=desiredGripperPos_mm,
        current_ampere_mag=0.02,
        sleep_sec=1.5,
        disable_after_open=True,
    )

    # ============================================================
    # 3. Move to grasp pose
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print(f"# Move to Grasp pose: {objectColor}")
        print("# ======================================")

    rv = positionCtrl_ee_by_T(
        robot=robot,
        T_SC_target=T_SC_target_after_open_gripper,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.3,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to move to grasp pose: {objectColor}")
        return False

    # ============================================================
    # 4. Close gripper
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print("# Grasp")
        print("# ======================================")

    close_gripper_for_grasp(
        gripper=gripper,
        grasp_arm=grasp_arm,
        grasp_current_obj=grasp_current_obj,
        sleep_sec=1.0
    )

    # ============================================================
    # 5. Slightly float the object
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print("# Slightly float the object")
        print("# ======================================")

    pos_after_open_gripper = T_SC_target_after_open_gripper[:3, 3].copy()
    pos_floating = pos_after_open_gripper + np.array([0.0, 0.0, object_floating_z_afterGrasp], dtype=np.float64)

    rv = positionCtrl_by_pos_and_T_orientation(
        robot=robot,
        pos=pos_floating,
        T_SC_orientation_source=T_SC_target_after_open_gripper,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.3,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to float the object: {objectColor}")
        return False

    # ============================================================
    # 6. Move to box while keeping floating height
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print(f"# Move to {boxColor} Box")
        print("# ======================================")

    pos_box = T_SC_target_for_box[:3, 3].copy()
    pos_box[2] = pos_floating[2]   # floating 높이 유지

    rv = positionCtrl_by_pos_and_T_orientation(
        robot=robot,
        pos=pos_box,
        T_SC_orientation_source=T_SC_target_for_box,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.6,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to move to {boxColor} box.")
        return False

    # ============================================================
    # 7. Open gripper
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print("# Open Gripper")
        print("# ======================================")

    open_gripper_for_grasp(
        gripper=gripper,
        grasp_arm=grasp_arm,
        desiredGripperPos_mm=desiredGripperPos_mm,
        current_ampere_mag=0.02,
        sleep_sec=1.5,
        disable_after_open=True,
    )
    
    # ============================================================
    # 8. close gripper
    # ============================================================
    if verbose:
        print("\n# ======================================")
        print("# Close Gripper after Laying")
        print("# ======================================")

    close_gripper_for_grasp(
        gripper=gripper,
        grasp_arm=grasp_arm,
        grasp_current_obj=grasp_current_obj,
        sleep_sec=1.0
    )
    
    
    if verbose:
        print(f"\n[DONE] Pick and place done: {objectColor} object -> {boxColor} box")

    return True



# Grasp Box's outerside and sligtly lift
def grasp_box_outerside_and_lift(
    robot,
    gripper,
    target_result,
    box_grasp_current,                  # [A]
    box_grasp_gripper_open_pos_mm=60,   # [mm]
    box_floating_z=0.08,                # [m]
    position_control_minimum_time=5.5,  # [sec]
    control_hold_time=0.5,              # [sec]
    fix_torso=True,
    verbose=True,
):
    """
    Grasp box side using target_result.

    Sequence:
        1. Move to {N1}: box side before opening gripper
        2. Open gripper
        3. Move to {N2}: box side after opening gripper
        4. Close gripper
        5. Slightly float box

    Required keys in target_result:
        "grasp_arm"
        "boxColor"
        "T_SC_target_for_box_side_before_open_gripper"
        "T_SC_target_for_box_side_after_open_gripper"
    """

    if target_result is None:
        print("[ERROR] target_result is None.")
        return False

    grasp_arm = target_result["grasp_arm"]
    boxColor = target_result.get("boxColor")

    T_SC_target_box_side_before_open = np.asarray(
        target_result["T_SC_target_for_box_side_before_open_gripper"],
        dtype=np.float64,
    ).copy()

    T_SC_target_box_side_after_open = np.asarray(
        target_result["T_SC_target_for_box_side_after_open_gripper"],
        dtype=np.float64,
    ).copy()

    # ============================================================
    # 1. Move to {N1}: box side with z-offset before opening gripper
    # ============================================================
    if verbose:
        print("\n# ==================================================")
        print(f"# Move to {boxColor} Box Side Before Opening Gripper")
        print("# ==================================================")

    # # --- For safety: keep current EE z-position (theoretically, it is not needed)
    # target_ee_link = get_target_ee(grasp_arm)
    # current_T = ysw.get_FK(robot, "base", target_ee_link)
    # current_pos = np.asarray(current_T["position"], dtype=np.float64)
    # T_SC_target_box_side_before_open[2, 3] = current_pos[2]
    # # ---
    
    rv = positionCtrl_ee_by_T(
        robot=robot,
        T_SC_target=T_SC_target_box_side_before_open,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.6,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to move to {boxColor} box side before opening gripper.")
        return False

    # ============================================================
    # 2. Open gripper
    # ============================================================
    if verbose:
        print("\n# ==================================================")
        print(f"# Open Gripper for grasping {boxColor} Box")
        print("# ==================================================")

    desiredGripperPos_mm = box_grasp_gripper_open_pos_mm

    if verbose:
        print(f"desiredGripperPos_mm: {desiredGripperPos_mm:.2f} mm")

    open_gripper_for_grasp(
        gripper=gripper,
        grasp_arm=grasp_arm,
        desiredGripperPos_mm=desiredGripperPos_mm,
        current_ampere_mag=0.02,
        sleep_sec=1.5,
        disable_after_open=True,
    )

    # ============================================================
    # 3. Move to {N2}: box side after opening gripper
    # ============================================================
    if verbose:
        print("\n# ==================================================")
        print(f"# Move Down to {boxColor} Box Side After Opening Gripper")
        print("# ==================================================")

    # # --- For safety: move downward from current EE z (theoretically, it is not needed)
    # target_ee_link = get_target_ee(grasp_arm)
    # current_T = ysw.get_FK(robot, "base", target_ee_link)
    # current_pos = np.asarray(current_T["position"], dtype=np.float64)
    # T_SC_target_box_side_after_open[2, 3] = current_pos[2] + manual_boxGrasp_down_distance        # -0.07: manual value
    # # ---

    rv = positionCtrl_ee_by_T(
        robot=robot,
        T_SC_target=T_SC_target_box_side_after_open,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.3,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to move to {boxColor} box side after opening gripper.")
        return False

    # ============================================================
    # 4. Close gripper
    # ============================================================
    if verbose:
        print("\n# ==================================================")
        print(f"# Close Gripper for {boxColor} Box")
        print("# ==================================================")

    close_gripper_for_grasp(
        gripper=gripper,
        grasp_arm=grasp_arm,
        grasp_current_obj=box_grasp_current,
        sleep_sec=1.0
    )

    # ============================================================
    # 5. Floating
    # ============================================================
    if verbose:
        print("\n# ==================================================")
        print(f"# Float {boxColor} Box")
        print("# ==================================================")

    pos_after_grasp = T_SC_target_box_side_after_open[:3, 3].copy()
    pos_floating = pos_after_grasp + np.array([0.0, 0.0, box_floating_z], dtype=np.float64)

    rv = positionCtrl_by_pos_and_T_orientation(
        robot=robot,
        pos=pos_floating,
        T_SC_orientation_source=T_SC_target_box_side_after_open,
        grasp_arm=grasp_arm,
        minimum_time=position_control_minimum_time * 0.3,
        control_hold_time=control_hold_time,
        fix_torso=fix_torso,
        get_return=True,
    )

    if rv is False:
        print(f"[ERROR] Failed to float {boxColor} box.")
        return False

    if verbose:
        print(f"\n[DONE] {boxColor.capitalize()} box side grasp and floating done.")

    return True



# 박스를 지면과 평행하게 정렬시키는 동작
def move_ee_to_box_lift_ready_pose(
    robot,
    target_result,
    pos_right=np.array([0.3, -0.3, 1.2]),
    pos_left=np.array([0.3, 0.3, 1.2]),
    target_T_key="T_SC_target_for_box_side_after_open_gripper",
    minimum_time=8.5,
    control_hold_time=0.5,
    fixed_joint_targets=None,
    get_return=True,
):
    """
    Move EE to a manually defined ready/lift pose while maintaining
    the orientation from target_result[target_T_key].
    
    [박스를 잡은 상태에서 옮기기 직전 준비 자세로 위치제어하는 함수]

    Args:
        target_result:
            output of compute_IK_for_colored_object_and_box()

        pos_right:
            target position when grasp_arm == "right"

        pos_left:
            target position when grasp_arm == "left"

        target_T_key:
            key of target_result used only for orientation

        fixed_joint_targets:
            joint target dict for torso etc.
            If None, default torso_0~5 = 0.0 is used.
    """

    if target_result is None:
        raise ValueError("target_result is None.")

    grasp_arm = target_result["grasp_arm"]

    if target_T_key not in target_result:
        raise KeyError(f"{target_T_key} does not exist in target_result.")

    T_final = np.asarray(target_result[target_T_key], dtype=np.float64)

    # ------------------------------------------------------------
    # Select target position and EE link
    # ------------------------------------------------------------
    if grasp_arm == "right":
        pos = np.asarray(pos_right, dtype=np.float64)
        target_link = "ee_finger_r1"

    elif grasp_arm == "left":
        pos = np.asarray(pos_left, dtype=np.float64)
        target_link = "ee_finger_l1"

    else:
        raise ValueError(f"Invalid grasp_arm: {grasp_arm}")

    # Orientation from target_result
    ori = ysw.rotMat_to_rpy(T_final[:3, :3])

    cartesian_position_targets = [
        {
            "reference_link": "base",
            "target_link": target_link,
            "pos": pos,
            "rpy": ori,
        },
    ]

    # ------------------------------------------------------------
    # Default fixed torso joint targets
    # ------------------------------------------------------------
    if fixed_joint_targets is None:
        fixed_joint_targets = {
            "torso_0": 0.0,
            "torso_1": 0.0,
            "torso_2": 0.0,
            "torso_3": 0.0,
            "torso_4": 0.0,
            "torso_5": 0.0,
        }

    rv = ysw.positionCtrl_cartesian_wholeBody(
        robot,
        position_targets=cartesian_position_targets,
        joint_targets=fixed_joint_targets,
        minimum_time=minimum_time,
        control_hold_time=control_hold_time,
        get_return=get_return,
    )

    return rv


# shelf 주변이나 shelf 내부가 비어있으므로, 가장 가깝다고 판단되는 픽셀들 평균으로 가장 가까운 거리를 찾아냄
def get_nearest_depth_avg_in_roi(
    depth_frame,
    pixel_roi_x=None,
    pixel_roi_y=None,
    nearest_ratio=0.05,
    min_num_points=30,
    max_valid_depth=None,
):
    """
    ROI 안에서 가장 가까운 depth 값들의 평균을 계산.

    Args:
        depth_frame:
            RealSense depth_frame

        pixel_roi_x:
            [x_min, x_max]

        pixel_roi_y:
            [y_min, y_max]

        nearest_ratio:
            가까운 depth 중 몇 %를 평균낼지.
            0.05 -> 가장 가까운 5%

        min_num_points:
            평균낼 최소 pixel 개수

        max_valid_depth:
            이 값보다 먼 depth는 무시.
            None이면 제한 없음.

    Return:
        result = {
            "nearest_depth_avg": ...,
            "nearest_depth_median": ...,
            "nearest_depth_min": ...,
            "num_valid_depth": ...,
            "num_used_depth": ...,
            "roi": [x_min, x_max, y_min, y_max],
        }
    """

    depth_raw = np.asanyarray(depth_frame.get_data())
    h, w = depth_raw.shape[:2]

    depth_scale = depth_frame.get_units()
    depth_m = depth_raw.astype(np.float32) * depth_scale

    # ROI x
    if pixel_roi_x is None:
        x_min = 0
        x_max = w
    else:
        x_min, x_max = pixel_roi_x
        x_min = int(np.clip(x_min, 0, w - 1))
        x_max = int(np.clip(x_max, 0, w))

    # ROI y
    if pixel_roi_y is None:
        y_min = 0
        y_max = h
    else:
        y_min, y_max = pixel_roi_y
        y_min = int(np.clip(y_min, 0, h - 1))
        y_max = int(np.clip(y_max, 0, h))

    roi_depth = depth_m[y_min:y_max, x_min:x_max]

    # 0 depth 제거
    valid_depths = roi_depth[roi_depth > 0.0]

    # 너무 먼 depth 제거
    if max_valid_depth is not None:
        valid_depths = valid_depths[valid_depths <= max_valid_depth]

    if len(valid_depths) == 0:
        return None

    valid_depths_sorted = np.sort(valid_depths)

    num_used = int(len(valid_depths_sorted) * nearest_ratio)
    num_used = max(num_used, min_num_points)
    num_used = min(num_used, len(valid_depths_sorted))

    nearest_depths = valid_depths_sorted[:num_used]

    result = {
        "nearest_depth_avg": float(np.mean(nearest_depths)),
        "nearest_depth_median": float(np.median(nearest_depths)),
        "nearest_depth_min": float(np.min(nearest_depths)),
        "num_valid_depth": int(len(valid_depths)),
        "num_used_depth": int(num_used),
        "roi": [x_min, x_max, y_min, y_max],
    }

    return result



# ==========================================================================================================
# Tuning Parameters
# ==========================================================================================================
# Position Control Speed
position_control_minimum_time = 4      # [sec]



# Move to Table/shelf
vision_record_duration_tableDistance = 5.0   # [sec]
offset_table_distance = 0.4                  # [m]
vision_record_duration_shelfDistance = 5.0   # [sec]
offset_shelf_distance = 0.45                  # [m]



# Vision
vision_record_duration_checkBox = 5.0         # [sec]
vision_record_duration_checkObject = 5.0      # [sec]
box_minimum_area = 8000                       # [pixel^2]
box_maximum_distance = 1.2                    # [m]
object_minimum_area = 50                      # [pixel^2]
object_maximum_area = 1000                    # [pixel^2]
object_maximum_distance = 1.2                 # [m]



# Position Control 
offset_gripper_open_m = 0.015  # [m]
object_manual_offset_downToGrasp = -0.005      # [m] for z-direction offset when grasp
object_floating_z = 0.117 - object_manual_offset_downToGrasp     # [m]
object_manual_offset_downToGrasp_blackObj = -0.006      # [m] for z-direction offset when grasp
object_floating_z_blackObj  = 0.116 - object_manual_offset_downToGrasp_blackObj      # [m]
object_manual_offset_downToGrasp_redObj  = -0.010      # [m] for z-direction offset when grasp
object_floating_z_redObj = 0.112 - object_manual_offset_downToGrasp_redObj     # [m]
object_pregrasp_z_offset = +0.1   # [m]
object_grasp_x_offset = +0.014   # [m]

box_pregrasp_z_offset = +0.05   # [m]
box_grasp_z_offset = -0.065   # [m]
box_grasp_floating_offset = 0.08    # [m]
box_grasp_x_offset  = -0.04    # [m] outer side center point에서, 조금 더 앞쪽을 잡기 위한 offset 
box_grasp_y_offset  = 0.05    # [m] 그리퍼가 양쪽 모두 바깥 side보다 더 바깥쪽으로 배치되는 경향을 보정

box_move_readyPosition_right = np.array([0.55, -0.25, 1.2])      # desired position of "ee_finger_r1" from "base"  
box_move_readyPosition_left = np.array([0.55, 0.25, 1.2])        # desired position of "ee_finger_l1" from "base"  



# Gripper
grasp_current_obj = 0.15      # [A]
grasp_current_box = 0.25      # [A]
box_grasp_gripper_open_pos_mm = 80.0      # [mm]
gripper_open_ampere = -0.1    # [A]  
gripper_close_ampere = 0.1    # [A]



# ROI setting [Pixel Origin: (0,0) = 왼쪽 위 꼭짓점]
width = 640
height = 480

# table까지 거리를 잴 때 사용하는 ROI (Posture: PACK_POS)
pixel_roi_x_tableDistance = [140, width - 140] 
pixel_roi_y_tableDistance = [320, height]    

# table위 물체 detection에 사용하는 ROI (Posture: DEMO_BASIC_POS_260826)
pixel_roi_x_tableTop = None
# pixel_roi_y_tableTop = [150, height]
pixel_roi_y_tableTop = [150, height - 50]

# 박스를 grasp하고, 180도 돌아서 shelf까지 갈 때 사용하는 ROI 
pixel_roi_x_shelfDistance = [130, width - 130]      
pixel_roi_y_shelfDistance = [200, height]    



# ==========================================================================================================
# Initialization
# ==========================================================================================================
ysw.print_demoStep(step=1, text="Initialize")

# ----------------------------------------------------------
# Robot Initialization
# ----------------------------------------------------------
robotConfig = ysw.setRobotConfig(ysw.MODE_DIRECT_UPC, ysw.MODEL_A)          # For Real Robot + UPC
robot = ysw.initialize(robotConfig.mode, robotConfig.address, robotConfig.model, option=ysw.CONNECT_AND_POWER)
if (robot != None):
    model = robot.model()
time.sleep(1)

# ----------------------------------------------------------
# Gripper Initialization
# Gripper Setting (Need to hard check of gripper ping test)
# ----------------------------------------------------------
gripper = ysw.Gripper()
gripper.connect_hardCheck(
    device_name=ysw.GRIPPER_DEVICE_NAME, 
    protocol_version=ysw.DXL_PROTOCOL_VERSION, 
    baudrate=ysw.DXL_BAUDRATE,
    retry_interval=1.0,
    max_retry=5
)
time.sleep(1)

# ----------------------------------------------------------
# Depth Camera Initialization
# D435i:  Head
# D405_R: Right Wrist
# D405_L: Left Wrist
# ----------------------------------------------------------
d435i = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.HEAD_D435I_SERIAL)
d435i.initialize()
time.sleep(1)
d405_R = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.RIGHT_D405_SERIAL)
d405_R.initialize()
time.sleep(1)
d405_L = ysw.RealSenseDepthCamera(rgb_on=True, depth_on=True, pixel=(640,480), fps=30, serial_number=ysw.LEFT_D405_SERIAL)
d405_L.initialize()
time.sleep(1)



# ==========================================================================================================
# [Posture] Initial pose: PACK_POS
# ==========================================================================================================
ysw.print_demoStep(step=2, text="(Posture) PACK_POS")

desired_pos = ysw.PACK_POS  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)



# ==========================================================================================================
# [Vision] Check the Distance to Table

# Get the Distance of Center Pixel 
# Table distance is approximated as the depth of center pixel
# ==========================================================================================================
ysw.print_demoStep(step=3, text="(Vision) Check the distance to Table")

record_duration = vision_record_duration_tableDistance      # [sec]
distance_list = []

d435i.startPipeline()

try:
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        
        if elapsed_time >= record_duration:
            break
        
        color_frame, depth_frame = d435i.getRawFrames()

        if color_frame is None or depth_frame is None:
            continue

        color_image, depth_image = d435i.frameToImage(color_frame, depth_frame)
        h, w, _ = color_image.shape
        
        # ROI center pixel in full image coordinate
        if pixel_roi_x_tableDistance is None:
            x_min = 0
            x_max = w
        else:
            x_min, x_max = pixel_roi_x_tableDistance
            x_min = int(np.clip(x_min, 0, w - 1))
            x_max = int(np.clip(x_max, 0, w))

        if pixel_roi_y_tableDistance is None:
            y_min = 0
            y_max = h
        else:
            y_min, y_max = pixel_roi_y_tableDistance
            y_min = int(np.clip(y_min, 0, h - 1))
            y_max = int(np.clip(y_max, 0, h))
            
        x_roi_center = (x_min + x_max) // 2
        y_roi_center = (y_min + y_max) // 2

        distance = d435i.getDepth(depth_frame, (x_roi_center, y_roi_center))   # unit = [m]
        distance_list.append(distance)
        print(f"Table Distance: {distance:.3f} [m]",)

finally:
    d435i.stopPipeline()
    d435i.destroyAllWindows()
    

# Average Distance
distance_avg = np.mean(distance_list)
print(f"\nTable Average Distance: {distance_avg} [m]")

moved_distance_to_table = distance_avg - offset_table_distance
print(f"moved_distance_to_table: {moved_distance_to_table} [m]")


# ==========================================================================================================
# [Posture] Demo Basic pose: DEMO_BASIC_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=4, text="(Posture) DEMO_BASIC_POS_260826")

desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)



# ==========================================================================================================
# [Move] Move forward to Table
# ==========================================================================================================
ysw.print_demoStep(step=5, text="(Mobile) Move Forward to Table")

# expire stream 에러 가끔 나오는데 원인 불명
ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_table,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)



# ==========================================================================================================
# Black Box check
# ==========================================================================================================
ysw.print_demoStep(step=6, text="(Vision) Black Box Detection")

result_blackBox = check_colored_box_and_set_grasp_arm(
    depthCamObj=d435i,
    boxColor="black",
    hsv_mask=ysw.HSV_MASK_BLACK_BOX,
    record_duration=vision_record_duration_checkBox,
    box_minimum_area=box_minimum_area,
    box_maximum_distance=box_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    verbose=True,
)

blackBox_samples = result_blackBox["box_samples"]
blackBox_sample_avg = result_blackBox["box_sample_avg"]
grasp_arm_blackObj = result_blackBox["grasp_arm"]



# ==========================================================================================================
# Black Objects check (indexing from the center located object)
# ==========================================================================================================
ysw.print_demoStep(step=7, text="(Vision) Black Objects Detection")

result_blackObj = check_colored_objects_sorted_by_center(
    depthCamObj=d435i,
    objectColor="black",
    hsv_mask=ysw.HSV_MASK_BLACK_BOX,
    record_duration=vision_record_duration_checkObject,
    object_minimum_area=object_minimum_area,
    object_maximum_area=object_maximum_area,
    maximum_depth=object_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    max_match_distance_pixel=40.0,
    min_rank_sample_ratio=0.4,
    min_rank_num_samples=50,
    verbose=True,
)

blackObj_sample_avg_list = result_blackObj["object_sample_avg_list"]



# ==========================================================================================================
# IK for Black Box and Objects
# ==========================================================================================================
ysw.print_demoStep(step=8, text="(IK) for Black Box and Objects")

black_target_result_list = []

for obj_idx, blackObj_sample_avg in enumerate(blackObj_sample_avg_list):
    if blackObj_sample_avg is None:
        print(f"[Black Object {obj_idx}] skipped: sample_avg is None")
        black_target_result_list.append(None)
        continue
    
    print(f"\n# ==================================================")
    print(f"# Compute IK for Black Object {obj_idx}")
    print(f"# ==================================================")
    
    black_target_result = compute_IK_for_colored_object_and_box(
        robot=robot,
        objectColor=f"black_{obj_idx}",
        object_sample_avg=blackObj_sample_avg,
        boxColor="black",
        box_sample_avg=blackBox_sample_avg,
        grasp_arm=grasp_arm_blackObj,
        object_manual_offset_downToGrasp=object_manual_offset_downToGrasp_blackObj,
        object_pregrasp_z_offset=object_pregrasp_z_offset,
        object_grasp_x_offset=object_grasp_x_offset, 
        box_grasp_z_offset=box_grasp_z_offset,
        box_grasp_x_offset=box_grasp_x_offset,
        box_grasp_y_offset=box_grasp_y_offset - 0.01,       # manual
        use_median=False,
        gripper_z_offset=ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER,
        verbose=True,
    )
    black_target_result_list.append(black_target_result)



# ==========================================================================================================
# Pick and Place for Black Box/Objects
# ==========================================================================================================
ysw.print_demoStep(step=9, text="[Pick and Place] Black Objects to Black Box")

for obj_idx, black_target_result in enumerate(black_target_result_list):
    if black_target_result is None:
        print(f"[Black Object {obj_idx}] skipped: target_result is None")
        continue

    print(f"\n# ==================================================")
    print(f"# Pick and Place Black Object {obj_idx}")
    print(f"# ==================================================")

    rv = pick_object_and_place_to_box(
        robot=robot,
        gripper=gripper,
        target_result=black_target_result,
        offset_gripper_open_m=offset_gripper_open_m,
        grasp_current_obj=grasp_current_obj,
        object_floating_z_afterGrasp=object_floating_z_blackObj,
        position_control_minimum_time=position_control_minimum_time,
        control_hold_time=0.5,
        fix_torso=True,
        verbose=True,
    )

    if rv is False:
        print(f"[Black Object {obj_idx}] Pick-place failed.")
        break
    
    
    
# ==========================================================================================================
# [Posture] Demo Basic pose: DEMO_BASIC_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=10, text="(Posture) DEMO_BASIC_POS_260826")

desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)



# ==========================================================================================================
# Red Box check
# ==========================================================================================================
ysw.print_demoStep(step=11, text="(Vision) Red Box Detection")

result_redBox = check_colored_box_and_set_grasp_arm(
    depthCamObj=d435i,
    boxColor="red",
    hsv_mask=ysw.HSV_MASK_RED_BOX,
    record_duration=vision_record_duration_checkBox,
    box_minimum_area=box_minimum_area,
    box_maximum_distance=box_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    verbose=True,
)

redBox_samples = result_redBox["box_samples"]
redBox_sample_avg = result_redBox["box_sample_avg"]
grasp_arm_redObj = result_redBox["grasp_arm"]



# ==========================================================================================================
# Red Objects check (indexing from the center located object)
# ==========================================================================================================
ysw.print_demoStep(step=12, text="(Vision) Red Objects Detection")

result_redObj = check_colored_objects_sorted_by_center(
    depthCamObj=d435i,
    objectColor="red",
    hsv_mask=ysw.HSV_MASK_RED,
    record_duration=vision_record_duration_checkObject,
    object_minimum_area=object_minimum_area,
    object_maximum_area=object_maximum_area,
    maximum_depth=object_maximum_distance,
    pixel_roi_x=pixel_roi_x_tableTop,
    pixel_roi_y=pixel_roi_y_tableTop,
    max_match_distance_pixel=40.0,
    min_rank_sample_ratio=0.4,
    min_rank_num_samples=50,
    verbose=True,
)

redObj_sample_avg_list = result_redObj["object_sample_avg_list"]



# ==========================================================================================================
# IK for Red Box and Objects
# ==========================================================================================================
ysw.print_demoStep(step=13, text="(IK) for Red Box and Objects")

red_target_result_list = []

for obj_idx, redObj_sample_avg in enumerate(redObj_sample_avg_list):
    if redObj_sample_avg is None:
        print(f"[Red Object {obj_idx}] skipped: sample_avg is None")
        red_target_result_list.append(None)
        continue
    
    print(f"\n# ==================================================")
    print(f"# Compute IK for Red Object {obj_idx}")
    print(f"# ==================================================")
    
    red_target_result = compute_IK_for_colored_object_and_box(
        robot=robot,
        objectColor=f"red_{obj_idx}",
        object_sample_avg=redObj_sample_avg,
        boxColor="red",
        box_sample_avg=redBox_sample_avg,
        grasp_arm=grasp_arm_redObj,
        object_manual_offset_downToGrasp=object_manual_offset_downToGrasp_redObj,
        object_pregrasp_z_offset=object_pregrasp_z_offset,
        object_grasp_x_offset=object_grasp_x_offset - 0.001, # Manual 
        box_grasp_z_offset=box_grasp_z_offset - 0.008,       # Manual
        box_grasp_x_offset=box_grasp_x_offset - 0.005,       # Manual
        box_grasp_y_offset=box_grasp_y_offset,
        use_median=False,
        gripper_z_offset=ysw.GRIPPER_Z_OFFSET_FROM_LINK_EE_FINGER,
        verbose=True,
    )
    red_target_result_list.append(red_target_result)



# ==========================================================================================================
# Pick and Place for Red Box/Objects
# ==========================================================================================================
ysw.print_demoStep(step=14, text="[Pick and Place] Red Objects to Red Box")

for obj_idx, red_target_result in enumerate(red_target_result_list):
    if red_target_result is None:
        print(f"[Red Object {obj_idx}] skipped: target_result is None")
        continue

    print(f"\n# ==================================================")
    print(f"# Pick and Place Red Object {obj_idx}")
    print(f"# ==================================================")

    rv = pick_object_and_place_to_box(
        robot=robot,
        gripper=gripper,
        target_result=red_target_result,
        offset_gripper_open_m=offset_gripper_open_m,
        grasp_current_obj=grasp_current_obj,
        object_floating_z_afterGrasp=object_floating_z_redObj,
        position_control_minimum_time=position_control_minimum_time,
        control_hold_time=0.5,
        fix_torso=True,
        verbose=True,
    )

    if rv is False:
        print(f"[Red Object {obj_idx}] Pick-place failed.")
        break



# ==========================================================================================================
# [Posture] Demo Basic pose: DEMO_BASIC_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=15, text="(Posture) DEMO_BASIC_POS_260826")

desired_pos = ysw.DEMO_BASIC_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.3, control_hold_time=0.5)



# ==========================================================================================================
# Grasp Black and Red Box's outer side and slightly lift
# ==========================================================================================================
ysw.print_demoStep(step=16, text="(Pick Boxes) grasp outer side and slightly lift [Black & Red] Boxes")

# Black Box Grasp and lift
rv = grasp_box_outerside_and_lift(
    robot=robot,
    gripper=gripper,
    target_result=black_target_result_list[0],
    box_grasp_current=grasp_current_box,
    box_grasp_gripper_open_pos_mm=box_grasp_gripper_open_pos_mm,
    box_floating_z=box_grasp_floating_offset,
    position_control_minimum_time=position_control_minimum_time,
    control_hold_time=0.5,
    fix_torso=True,
    verbose=True,
)

# Red Box Grasp an lift
rv = grasp_box_outerside_and_lift(
    robot=robot,
    gripper=gripper,
    target_result=red_target_result_list[0],
    box_grasp_current=grasp_current_box,
    box_grasp_gripper_open_pos_mm=box_grasp_gripper_open_pos_mm,
    box_floating_z=box_grasp_floating_offset,
    position_control_minimum_time=position_control_minimum_time,
    control_hold_time=0.5,
    fix_torso=True,
    verbose=True,
)



# ==========================================================================================================
# [Posture] DEMO_MOVE_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=17, text="(Posture) DEMO_MOVE_POS_260826")

desired_pos = ysw.DEMO_MOVE_POS_260826  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time * 0.6, control_hold_time=0.5)



# ==========================================================================================================
# Change the posture to Box-move-to-shelf Ready Posture
# ==========================================================================================================
ysw.print_demoStep(step=18, text="(Posture) Box-move-to-shelf Ready Posture")

# For Black Box
rv = move_ee_to_box_lift_ready_pose(
    robot=robot,
    target_result=black_target_result_list[0],
    pos_right=box_move_readyPosition_right,      # manual setting
    pos_left=box_move_readyPosition_left,        # manual setting
    target_T_key="T_SC_target_for_box_side_after_open_gripper",
    minimum_time=position_control_minimum_time * 0.6,
    control_hold_time=0.5,
    get_return=True,
)

# For Red Box
rv = move_ee_to_box_lift_ready_pose(
    robot=robot,
    target_result=red_target_result_list[0],
    pos_right=box_move_readyPosition_right,      # manual setting
    pos_left=box_move_readyPosition_left,        # manual setting
    target_T_key="T_SC_target_for_box_side_after_open_gripper",
    minimum_time=position_control_minimum_time * 0.6,
    control_hold_time=0.5,
    get_return=True,
)



# ==========================================================================================================
# [Mobile] Slightly move backward to turn
# ==========================================================================================================
ysw.print_demoStep(step=19, text="(Mobile) Move Backward slightly")

moved_distance_to_back = -0.7       # manually changed

ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_back,
    max_speed=0.12,
    accel=0.1,
    tolerance_m=0.01
)
time.sleep(1)



# ==========================================================================================================
# [Mobile] Turn Right 180 degree (to go shelf)
# ==========================================================================================================
ysw.print_demoStep(step=20, text="(Mobile) Turn Right 180 degree")

desired_yaw_deg = -176                   # Unit: [deg], (+): CCW, (-): CW

ysw.mobile_turn_inplace(
    robot,
    angle_deg=desired_yaw_deg, 
    max_yaw_vel=0.6,
    yaw_accel=1.5,
    tolerance_deg=1.0
)



# ==========================================================================================================
# [Vision] Check the distance to shelf
# Get the Distance of Table using nearest depth average in ROI
# ==========================================================================================================
ysw.print_demoStep(step=21, text="(Vision) Check the distance to shelf")

record_duration = vision_record_duration_shelfDistance      # [sec]

distance_list = []
distance_median_list = []
distance_min_list = []

rv = d435i.startPipeline()
if rv is False:
    raise RuntimeError("[D435i] Failed to start pipeline.")

try:
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time

        if elapsed_time >= record_duration:
            break

        color_frame, depth_frame = d435i.getRawFrames()

        if color_frame is None or depth_frame is None:
            continue

        color_image, depth_image = d435i.frameToImage(color_frame, depth_frame)

        if color_image is None or depth_image is None:
            continue

        nearest_depth_result = get_nearest_depth_avg_in_roi(
            depth_frame=depth_frame,
            pixel_roi_x=pixel_roi_x_shelfDistance,
            pixel_roi_y=pixel_roi_y_shelfDistance,
            nearest_ratio=0.05,      # 가장 가까운 5%
            min_num_points=30,       # 최소 30개 pixel 평균
            max_valid_depth=3.0,     # 필요하면 2.0 [m] 이런 식으로 제한 가능
        )

        if nearest_depth_result is None:
            print("[WARNING] No valid depth in ROI.")
            continue

        nearest_depth_avg = nearest_depth_result["nearest_depth_avg"]
        nearest_depth_median = nearest_depth_result["nearest_depth_median"]
        nearest_depth_min = nearest_depth_result["nearest_depth_min"]

        distance_list.append(nearest_depth_avg)
        distance_median_list.append(nearest_depth_median)
        distance_min_list.append(nearest_depth_min)

        print(
            f"Table Distance nearest_avg: {nearest_depth_avg:.3f} [m], "
            f"median: {nearest_depth_median:.3f} [m], "
            f"min: {nearest_depth_min:.3f} [m], "
            f"used: {nearest_depth_result['num_used_depth']}, "
            f"valid: {nearest_depth_result['num_valid_depth']}"
        )

finally:
    d435i.stopPipeline()
    d435i.destroyAllWindows()

# ------------------
# Average Distance
#-------------------
if len(distance_list) == 0:
    raise RuntimeError("[ERROR] No valid table distance was measured.")

distance_avg = float(np.mean(distance_list))
distance_median = float(np.median(distance_list))
distance_std = float(np.std(distance_list))

print("\n========== Shelf Distance Result ==========")
print(f"num_samples           : {len(distance_list)}")
print(f"Shelf Average Distance: {distance_avg:.4f} [m]")
print(f"Shelf Median Distance : {distance_median:.4f} [m]")
print(f"Shelf Distance Std    : {distance_std:.4f} [m]")

moved_distance_to_shelf = distance_avg - offset_shelf_distance

print(f"\noffset_shelf_distance : {offset_shelf_distance:.4f} [m]")
print(f"moved_distance_to_shelf: {moved_distance_to_shelf:.4f} [m]")



# ==========================================================================================================
# [Mobile] Move Forward to Shelf
# ==========================================================================================================
ysw.print_demoStep(step=22, text="(Mobile) Move Forward to Shelf")

ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_shelf,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)



# ==========================================================================================================
# [Gripper] Open grippers (Lay Boxes) and Close grippers
# ==========================================================================================================
ysw.print_demoStep(step=23, text="(Gripper) Lay Boxes")

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_L, current_ampere=gripper_open_ampere)
time.sleep(1)

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_R, current_ampere=gripper_open_ampere)
time.sleep(1)

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_L, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_L, current_ampere=gripper_close_ampere)
time.sleep(1)

gripper.initialize_setting(dxl_id=ysw.GRIPPER_ID_R, operation_mode=ysw.DXL_CURRENT_CONTROL_MODE)
gripper.write_current_ampere(dxl_id=ysw.GRIPPER_ID_R, current_ampere=gripper_close_ampere)
time.sleep(1)




# ==========================================================================================================
# [Mobile] Go initial states
# ==========================================================================================================
ysw.print_demoStep(step=24, text="(Mobile) Move Backward slightly and Turn Left 180 degree")

moved_distance_to_back = -0.7
ysw.mobile_linear_distance(
    robot,
    distance_m=moved_distance_to_back,
    max_speed=0.25,
    accel=0.8,
    tolerance_m=0.01
)
time.sleep(1)

desired_yaw_deg = +176                   # Unit: [deg], (+): CCW, (-): CW
ysw.mobile_turn_inplace(
    robot,
    angle_deg=desired_yaw_deg, 
    max_yaw_vel=0.6,
    yaw_accel=1.5,
    tolerance_deg=1.0
)
time.sleep(1)



# ==========================================================================================================
# [Posture] DEMO_MOVE_POS_260826
# ==========================================================================================================
ysw.print_demoStep(step=25, text="(Posture) DEMO_MOVE_POS_260826")

desired_pos = ysw.PACK_POS  
ysw.positionCtrl_joint_22dof(robot, desired_pos, minimum_time=position_control_minimum_time, control_hold_time=0.5)





