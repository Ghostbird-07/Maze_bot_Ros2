import cv2
import numpy as np

from . import config

def imfill(image):
    """Fill enclosed regions in binary image"""
    cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]  # OpenCV 4.2
    for idx, _ in enumerate(cnts):
        cv2.drawContours(image, cnts, idx, 255, -1)
    return image

def ret_largest_obj(img):
    """Return the largest object in the binary image"""
    # Find contours in the image
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    
    # If no contours found, return empty image and None
    if len(cnts) == 0:
        return np.zeros_like(img), None
    
    # Find the largest contour
    Max_Cntr_area = 0
    Max_Cntr_idx = -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area > Max_Cntr_area:
            Max_Cntr_area = area
            Max_Cntr_idx = index
    
    # Create a mask with only the largest object
    img_largestobject = np.zeros_like(img)
    if (Max_Cntr_idx != -1):
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, -1)  # Fill the contour
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, 2)   # Draw the contour
        return img_largestobject, cnts[Max_Cntr_idx]
    else:
        return img_largestobject, None

def ret_smallest_obj(cnts, noise_thresh=10):
    """Return the index of the smallest object in the contours list, above noise threshold"""
    if not cnts:
        return -1
        
    Min_Cntr_area = float('inf')
    Min_Cntr_idx = -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if (area < Min_Cntr_area) and (area > noise_thresh):
            Min_Cntr_area = area
            Min_Cntr_idx = index
    
    if Min_Cntr_idx != -1:
        print("min_area", Min_Cntr_area)
    return Min_Cntr_idx

def get_centroid(cnt):
    """Calculate the centroid of a contour"""
    M = cv2.moments(cnt)
    if M['m00'] == 0:
        return (0, 0)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cy, cx)

def connect_objs(bin_img):
    """Connect nearby objects in binary image using morphological operations"""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)

class Debugging:
    """Debug utilities for mazebot"""
    
    def __init__(self): 
        self.time_elasped = 0
        self.Live_created = False
        
        # Create debug control window
        cv2.namedWindow('CONFIG')
        
        # Create switches for different debug functionalities
        cv2.createTrackbar('Debug', 'CONFIG', False, True, self.nothing)
        cv2.createTrackbar('Debug Loc', 'CONFIG', False, True, self.nothing)
        cv2.createTrackbar('Debug Mapp.', 'CONFIG', False, True, self.nothing)
        cv2.createTrackbar('Debug Path P.', 'CONFIG', False, True, self.nothing)
        cv2.createTrackbar('Debug Motion P.', 'CONFIG', False, True, self.nothing)
        cv2.createTrackbar('Debug_Live', 'CONFIG', False, True, self.nothing)

    def nothing(self, x):
        """Empty callback function for trackbars"""
        pass

    def setDebugParameters(self):
        """Update debug parameters based on trackbar positions"""
        if (self.time_elasped > 5):
            # Get current positions of trackbars
            debug = cv2.getTrackbarPos('Debug', 'CONFIG')
            debug_localization = cv2.getTrackbarPos('Debug Loc', 'CONFIG')
            debug_mapping = cv2.getTrackbarPos('Debug Mapp.', 'CONFIG')
            debug_pathplanning = cv2.getTrackbarPos('Debug Path P.', 'CONFIG')
            debug_motionplanning = cv2.getTrackbarPos('Debug Motion P.', 'CONFIG')
            debug_live = cv2.getTrackbarPos('Debug_Live', 'CONFIG')

            # Update config values
            config.debug = bool(debug)
            config.debug_localization = bool(debug_localization)
            config.debug_mapping = bool(debug_mapping)
            config.debug_pathplanning = bool(debug_pathplanning)
            config.debug_motionplanning = bool(debug_motionplanning)
            config.debug_live = bool(debug_live)
        else: 
            self.time_elasped += 1
        
        # Handle live debugging settings
        if config.debug_live:
            if not self.Live_created:
                self.Live_created = True
                cv2.namedWindow('CONFIG_LIVE')
                cv2.createTrackbar('Debug (Live)', 'CONFIG_LIVE', 0, 100, self.nothing)
                cv2.createTrackbar('Debug_map (Live)', 'CONFIG_LIVE', 0, 100, self.nothing)
                cv2.createTrackbar('Debug_path (Live)', 'CONFIG_LIVE', 0, 100, self.nothing)

            debug_live_amount = cv2.getTrackbarPos('Debug (Live)', 'CONFIG_LIVE')
            debug_map_live_amount = cv2.getTrackbarPos('Debug_map (Live)', 'CONFIG_LIVE')
            debug_path_live_amount = cv2.getTrackbarPos('Debug_path (Live)', 'CONFIG_LIVE')

            config.debug_live_amount = (debug_live_amount / 100)
            config.debug_map_live_amount = (debug_map_live_amount / 100)
            config.debug_path_live_amount = (debug_path_live_amount / 100)
        else:
            self.Live_created = False
            try:
                cv2.destroyWindow('CONFIG_LIVE')
            except:
                pass