#!/usr/bin/python3

import ctypes
import time
import rospy as ros
import rospkg
import os
import numpy as np
import cv2 as cv
import subprocess
import multiprocessing as mp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

ros.init_node("testModule")
roscpp_init('testModule', [])

# CPP Module Wrapper

rospack = rospkg.RosPack()
driverPath = os.path.join(rospack.get_path("hikrobot_camera"), "../../devel/lib/libhikrobot_camera.so")

Driver = ctypes.cdll.LoadLibrary(driverPath)

Camera_ptr = ctypes.POINTER(ctypes.c_char_p)
NodeHandle_ptr = ctypes.POINTER(ctypes.c_char_p)
Mat_ptr = ctypes.c_char_p

Driver.createCamera.restype = Camera_ptr
Driver.createCamera.argtypes = [ctypes.c_int]

Driver.triggerCapture.argtypes = [Camera_ptr, Mat_ptr]
Driver.getImage.argtypes = [Camera_ptr, Mat_ptr]

Driver.getHeight.argtypes = [Camera_ptr]
Driver.getHeight.restype = ctypes.c_int

Driver.getWidth.argtypes = [Camera_ptr]
Driver.getWidth.restype = ctypes.c_int

# Python Module

class Camera:
    def __init__(self, idx: int) -> None:
        self.idx = idx
        self.cam = Driver.createCamera(idx)
        self.height = Driver.getHeight(self.cam)
        self.width = Driver.getWidth(self.cam)
        
    def triggerCapture(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img = np.asarray(img)
        ptr = img.ctypes.data_as(ctypes.c_char_p)
        print(f"Triggering capture for camera {self.idx}")
        Driver.getImage(self.cam, ptr)
        return img

def run(idx: int):
    cam = Camera(idx)
    print(f"Camera {cam.idx} is running")
    while not ros.is_shutdown():
        start = time.time()
        img = cam.triggerCapture()
        end = time.time()
        print(f"Time (cam {cam.idx}): {end - start}s")
        

def main():
    # left_cam = Camera(0)
    # right_cam = Camera(1)

    # run(left_cam)
    left_process = mp.Process(target=run, args=(0,))
    # right_process = mp.Process(target=run, args=(right_cam,))
    
    left_process.start()
    # right_process.start()
    left_process.join()
    # right_process.join()

    ros.spin()
    
if __name__ == "__main__":
    main()
    
    
    
