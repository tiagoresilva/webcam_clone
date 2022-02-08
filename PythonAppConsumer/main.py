import namedmutex
import ctypes
from ctypes import sizeof, wintypes
import time
import mmap
import multiprocessing as mp
import multiprocessing.connection as mpc
import queue
import time
import socket
import tempfile
import os.path

import numpy as np
import cv2 as cv

# Set to True if you whish to see the captured and shared image windows
DEBUG_WINDOWS = False

# Set to false if you wish to see the FPS being shared
DEBUG_FPS = True


# Function to run consumer processes
def img_process(proc_num):
    '''
        proc_num: Process number (for the socket connection);
    '''
    if DEBUG_WINDOWS:
        cv.namedWindow(f'(Python-Consumer) Process {proc_num}')
    MAX_NUM_BYTES = 200

    # Aruco detector configuration
    # @param self Detector parameters.
    aruco_parameters = cv.aruco.DetectorParameters_create()
    if proc_num == 0:  # Process 0
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    else:  # Process 1
        ## Set dictionary size depending on the aruco marker selected
        aruco_parameters.adaptiveThreshConstant = 10
        aruco_parameters.minCornerDistanceRate = 0.005
        aruco_parameters.minMarkerDistanceRate = 0.005
        aruco_parameters.polygonalApproxAccuracyRate = 0.01
        aruco_parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        aruco_parameters.cornerRefinementWinSize = 1
        aruco_parameters.cornerRefinementMaxIterations = 100
        aruco_parameters.maxErroneousBitsInBorderRate = 0.1 
        ## Set dictionary size depending on the aruco m arker selected
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)
        

    # Get access to the shared memory. We will do this in two runs, the first
    # to get the base sizes, and then the real deal.
    # Check the C# Producer code for more information on this
    shared_mem = mmap.mmap(fileno=-1, tagname='SharedMemory',
                           length=4  # int frame_height
                                  + 4  # int frame_width
                                  + 4  # int bgr_frame_step
                                  + 4  # int gray_frame_step
                                  + 1,  # byte num_shared_buffers
                           access=mmap.ACCESS_WRITE)
    offset = 0

    # shared_frame_height
    shared_frame_height = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    frame_height = shared_frame_height[0]
    offset += 4
    
    # shared_frame_width
    shared_frame_width = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    frame_width = shared_frame_width[0]
    offset += 4

    # shared_bgr_frame_step
    shared_bgr_frame_step = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    bgr_frame_step = shared_bgr_frame_step[0]
    offset += 4

    # shared_gray_frame_step
    shared_gray_frame_step = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    gray_frame_step = shared_gray_frame_step[0]
    offset += 4

    # Compute image sizes from the above data
    bgr_step_size =  frame_width*3
    bgr_frame_size = frame_height*bgr_step_size
    gray_step_size = frame_width
    gray_frame_size = frame_height*gray_step_size

    # shared_num_frame_buffers
    shared_num_frame_buffers = \
        np.frombuffer(buffer=shared_mem, dtype=np.uint8, count=1, offset=offset)
    num_frame_buffers = shared_num_frame_buffers[0]
    offset += 1

    # Reopen shared memory with the correct size
    shared_mem.close()
    shared_mem = mmap.mmap(
        fileno=-1, tagname='SharedMemory',
        length=4*4 + 1 +  # Previous memory region
               + 1  # byte shared_latest_cam_buffer_idx
               + num_frame_buffers  # byte shared_buffers_idx_in_use[NUM_FRAME_BUFFERS]
               + bgr_frame_size  # shared_rgb_frame[height, width, 3] 
               + num_frame_buffers*gray_frame_size,  # shared_gray_frame[height, width, NUM_FRAME_BUFFERS]
        access=mmap.ACCESS_WRITE)

    # shared_latest_cam_buffer_idx
    shared_latest_cam_buffer_idx = \
        np.frombuffer(buffer=shared_mem, dtype=np.uint8, count=1, offset=offset)
    offset += 1

    # Local (non-shared) copy of the shared_latest_cam_buffer_idx
    latest_cam_buffer_idx = num_frame_buffers

    # shared_buffers_idx_in_use[NUM_FRAME_BUFFERS]
    shared_buffers_idx_in_use = np.frombuffer(
        buffer=shared_mem, dtype=np.uint8,
        count=num_frame_buffers, offset=offset
       )
    offset += num_frame_buffers

    # BGR image (not used here)
    shared_rgb_frame = np.frombuffer(
        buffer=shared_mem, dtype=np.uint8,
        count=bgr_frame_size, offset=offset
       ).reshape(frame_height, frame_width, 3)
    offset += bgr_frame_size
    
    # Grayscale images
    shared_gray_frames = list()
    for i in range(num_frame_buffers):
        shared_gray_frames.append(np.frombuffer(
            buffer=shared_mem, dtype=np.uint8,
            count=gray_frame_size,
            offset=offset,
           ).reshape(frame_height, frame_width))
        offset += gray_frame_size

    # Connect to producer socket for this consumer
    f = open(r'\\.\pipe\pipe'+f'{proc_num}', 'rb', buffering=0)

    # Get the shared mutex
    mutex = namedmutex.NamedMutex('ARMutex', existing=True, acquire=False)

    # Variable to hold the image processing result
    result_img = np.empty((frame_height, frame_width),
                          dtype=np.uint8)
    start = time.time()
    num_frames = 0
    frame_idx = -1
    msg = bytearray(MAX_NUM_BYTES)
    while True:
        # Wait for a signal that a new frame is available. This is a blocking
        # call. The message contents per se are not being used currently.
        try:
            result = f.readinto(msg)
        except Exception as e:
            print(f'Got an error reading from the pipe, exiting: {e}')
            break
        if result == 0:
            print("Pipe closed, exiting...")
            break
        elif result == MAX_NUM_BYTES:
            print("Not all bytes might have beed read!")

        # If we did not get the '1', somethin is wrong!!
        if msg[0] != 1:
            print('Unexpected pipe message...')
            continue

        # Check the index of the latest frame and signal its use
        mutex.acquire(4)
        if mutex.acquired:
            # Decrease the last frame index usage (except the first time)
            if frame_idx != -1:
                shared_buffers_idx_in_use[frame_idx] -= 1
            # Store the latest frame index
            frame_idx = shared_latest_cam_buffer_idx[0]
            # Increase the number of processes currently using this frame index
            shared_buffers_idx_in_use[frame_idx] += 1
        else:
            print('Unable to acquire mutex....')
            continue
        mutex.release()

        # Process frame (detect ARUCO marker)
        # We do not need to yse the shared_frame_arr lock, since the frame will
        # not be updated while we are using it.
        for i in range(10):
            corners, ids, rejectedImgPoints = \
                cv.aruco.detectMarkers(shared_gray_frames[frame_idx],
                                    aruco_dict, parameters=aruco_parameters)

        if DEBUG_WINDOWS:
            # Debug code: show the result and update the FPS indo
            cv.imshow(f'(Python-Consumer) Process {proc_num}', shared_gray_frames[frame_idx])
            #cv.imshow(f'(Python-Consumer) Process {proc_num}', result_img)
            cv.waitKey(5)
        if DEBUG_FPS:
            num_frames += 1
            if num_frames == 100:
                end = time.time()
                print(f'Process {proc_num}: {num_frames/(end-start):.2f} FPS')
                num_frames = 0
                start = end
    f.close()
    shared_mem.close()

# Producer process
if __name__ == '__main__':
    # Create two processes
    proc0 = mp.Process(target=img_process, name='Process0',
                       args=(0,))
    #proc1 = mp.Process(target=img_process, name='Process1',
    #                   args=(1,))

    # Start the two processes
    proc0.start()
    #proc1.start()

    # Wait until nboth processes are finished
    proc0.join()
    #proc1.join()
