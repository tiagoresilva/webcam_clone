import namedmutex
import time
import mmap
import win32pipe
import win32file

import numpy as np
import cv2 as cv

# Set to True if you whish to see the captured and shared image windows
DEBUG_WINDOWS = False

# Set to false if you wish to see the FPS being shared
DEBUG_FPS = True

# Either read the frames from a video file or from a camera
USE_VIDEO = True

# Amount of sleep/wait time [s]
WAIT_TIME = 0.030

# Producer process
if __name__ == '__main__':

    NUM_PROCESSES = 1  # Cannot exceed 253 processes
    NUM_FRAME_BUFFERS = NUM_PROCESSES + 2  # Number of shared image buffers
    msg = bytearray([1])  # Trigger message
    num_frames = 0  # For FPS estimate
    MAX_NUM_BYTES = 256

    # Create the shared mutex
    mutex = namedmutex.NamedMutex('ARMutex', existing=False, acquire=False)

    # Access video or camera
    if USE_VIDEO:
        cap = cv.VideoCapture(
            'C:/Users/hugoc/Downloads/temp/WIN_20211206_17_44_35_Pro.mp4')
        fps = cap.get(cv.CAP_PROP_FPS)
        frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    else:
        cap = cv.VideoCapture(0)
        # FPS
        fps = 30
        cap.set(cv.CAP_PROP_FRAME_WIDTH, fps)
        # Width
        frame_width = int(640)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
        # Height
        frame_height = int(480)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_height)

    if DEBUG_WINDOWS:
        cv.namedWindow('Main process')

        bgr_wname = '(C#-Producer) Color image'  # The name of the window
        cv.namedWindow(bgr_wname)
        gray_wname = [''] * NUM_FRAME_BUFFERS
        for i in range(NUM_FRAME_BUFFERS):
            gray_wname[i] = f'(C#-Producer {i}) Grayscale image'  # The name of the window
            # Create the window using the specific name
            cv.namedWindow(gray_wname[i])

    # Acquire first image
    if cap.isOpened() is False:
        raise RuntimeError('capture initialization failed')
    ret, org_bgr_image = cap.read()
    if ret is False:
        raise RuntimeError('Error, unable to acquire frame...')
    org_gray_image = cv.cvtColor(src=org_bgr_image, code=cv.COLOR_BGR2GRAY)
    bgr_img_size = org_bgr_image.strides[0] * frame_height
    gray_img_size = org_gray_image.strides[0] * frame_height

    # Create the shared memory:
    shared_mem = mmap.mmap(
        fileno=-1, tagname='SharedMemory',
        length=4  # int frame_height
        + 4  # int frame_width
        + 4  # int bgr_frame_step
        + 4  # int gray_frame_step
        + 1  # byte num_shared_buffers
        + 1  # byte shared_latest_cam_buffer_idx
        + NUM_FRAME_BUFFERS  # byte shared_buffers_idx_in_use[NUM_FRAME_BUFFERS]
        + bgr_img_size  # shared_rgb_frame[height, width, 3]
        + NUM_FRAME_BUFFERS*gray_img_size,  # shared_gray_frame[height, width, NUM_FRAME_BUFFERS]
        access=mmap.ACCESS_WRITE)

    offset = 0

    # shared_frame_height
    shared_frame_height = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    shared_frame_height[0] = frame_height
    offset += 4

    # shared_frame_width
    shared_frame_width = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    shared_frame_width[0] = frame_width
    offset += 4

    # shared_bgr_frame_step
    shared_bgr_frame_step = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    shared_bgr_frame_step[0] = org_bgr_image.strides[0]
    offset += 4

    # shared_gray_frame_step
    shared_gray_frame_step = \
        np.frombuffer(buffer=shared_mem, dtype=np.int, count=1, offset=offset)
    gray_frame_step = org_gray_image.strides[0]
    offset += 4

    # shared_num_frame_buffers
    shared_num_frame_buffers = np.frombuffer(buffer=shared_mem, dtype=np.uint8,
                                             count=1, offset=offset)
    shared_num_frame_buffers[0] = NUM_FRAME_BUFFERS
    offset += 1

    # shared_latest_cam_buffer_idx
    shared_latest_cam_buffer_idx = np.frombuffer(buffer=shared_mem,
                                                 dtype=np.uint8, count=1,
                                                 offset=offset)
    offset += 1

    # shared_buffers_idx_in_use[NUM_FRAME_BUFFERS]
    shared_buffers_idx_in_use = np.frombuffer(
        buffer=shared_mem, dtype=np.uint8,
        count=NUM_FRAME_BUFFERS, offset=offset
    )
    offset += NUM_FRAME_BUFFERS

    # BGR image (not used here)
    shared_rgb_frame = np.frombuffer(
        buffer=shared_mem, dtype=np.uint8,
        count=bgr_img_size, offset=offset
    ).reshape(frame_height, frame_width, 3)
    offset += bgr_img_size

    # Grayscale images
    shared_gray_frames = list()
    for i in range(NUM_FRAME_BUFFERS):
        shared_gray_frames.append(np.frombuffer(
            buffer=shared_mem, dtype=np.uint8,
            count=gray_img_size,
            offset=offset,
        ).reshape(frame_height, frame_width))
        offset += gray_img_size

    # Create NUM_PROCESSES named pipes to trigger new image availability
    # (1 per consumer process)
    pipes = [None] * NUM_PROCESSES
    pipes_connected = [False] * NUM_PROCESSES
    blocking_pipe = True  # Set to True to block/wait for clients
    for i in range(NUM_PROCESSES):
        # Create the named pipe
        # pipes[i] = os.mkfifo()  # Only in UNIX
        pipe_path = r'\\.\pipe\pipe'+f'{i}'
        if blocking_pipe is True:
            pipes[i] = win32pipe.CreateNamedPipe(
                pipe_path,
                win32pipe.PIPE_ACCESS_DUPLEX,
                win32pipe.PIPE_TYPE_BYTE | win32pipe.PIPE_WAIT,
                1, MAX_NUM_BYTES, MAX_NUM_BYTES, 0, None)
        else:
            pipes[i] = win32pipe.CreateNamedPipe(
                pipe_path,
                win32pipe.PIPE_ACCESS_DUPLEX,
                win32pipe.PIPE_TYPE_BYTE | win32pipe.PIPE_NOWAIT,
                1, MAX_NUM_BYTES, MAX_NUM_BYTES, 0, None)

    # Try to connect to each process
    for i in range(NUM_PROCESSES):
        # Try to connect to the client
        print(f'Waiting for client {i + 1} connection...')
        try:
            win32pipe.ConnectNamedPipe(pipes[i], None)
            pipes_connected[i] = True
            print(f'Client {i} connected...')
        except Exception:
            pass  # Client might not be available at this time

    start_time = time.time()
    while True:
        # Check if the clients have already connected
        for i in range(NUM_PROCESSES):
            if pipes_connected[i]is True:
                continue
            # Try to connect to the client
            try:
                win32pipe.ConnectNamedPipe(pipes[i], None)
                pipes_connected[i] = True
                print(f'Client {i} connected...')
            except Exception:
                pass  # Client might not be available at this time

        # Wait for the mutex to be available.
        mutex.acquire(4)
        if mutex.acquired is False:
            print('Unable to acquire mutex....')
            continue
        # Find free (unused) frame buffer (we could have a specific mutex for
        # this, for better performance)
        next_cam_buffer_idx = NUM_FRAME_BUFFERS
        for i in range(NUM_FRAME_BUFFERS):
            if (shared_buffers_idx_in_use[i] == 0) and \
               (i != shared_latest_cam_buffer_idx[0]):
                next_cam_buffer_idx = i
                break
        # Safety check
        if next_cam_buffer_idx == NUM_FRAME_BUFFERS:
            raise RuntimeError('No available buffer index!')
        # Release our mutex. We will capture it again later on
        mutex.release()

        ret, _ = cap.read(shared_rgb_frame)
        if ret is False:
            print('Unable to acquire more frames...')
            break
        cv.cvtColor(src=shared_rgb_frame, code=cv.COLOR_BGR2GRAY,
                    dst=shared_gray_frames[next_cam_buffer_idx])

        # Updated the latest shared frame buffer index (protected by the mutex)
        mutex.acquire()
        shared_latest_cam_buffer_idx[0] = next_cam_buffer_idx
        mutex.release()

        # Message all connected consumers that a new image is available
        for i in range(NUM_PROCESSES):
            if (pipes_connected[i]):
                res = win32file.WriteFile(pipes[i], msg)
        if DEBUG_WINDOWS:
            cv.imshow(bgr_wname, shared_rgb_frame)
            for i in range(NUM_FRAME_BUFFERS):
                cv.imshow(gray_wname[i], shared_gray_frames[i])
            if cv.waitKey(5) == 27:  # Wait for the key pressing event
                print('Exiting...')
                break
        else:
            time.sleep(WAIT_TIME)
        if DEBUG_FPS:
            # Compute the FPS
            num_frames += 1
            if num_frames == 100:
                end_time = time.time()
                print(f'Producer: {num_frames / (end_time - start_time):.2f} FPS')
                num_frames = 0
                start_time = end_time
    # We are exiting,close all pipes and memory
    for i in range(NUM_PROCESSES):
        if (pipes_connected[i]):
            win32pipe.DisconnectNamedPipe(pipes[i])
            pipes_connected[i] = False
    shared_mem.close()
