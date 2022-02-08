// CppAppConsumer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>
#include <chrono>
#include <string>

// Named pipe
#include <windows.h>
#include <tchar.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>

// Set to true if you whish to see the captured and shared image windows
#define DEBUG_WINDOWS false

// Set to true if you wish to see the FPS being shared
#define DEBUG_FPS true

// Maximum buffer size for incoming messages through the named pipe
constexpr auto MAX_NUM_BYTES = 256;


void imgProcess(int proc_num)
{
    std::cout << "Thread " << proc_num << " running..." << std::endl;

    uchar msg[MAX_NUM_BYTES];
    DWORD num_bytes_read;
    uint8_t i;
    bool fSuccess;
    
    cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    if (proc_num == 0)  // Process 0
    {
        aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    } else  // Process 1
    {
        // Set dictionary size depending on the aruco marker selected
        aruco_parameters->adaptiveThreshConstant = 10;
        aruco_parameters->minCornerDistanceRate = 0.005;
        aruco_parameters->minMarkerDistanceRate = 0.005;
        aruco_parameters->polygonalApproxAccuracyRate = 0.01;
        aruco_parameters->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
        aruco_parameters->cornerRefinementWinSize = 1;
        aruco_parameters->cornerRefinementMaxIterations = 100;
        aruco_parameters->maxErroneousBitsInBorderRate = 0.1;
        // Set dictionary size depending on the aruco m arker selected
        aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    }
    
    // Open the shared memory
    boost::interprocess::windows_shared_memory 
        shm(boost::interprocess::open_only, "SharedMemory",
            boost::interprocess::read_write);

    // Map the whole shared memory in this process
    boost::interprocess::mapped_region region(shm, boost::interprocess::read_write);

    // Get a pointer to thememory start address
    uchar* acc_ptr = static_cast<uchar*>(region.get_address());
    int offset = 0;

    // shared_frame_height
    int* shared_frame_height = (int*)acc_ptr;
    int frame_height = *shared_frame_height;
    offset += sizeof(int);

    // shared_frame_height
    int* shared_frame_width = (int*)(acc_ptr + offset);
    int frame_width = *shared_frame_width;
    offset += sizeof(int);

    // shared_bgr_frame_step
    int* shared_bgr_frame_step = (int*)(acc_ptr + offset);
    offset += sizeof(int);

    // shared_gray_frame_step
    int* shared_gray_frame_step = (int*)(acc_ptr + offset);
    offset += sizeof(int);

    // num_frame_buffers
    uchar* num_frame_buffers = acc_ptr + offset;
    // Store value
    uchar NUM_FRAME_BUFFERS = *num_frame_buffers;
    offset += sizeof(uchar);

    // shared_latest_cam_buffer_idx
    uchar* shared_latest_cam_buffer_idx = acc_ptr + offset;
    offset += sizeof(uchar);

    // Local (non-shared) copy of the shared_latest_cam_buffer_idx
    uchar latest_cam_buffer_idx = NUM_FRAME_BUFFERS;

    // shared_latest_cam_buffer_idx[NUM_FRAME_BUFFERS]
    uchar* shared_buffers_idx_in_use = acc_ptr + offset;
    offset += NUM_FRAME_BUFFERS * sizeof(uchar);

    // BGR image access buffer
    cv::Mat bgr_image(frame_height, frame_width, CV_8UC3,
                      acc_ptr + offset, *shared_bgr_frame_step);
    offset += frame_height * (*shared_bgr_frame_step);

    // Grayscale image access buffers
    cv::Mat* gray_images = new cv::Mat[NUM_FRAME_BUFFERS];
    for (i = 0; i < NUM_FRAME_BUFFERS; i++)
    {
        gray_images[i] = cv::Mat(frame_height, frame_width, CV_8UC1,
                                 acc_ptr + offset, *shared_gray_frame_step);
        offset += frame_height * (*shared_gray_frame_step);
    }
    
    // Connect to the pipe or wait until the pipe is available.
    std::cout << "Attempting to connect to the named pipe..." << std::endl;

    std::string pipe_name = "//./pipe/pipe" + std::to_string(proc_num);
    DWORD last_error;
    HANDLE pipeClient;
    while (true)
    {
        pipeClient = CreateFile(
            pipe_name.c_str(),   // pipe name 
            GENERIC_READ,  // read and write access 
            0,              // no sharing 
            NULL,           // default security attributes
            OPEN_EXISTING,  // opens existing pipe 
            0,              // default attributes 
            NULL);          // no template file
        if (pipeClient != INVALID_HANDLE_VALUE)
        {
            std::cout << "Connected to the named pipe." << std::endl;
            break;
        }

        last_error = GetLastError();
        if (last_error != ERROR_PIPE_BUSY)
        {
            std::cerr << "Unable to connect to the named pipe: " << last_error << std::endl;
            return;
        } else
        {
            // Wait a few seconds for the pipe
            if (!WaitNamedPipe(pipe_name.c_str(), 1))
            {
                std::cerr << "Timeout connecting to the named pipe." << std::endl;
                return;
            }
        }
    }

#if DEBUG_WINDOWS
    std::string bgr_wname = "(C#-Consumer " + std::to_string(proc_num) + ") Color image"; //The name of the window
    cv::namedWindow(bgr_wname, cv::WindowFlags::WINDOW_NORMAL);
    std::string gray_wname = "(C#-Consumer " + std::to_string(proc_num) + ") Grayscale image"; //The name of the window
    cv::namedWindow(gray_wname, cv::WindowFlags::WINDOW_NORMAL);
#endif
    // Get our shared mutex
    HANDLE mtx = OpenMutex(
        SYNCHRONIZE, // Default security attributes
        false, // Do not share with shild processes
        TEXT("ARMutex")); // Mutex's name
    if (mtx == NULL)
    {
        std::cerr << "Unable to access the shared named mutex:" << GetLastError() << std::endl;
        return;
    }

    // Initialize with "impossible value"
    uchar frame_idx = NUM_FRAME_BUFFERS;
#if DEBUG_FPS
    int num_frames = 0; // For FPS estimate
    int64 startTime = cv::getTickCount();
#endif
    
    // For aruco markers detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;

    while (true)
    {
        // Wait for message through the pipe
        fSuccess = ReadFile(
            pipeClient,    // pipe handle 
            msg,    // buffer to receive reply 
            MAX_NUM_BYTES * sizeof(TCHAR),  // size of buffer 
            &num_bytes_read,  // number of bytes read 
            NULL);    // not overlapped 

        if ((!fSuccess) && (GetLastError() != ERROR_MORE_DATA))
        {
            std::cerr << "Error reading from the pipe: " << GetLastError() << std::endl;
            break;
        } else if (num_bytes_read == 0)
        {
            std::cout << "No bytes read!" << std::endl;
            break;
        } else
        {
            if ((!fSuccess) && (GetLastError() == ERROR_MORE_DATA))
                std::cerr << "Not all bytes might have been read!" << std::endl;
        }
        // Wait for the mutex to be available
        DWORD dwWaitResult = WaitForSingleObject(
            mtx,    // handle to mutex
            INFINITE);  // no time-out interval
        if (dwWaitResult != WAIT_OBJECT_0)
        {
            std::cerr << "Error waiting for mutex..." << std::endl;
            break;
        }

        // We are not using the last image anymore)
        if (frame_idx != NUM_FRAME_BUFFERS)
            shared_buffers_idx_in_use[frame_idx]--;
        // Get new index
        frame_idx = *shared_latest_cam_buffer_idx;
        // Update number of users of the buffer
        shared_buffers_idx_in_use[frame_idx]++;
        ReleaseMutex(mtx);

        // Process images
        for (i = 0; i < 10; i++)
            cv::aruco::detectMarkers(gray_images[frame_idx],
                                     aruco_dict,
                                     corners,
                                     ids,
                                     aruco_parameters,
                                     rejectedCandidates);
#if DEBUG_WINDOWS
        // Show images for debugging purposes
        cv::imshow(bgr_wname, bgr_image);
        cv::imshow(gray_wname, gray_images[frame_idx]);
        if (cv::waitKey(5) == 'q')
            break;
#endif
#if DEBUG_FPS
        // Compute the FPS
        num_frames += 1;
        if (num_frames == 100)
        {
            int64 endTime = cv::getTickCount();
            std::cout << "Process " << proc_num << ": " << 
                num_frames / ((endTime - startTime)/ cv::getTickFrequency())
                << " FPS\n";
            num_frames = 0;
            startTime = endTime;
        }
#endif
    }
    CloseHandle(pipeClient);
    CloseHandle(mtx);
}


int main()
{
    std::cout << "C++-based consumer app starting!\n";

    // Create threads (they will start immedeatey)
    std::thread proc0(imgProcess, 0);
    //std::thread proc1(imgProcess, 1);
    //std::this_thread::sleep_for(std::chrono::seconds(2));

    // Wait for both threads to finish
    proc0.join();
    //proc1.join();
}
