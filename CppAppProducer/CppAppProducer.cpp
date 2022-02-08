// CppAppProducer.cpp : This file contains the 'main' function. Program execution begins and ends there.
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
#include <opencv2/imgproc.hpp>

#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>

// Set to true if you whish to see the captured and shared image windows
#define DEBUG_WINDOWS false

// Set to true if you wish to see the FPS being shared
#define DEBUG_FPS true

// Use a camera or video feed
#define USE_VIDEO  true // Get images from a video feed
//#undef USE_VIDEO  // Get images from a camera feed

// Maximum buffer size for incoming messages through the named pipe
constexpr auto MAX_NUM_BYTES = 256;

// Amount of sleep/wait time [ms]
#define WAIT_TIME 30

int main()
{
	std::cout << "C++-based producer app starting!\n";

	byte NUM_PROCESSES = 1; // Cannot exceed 253 processes
	byte NUM_FRAME_BUFFERS = NUM_PROCESSES + 2; // Number of shared buffers for images
	byte msg[MAX_NUM_BYTES];
	msg[0] = 1; // Trigger message
	byte i;
	DWORD bytes_written;
#if DEBUG_FPS
	int num_frames = 0; // For FPS estimate
#endif

	// Create the named mutex. Only one system object with a given 
	//  name can exist; the local Mutex object represents 
	// this system object, regardless of which process or thread
	// caused the mutex to be created.
	HANDLE mtx = CreateMutex(
		NULL, // Default security attributes
		false, // Do not share with shild processes
		TEXT("ARMutex")); // Mutex's name
	if (mtx == NULL)
	{
		std::cerr << "Unable to create the shared named mutex:" << GetLastError() << std::endl;
		return -1;
	}

	// Get video source
#if USE_VIDEO
	cv::VideoCapture cam("C:/Users/hugoc/Downloads/temp/WIN_20211206_17_44_35_Pro.mp4");
#else
	cv::VideoCapture cam(0,  cv::CAP_MSMF);
#endif
	// set video properties
#if USE_VIDEO
	double fps = cam.get(cv::CAP_PROP_FPS);
	int frame_width = cam.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = cam.get(cv::CAP_PROP_FRAME_HEIGHT);
#else
		// Frames per second
	double fps = 30;
	if (cam.set(cv::CAP_PROP_FPS, fps) == false)
		std::cerr << "Unable to change FPS" << std::endl;
	// Width
	int frame_width = 640;
	if (cam.set(cv::CAP_PROP_FRAME_WIDTH, frame_width) == false)
		std::cerr << "Unable to change frame width" << std::endl;
	// Frames per second
	int frame_height = 480;
	if (cam.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height) == false)
		std::cerr << "Unable to change frame height" << std::endl;
#endif

#if DEBUG_WINDOWS
	std::string bgr_wname = "(C#-Producer) Color image"; // The name of the window
	cv::namedWindow(bgr_wname, cv::WindowFlags::WINDOW_NORMAL);
	std::vector<std::string> gray_wnames;
	for (i = 0; i < NUM_FRAME_BUFFERS; i++)
	{
		// Create the window using the specific name
		gray_wnames.push_back("(C#-Producer) Grayscale image " + std::to_string(i));
		cv::namedWindow(gray_wnames[i], cv::WindowFlags::WINDOW_NORMAL);
	}
#endif

	// Acquire first image
	if (!cam.isOpened())
		throw "Capture initialization failed";
	cv::Mat org_bgr_image;
	cam.read(org_bgr_image);
	cv::Mat org_gray_image;
	cv::cvtColor(org_bgr_image, org_gray_image, cv::COLOR_BGR2GRAY);
	int bgr_img_size = org_bgr_image.step * frame_height,
		gray_img_size = org_gray_image.step * frame_height;

	// Open the shared memory
	boost::interprocess::windows_shared_memory
		shm(boost::interprocess::create_only, "SharedMemory",
			boost::interprocess::read_write,
			sizeof(int) // int frame_height
			+ sizeof(int) // int frame_width
			+ sizeof(int) // int bgr_frame_step
			+ sizeof(int) // int gray_frame_step
			+ sizeof(byte) // num_shared_buffers
			+ sizeof(byte) // byte shared_latest_cam_buffer_idx (writen by the producer)
			+ NUM_FRAME_BUFFERS * sizeof(byte) // byte shared_buffers_idx_in_use[NUM_FRAME_BUFFERS] (written by the producer and consumers)
			+ bgr_img_size * sizeof(byte) // shared_rgb_frame[height, width, 3] (written by the producer) (not currently being used by anyone)
			+ gray_img_size * NUM_FRAME_BUFFERS * sizeof(byte)); // shared_gray_frame[height, width, NUM_FRAME_BUFFERS] (written by the producer)

	// Map the whole shared memory in this process
	boost::interprocess::mapped_region region(shm, boost::interprocess::read_write);

	// Get a pointer to thememory start address
	byte* acc_ptr = static_cast<byte*>(region.get_address());
	int offset = 0;

	// shared_frame_height
	int* shared_frame_height = (int*)acc_ptr;
	*shared_frame_height = frame_height;
	offset += sizeof(int);

	// shared_frame_height
	int* shared_frame_width = (int*)(acc_ptr + offset);
	*shared_frame_width = frame_width;
	offset += sizeof(int);

	// shared_bgr_frame_step
	int* shared_bgr_frame_step = (int*)(acc_ptr + offset);
	// Initialize it 
	*shared_bgr_frame_step = org_bgr_image.step;
	offset += sizeof(int);

	// shared_gray_frame_step
	int* shared_gray_frame_step = (int*)(acc_ptr + offset);
	// Initialize it 
	*shared_gray_frame_step = org_gray_image.step;
	offset += sizeof(int);

	// num_frame_buffers
	byte* num_frame_buffers = acc_ptr + offset;
	// Initialize it 
	*num_frame_buffers = NUM_FRAME_BUFFERS;
	offset += sizeof(byte);

	// shared_latest_cam_buffer_idx
	byte* shared_latest_cam_buffer_idx = acc_ptr + offset;
	// Initialize it 
	*shared_latest_cam_buffer_idx = 0;
	offset += sizeof(byte);

	// Local (non-shared) copy of the shared_latest_cam_buffer_idx
	byte latest_cam_buffer_idx = NUM_FRAME_BUFFERS;

	// shared_latest_cam_buffer_idx[NUM_FRAME_BUFFERS]
	byte* shared_buffers_idx_in_use = acc_ptr + offset;
	// Initialize it to 0
	for (i = 0; i < NUM_FRAME_BUFFERS; i++)
		shared_buffers_idx_in_use[i] = 0;
	offset += NUM_FRAME_BUFFERS * sizeof(byte);

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

	// Create NUM_PROCESSES named pipes to trigger new image availability (1 per consumer process)
	std::vector<HANDLE> pipeServers;
	for (i = 0; i < NUM_PROCESSES; i++)
	{
		std::string pipe_name = "//./pipe/pipe" + std::to_string(i);
		pipeServers.push_back(CreateNamedPipe(
			pipe_name.c_str(),   // pipe name 
			PIPE_ACCESS_OUTBOUND,  // write access 
			PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT ,
			1,           // Max instances
			MAX_NUM_BYTES,  // Out buffer size 
			MAX_NUM_BYTES,  // In buffer size
			0,				// Default timeout (50 ms) - not used here
			NULL));          // Default security attributes

		if (pipeServers[i] == INVALID_HANDLE_VALUE)
			throw "Unabel to create the named pipe" + pipe_name;
	}

	// Wait for each client to connect
	for (i = 0; i < NUM_PROCESSES; i++)
	{
		std::cout << "Waiting for client " << std::to_string(i) << " connection..." << std::endl;
		// Either use the three following lines of code to wait for all producers
		if (ConnectNamedPipe(pipeServers[i], NULL) == 0)
			throw "Unable to connect to pipe " + std::to_string(i);
		std::cout << "Connected to client " << std::to_string(i) << "." << std::endl;
		// Asynchronouse connection is not implemented...
	}

#if DEBUG_FPS
	int64 startTime = cv::getTickCount();
#endif

	// Main, infinite, cycle
	while (true)
	{
		// Wait for the mutex to be available
		DWORD dwWaitResult = WaitForSingleObject(
			mtx,    // handle to mutex
			INFINITE);  // no time-out interval
		if (dwWaitResult != WAIT_OBJECT_0)
		{
			std::cerr << "Error waiting for mutex..." << std::endl;
			break;
		}
		// Find free (unused) frame buffer (we could have a specific mutex for this, for better performance)
		byte next_cam_buffer_idx = NUM_FRAME_BUFFERS;
		for (i = 0; i < NUM_FRAME_BUFFERS; i++)
		{
			if ((shared_buffers_idx_in_use[i] == 0) &&
				(i != *shared_latest_cam_buffer_idx))
			{
				next_cam_buffer_idx = i;
				break;
			}
		}
		// Safety check
		if (next_cam_buffer_idx == NUM_FRAME_BUFFERS)
			throw "No available buffer index!";
		// Release our mutex. We will capture it again later on
		ReleaseMutex(mtx);

		// Capture and store images. The grayscale image goes to the appropriate buffer
		if (cam.read(bgr_image) == false)
		{
			std::cerr << "Unable to acquire more frames.Exiting..." << std::endl;
			break;
		}
		cv::cvtColor(bgr_image, gray_images[next_cam_buffer_idx], cv::COLOR_BGR2GRAY);

		// Updated the latest shared frame buffer index (protected by the mutex)
		dwWaitResult = WaitForSingleObject(
			mtx,    // handle to mutex
			INFINITE);  // no time-out interval
		if (dwWaitResult != WAIT_OBJECT_0)
		{
			std::cerr << "Error waiting for mutex..." << std::endl;
			break;
		}
		*shared_latest_cam_buffer_idx = next_cam_buffer_idx;
		ReleaseMutex(mtx);

		// Message all connected consumers that a new image is available
		for (i = 0; i < NUM_PROCESSES; i++)
		{
			WriteFile(pipeServers[i],
				msg,
				1,   // = length of string + terminating '\0' !!!
				&bytes_written,
				NULL);
		}
#if DEBUG_WINDOWS
		cv::imshow(bgr_wname, bgr_image);
		for (i = 0; i < NUM_FRAME_BUFFERS; i++)
			cv::imshow(gray_wnames[i], gray_images[i]);
		if (cv::waitKey(5) == 'q')
		{
			std::cout << "Exiting..."<< std::endl;
			break;
		}
#else
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
#endif
#if DEBUG_FPS
		// Compute the FPS
		num_frames += 1;
		if (num_frames == 100)
		{
			int64 endTime = cv::getTickCount();
			std::cout << "Producer: " <<
				num_frames / ((endTime - startTime) / cv::getTickFrequency())
				<< " FPS\n";
			num_frames = 0;
			startTime = endTime;
		}
#endif
	}
	for (i = 0; i < NUM_PROCESSES; i++)
		CloseHandle(pipeServers[i]);
	CloseHandle(mtx);
}
