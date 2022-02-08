// Define this if you whish to see the captured and shared image windows
#define DEBUG_WINDOWS
//#undef DEBUG_WINDOWS

// Define this if you wish to see the FPS being shared
#define DEBUG_FPS
//#undef DEBUG_FPS

using System;
using System.Threading;
using System.IO.Pipes;
using System.IO.MemoryMappedFiles;

using Emgu.CV;
using Emgu.CV.CvEnum;
using System.Threading.Tasks;

static class Constants
{
    // Amount of sleep/wait time [ms]
    public const int WAIT_TIME = 30;
}


public class MultiProcessShMemWithPython
{
    public static unsafe void Main()
    {
        byte NUM_PROCESSES = 3; // Cannot exceed 253 processes
        byte NUM_FRAME_BUFFERS = (byte)(NUM_PROCESSES + 2); // Number of shared buffers for images
        int frame_width = 640, frame_height = 480, fps = 30;
        byte[] msg = new byte[1];
        msg[0] = 1; // Trigger message
        byte i; // General use
        int num_frames = 0; // For FPS estimate
        var startTime = Environment.TickCount;

        // Create the named mutex. Only one system object with a given 
        //  name can exist; the local Mutex object represents 
        // this system object, regardless of which process or thread
        // caused the mutex to be created.
        Mutex mtx = new Mutex(false, "ARMutex");

        // For EMGU docs, see https://www.emgu.com/wiki/files/4.5.4/document/html/R_Project_Emgu_CV_Documentation.htm
        //VideoCapture cam = new VideoCapture(0, VideoCapture.API.Msmf);
        VideoCapture cam = new VideoCapture("C:/Users/hugoc/Downloads/temp/WIN_20211206_17_44_35_Pro.mp4");
        // set video properties
        // More properties at: https://www.emgu.com/wiki/files/4.5.4/document/html/T_Emgu_CV_CvEnum_CapProp.htm
        // Frames per second
        if (cam.Set(CapProp.Fps, fps) == false) Console.WriteLine("Unable to change FPS");
        // Width
        if (cam.Set(CapProp.FrameWidth, frame_width) == false) Console.WriteLine("Unable to change frame width");
        // Frames per second
        if (cam.Set(CapProp.FrameHeight, frame_height) == false) Console.WriteLine("Unable to change frame height");

#if DEBUG_WINDOWS
        String bgr_wname = "(C#) Color image"; //The name of the window
        String gray_wname = "(C#) Grayscale image"; //The name of the window
        CvInvoke.NamedWindow(bgr_wname); //Create the window using the specific name
        for (i=0; i < NUM_FRAME_BUFFERS; i++)
            CvInvoke.NamedWindow(gray_wname + $" {i}"); //Create the window using the specific name
#endif

        // Acquire first image
        Mat org_bgr_image = cam.QueryFrame();
        Mat org_gray_image = new Mat(frame_height, frame_width, DepthType.Cv8U, 1);
        CvInvoke.CvtColor(org_bgr_image, org_gray_image, ColorConversion.Bgr2Gray);
        int bgr_img_size = org_bgr_image.Step * frame_height,
            gray_img_size = org_gray_image.Step * frame_height;

        // The shared memory will contain:
        // 0 --> byte num_frame_buffers (writen by the producer)
        // 1 -->  byte shared_latest_cam_buffer_idx (writen by the producer)
        // 2 : NUM_FRAME_BUFFERS+1 --> byte shared_buffers_idx_in_use[NUM_FRAME_BUFFERS] (written by the producer and consumers)
        // NUM_FRAME_BUFFERS+2 : NUM_FRAME_BUFFERS+2 + w*h*3 - 1 --> shared_rgb_frame[height, width, 3] (written by the producer)
        // NUM_FRAME_BUFFERS+2 + w*h*3 : NUM_FRAME_BUFFERS+2 + w*h*3 + w*h*NUM_FRAME_BUFFERS --> shared_gray_frame[height, width, NUM_FRAME_BUFFERS] (written by the producer)
        // Create the shared memory with all needed space
        using (MemoryMappedFile mmf = MemoryMappedFile.CreateNew("SharedMemory",
            sizeof(int) // int frame_height
            + sizeof(int) // int frame_width
            + sizeof(int) // int bgr_frame_step
            + sizeof(int) // int gray_frame_step
            + sizeof(byte) // num_shared_buffers
            + sizeof(byte) // byte shared_latest_cam_buffer_idx (writen by the producer)
            + NUM_FRAME_BUFFERS * sizeof(byte) // byte shared_buffers_idx_in_use[NUM_FRAME_BUFFERS] (written by the producer and consumers)
            + bgr_img_size * sizeof(byte) // shared_rgb_frame[height, width, 3] (written by the producer) (not currently being used by anyone)
            + gray_img_size * NUM_FRAME_BUFFERS * sizeof(byte))) // shared_gray_frame[height, width, NUM_FRAME_BUFFERS] (written by the producer)
        using (MemoryMappedViewAccessor accessor_view = mmf.CreateViewAccessor()) // Access entire shared memory
        {
            // Shared memory access pointer
            byte* acc_ptr = null;
            accessor_view.SafeMemoryMappedViewHandle.AcquirePointer(ref acc_ptr);

            int offset = 0;

            // shared_frame_height
            int* shared_frame_height = (int*)acc_ptr;
            // Initialize it 
            *shared_frame_height = frame_height;
            offset += sizeof(int);

            // shared_frame_height
            int* shared_frame_width = (int*)(acc_ptr + offset);
            // Initialize it 
            *shared_frame_width = frame_width;
            offset += sizeof(int);

            // shared_bgr_frame_step
            int* shared_bgr_frame_step = (int*)(acc_ptr + offset);
            // Initialize it 
            *shared_bgr_frame_step = org_bgr_image.Step;
            offset += sizeof(int);

            // shared_bgr_frame_step
            int* shared_gray_frame_step = (int*)(acc_ptr + offset);
            // Initialize it 
            *shared_gray_frame_step = org_gray_image.Step;
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
            Mat bgr_image = new Mat(frame_height, frame_width, DepthType.Cv8U,
                                    3, (IntPtr)(acc_ptr + offset),
                                    org_bgr_image.Step);
            offset += bgr_img_size;

            // Grayscale image access buffers
            Mat[] gray_images = new Mat[NUM_FRAME_BUFFERS];
            for (i = 0; i < NUM_FRAME_BUFFERS; i++)
            {
                gray_images[i] = new Mat(frame_height, frame_width, DepthType.Cv8U,
                                         1, (IntPtr)(acc_ptr + offset),
                                         org_gray_image.Step);
                offset += gray_img_size;
            }

            // Create NUM_PROCESSES named pipes to trigger new image availability (1 per consumer process)
            NamedPipeServerStream[] pipeServers = new NamedPipeServerStream[NUM_PROCESSES];
            for (i = 0; i < NUM_PROCESSES; i++)
            {
                pipeServers[i] = new NamedPipeServerStream(
                    $"pipe{i}", PipeDirection.Out, 1, PipeTransmissionMode.Byte,
                    PipeOptions.Asynchronous | PipeOptions.WriteThrough);
            }

            // Wait for each client to connect
            Task[] pipeServerTasks = new Task[NUM_PROCESSES];
            bool[] pipeConnected = new bool[NUM_PROCESSES];
            for (i = 0; i < NUM_PROCESSES; i++)
            {
                Console.WriteLine($"Waiting for client {i + 1} connection...");
                //pipeServers[i].WaitForConnection();
                pipeServerTasks[i] = pipeServers[i].WaitForConnectionAsync();
                pipeConnected[i] = false;
            }

            // Main, infinite, cycle
            while (true)
            {
                // Check if the clients have already connected
                for (i = 0; i < NUM_PROCESSES; i++)
                {
                    if (pipeConnected[i] == false)
                    {
                        if (pipeServerTasks[i].IsCompletedSuccessfully)
                        {
                            pipeConnected[i] = true;
                            Console.WriteLine($"Client {i} connected.");
                        }
                    }
                }

                // Wait for the mutex to be available.        
                mtx.WaitOne();
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
                    throw new InvalidProgramException("No available buffer index!");
                // Release our mutex. We will capture it again later on
                mtx.ReleaseMutex();

                // Capture and store images. The grayscale image goes to the appropriate buffer
                if (cam.Read(bgr_image) == false)
                {
                    Console.WriteLine("Unable to acquire more frames. Exiting...");
                    break;
                }
                CvInvoke.CvtColor(bgr_image, gray_images[next_cam_buffer_idx], ColorConversion.Bgr2Gray);

                // Updated the latest shared frame buffer index (protected by the mutex)
                mtx.WaitOne();
                *shared_latest_cam_buffer_idx = next_cam_buffer_idx;
                mtx.ReleaseMutex();

                // Message all connectec consumers that a new image is available
                for (i = 0; i < NUM_PROCESSES; i++)
                {
                    if (pipeConnected[i])
                        pipeServers[i].WriteAsync(msg, 0, 1);
                }
#if DEBUG_WINDOWS
                CvInvoke.Imshow(bgr_wname, bgr_image);
                for (i = 0; i < NUM_FRAME_BUFFERS; i++)
                    CvInvoke.Imshow(gray_wname + $" {i}", gray_images[i]);
                int key = CvInvoke.WaitKey(Constants.WAIT_TIME);  //Wait for the key pressing event
                if (key == 'q')
                {
                    Console.WriteLine("Exiting...");
                    break;
                }
#else
                Thread.Sleep(Constants.WAIT_TIME);
#endif
#if DEBUG_FPS
                // Compute the FPS
                num_frames += 1;
                if (num_frames == 100)
                {
                    var endTime = Environment.TickCount;
                    Console.WriteLine($"Producer: {1000 * (num_frames / (1.0 * endTime - startTime)):0.00} FPS");
                    num_frames = 0;
                    startTime = endTime;
                }
#endif
            }
            accessor_view.SafeMemoryMappedViewHandle.ReleasePointer();
            mmf.Dispose();
        }

#if DEBUG_WINDOWS
        CvInvoke.DestroyWindow(bgr_wname);
        for (i = 0; i < NUM_FRAME_BUFFERS; i++)
            CvInvoke.DestroyWindow(gray_wname + $" {i}");
#endif
        mtx.Dispose();
    }
}