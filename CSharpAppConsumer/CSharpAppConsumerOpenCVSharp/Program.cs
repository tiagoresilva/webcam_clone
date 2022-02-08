// Define this if you whish to see the captured and shared image windows
//#define DEBUG_WINDOWS
#undef DEBUG_WINDOWS

// Define this if you wish to see the FPS being shared
#define DEBUG_FPS
//#undef DEBUG_FPS

using System.IO.Pipes;
using System.IO.MemoryMappedFiles;

using OpenCvSharp;
using System;
using System.Threading;


namespace CSharpAppConsumer
{
    class Program
    {
        unsafe static void ImgProcess(object proc_num_arg)
        {
            int MAX_NUM_BYTES = 256;
            byte[] msg = new byte[MAX_NUM_BYTES];
            byte i;
            int proc_num = (int) proc_num_arg;

            var aruco_parameters = OpenCvSharp.Aruco.DetectorParameters.Create();
            OpenCvSharp.Aruco.Dictionary aruco_dict;
            if (proc_num == 0)  // Process 0
            {
                aruco_dict = OpenCvSharp.Aruco.CvAruco.GetPredefinedDictionary(
                    OpenCvSharp.Aruco.PredefinedDictionaryName.Dict6X6_250);
            } else  // Process 1
            {
                // Set dictionary size depending on the aruco marker selected
                aruco_parameters.AdaptiveThreshConstant = 10;
                aruco_parameters.MinCornerDistanceRate = 0.005;
                aruco_parameters.MinMarkerDistanceRate = 0.005;
                aruco_parameters.PolygonalApproxAccuracyRate = 0.01;
                aruco_parameters.CornerRefinementMethod = OpenCvSharp.Aruco.CornerRefineMethod.Subpix;
                aruco_parameters.CornerRefinementWinSize = 1;
                aruco_parameters.CornerRefinementMaxIterations = 100;
                aruco_parameters.MaxErroneousBitsInBorderRate = 0.1;
                // Set dictionary size depending on the aruco m arker selected
                aruco_dict = OpenCvSharp.Aruco.CvAruco.GetPredefinedDictionary(
                    OpenCvSharp.Aruco.PredefinedDictionaryName.Dict4X4_250);
            }

            // Open the shared memory
            using (MemoryMappedFile mmf = MemoryMappedFile.OpenExisting("SharedMemory"))
            using (MemoryMappedViewAccessor accessor_view = mmf.CreateViewAccessor()) // Access entire shared memory
            using (NamedPipeClientStream pipeClient =
               new NamedPipeClientStream(".", $"pipe{proc_num}", PipeDirection.In))
            {
                int offset = 0;

                // Shared memory access pointer
                byte* acc_ptr = null;
                accessor_view.SafeMemoryMappedViewHandle.AcquirePointer(ref acc_ptr);

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
                byte* num_frame_buffers = acc_ptr + offset;
                // Store value
                byte NUM_FRAME_BUFFERS = *num_frame_buffers;
                offset += sizeof(byte);

                // shared_latest_cam_buffer_idx
                byte* shared_latest_cam_buffer_idx = acc_ptr + offset;
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
                Mat bgr_image = new Mat(frame_height, frame_width, MatType.CV_8UC3,
                                        (IntPtr)(acc_ptr + offset),
                                        *shared_bgr_frame_step);
                offset += frame_height * (*shared_bgr_frame_step);

                // Grayscale image access buffers
                Mat[] gray_images = new Mat[NUM_FRAME_BUFFERS];
                for (i = 0; i < NUM_FRAME_BUFFERS; i++)
                {
                    gray_images[i] = new Mat(frame_height, frame_width, MatType.CV_8UC1,
                                             (IntPtr)(acc_ptr + offset),
                                             *shared_gray_frame_step);
                    offset += frame_height * (*shared_gray_frame_step);
                }

                // Connect to the pipe or wait until the pipe is available.
                Console.Write("Attempting to connect to our pipe...");
                pipeClient.Connect();
                Console.WriteLine("Connected to pipe.");
#if DEBUG_WINDOWS
                String bgr_wname = $"(C#-Consumer {proc_num}) Color image"; //The name of the window
                Window bgr_window = new Window(bgr_wname);
                String gray_wname = $"(C#-Consumer {proc_num}) Grayscale image"; //The name of the window
                Window gray_window = new Window(gray_wname);
#endif
                // Get our shared mutex
                Mutex mtx = new Mutex(false, "ARMutex");

                // Initialize with "impossible value"
                byte frame_idx = NUM_FRAME_BUFFERS;
#if DEBUG_FPS
                int num_frames = 0; // For FPS estimate
                var startTime = Environment.TickCount;
#endif
                while (true)
                {

                    int num_bytes_read = pipeClient.Read(msg, 0, MAX_NUM_BYTES);
                    if (num_bytes_read == 0)
                    {
                        Console.WriteLine("No bytes read!");
                        break;
                    } else if (num_bytes_read == MAX_NUM_BYTES)
                        Console.Write("Not all bytes might have beed read!");
                    else
                    {
                        mtx.WaitOne();
                        // We are not using the last image anymore)
                        if (frame_idx != NUM_FRAME_BUFFERS)
                            shared_buffers_idx_in_use[frame_idx]--;
                        // Get new index
                        frame_idx = *shared_latest_cam_buffer_idx;
                        // Update number of users of the buffer
                        shared_buffers_idx_in_use[frame_idx]++;
                        mtx.ReleaseMutex();

                        // Process images
                        for (i = 0; i < 10; i++)
                            OpenCvSharp.Aruco.CvAruco.DetectMarkers(
                                gray_images[frame_idx],
                                aruco_dict,
                                out var corners,
                                out var ids,
                                aruco_parameters,
                                out var rejectedImgPoints);
#if DEBUG_WINDOWS
                        // Show images for debugging purposes
                        bgr_window.ShowImage(bgr_image);
                        gray_window.ShowImage(gray_images[frame_idx]);
                        if (Cv2.WaitKey(5) == 'q')
                            break;
#endif
#if DEBUG_FPS
                        // Compute the FPS
                        num_frames += 1;
                        if (num_frames == 100)
                        {
                            var endTime = Environment.TickCount;
                            Console.WriteLine($"Process {proc_num}: {num_frames * 1000.0 / (endTime - startTime):0.00} FPS");
                            num_frames = 0;
                            startTime = endTime;
                        }
#endif
                    }
                }
                mtx.Dispose();
            }
        }

        static unsafe void Main(string[] args)
        {
            // Create threads
            // We are using static methods here, but we could use methods from
            // a specific instance.
            Thread proc0 = new Thread(Program.ImgProcess);
            //Thread proc1 = new Thread(Program.ImgProcess);

            // Start the threads
            proc0.Start(0);
            //proc1.Start(1);

            // Wait for both threads to finish
            proc0.Join();
            //proc1.Join();
        }
    }
}
