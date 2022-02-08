// Define this if you whish to see the captured and shared image windows
#define DEBUG_WINDOWS
//#undef DEBUG_WINDOWS

// Define this if you wish to see the FPS being shared
#define DEBUG_FPS
//#undef DEBUG_FPS

using System.IO.Pipes;
using System.IO.MemoryMappedFiles;

using Emgu.CV;
using Emgu.CV.CvEnum;
using System.Threading;
using System;

namespace CSharpAppConsumer
{
    class Program
    {
        static unsafe void Main(string[] args)
        {
            int MAX_NUM_BYTES = 200;
            byte[] msg = new byte[MAX_NUM_BYTES];
            byte i;

            Mutex mtx = new Mutex(false, "ARMutex");

            // Open the shared memory
            using (MemoryMappedFile mmf = MemoryMappedFile.OpenExisting("SharedMemory"))
            using (MemoryMappedViewAccessor accessor_view = mmf.CreateViewAccessor()) // Access entire shared memory
            using (NamedPipeClientStream pipeClient =
               new NamedPipeClientStream(".", "pipe0", PipeDirection.In))
            {
                int offset = 0;

                // Shared memory access pointer
                byte* acc_ptr = null;
                accessor_view.SafeMemoryMappedViewHandle.AcquirePointer(ref acc_ptr);

                // shared_frame_height
                int* shared_frame_height = (int*)acc_ptr;
                // Initialize it 
                int frame_height = *shared_frame_height;
                offset += sizeof(int);

                // shared_frame_height
                int* shared_frame_width = (int*)(acc_ptr + offset);
                // Initialize it 
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
                Mat bgr_image = new Mat(frame_height, frame_width, DepthType.Cv8U,
                                        3, (IntPtr)(acc_ptr + offset),
                                        *shared_bgr_frame_step);
                offset += frame_height * (*shared_bgr_frame_step);

                // Grayscale image access buffers
                Mat[] gray_images = new Mat[NUM_FRAME_BUFFERS];
                for (i = 0; i < NUM_FRAME_BUFFERS; i++)
                {
                    gray_images[i] = new Mat(frame_height, frame_width, DepthType.Cv8U,
                                             1, (IntPtr)(acc_ptr + offset),
                                             *shared_gray_frame_step);
                    offset += frame_height * (*shared_gray_frame_step);
                }

                // Connect to the pipe or wait until the pipe is available.
                Console.Write("Attempting to connect to our pipe...");
                pipeClient.Connect();
                Console.WriteLine("Connected to pipe.");
#if DEBUG_WINDOWS
                String bgr_wname = "(C#) Color image"; //The name of the window
                String gray_wname = "(C#) Grayscale image"; //The name of the window
                CvInvoke.NamedWindow(bgr_wname);
                CvInvoke.NamedWindow(gray_wname);
#endif
                // Initialize with "impossible value"
                byte frame_idx = NUM_FRAME_BUFFERS;
                int num_frames = 0; // For FPS estimate
                var startTime = Environment.TickCount;
                while (true)
                {

                    int num_bytes_read = pipeClient.Read(msg, 0, MAX_NUM_BYTES);
                    if (num_bytes_read == 0)
                        Console.WriteLine("No bytes read!");
                    else if (num_bytes_read == MAX_NUM_BYTES)
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
#if DEBUG_WINDOWS
                        // Show images for debugging purposes
                        CvInvoke.Imshow(bgr_wname, bgr_image);
                        CvInvoke.Imshow(gray_wname, gray_images[frame_idx]);
                        if (CvInvoke.WaitKey(5) == 'q')
                            break;
#endif
#if DEBUG_FPS
                        // Compute the FPS
                        num_frames += 1;
                        if (num_frames == 100)
                        {
                            var endTime = Environment.TickCount;
                            Console.WriteLine($"Process 0: {num_frames * 1000.0 / (endTime - startTime):0.00} FPS");
                            num_frames = 0;
                            startTime = endTime;
                        }
#endif
                    }
                }
            }
            mtx.Dispose();
        }
    }
}
