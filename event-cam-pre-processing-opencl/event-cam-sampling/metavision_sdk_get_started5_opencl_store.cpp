#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/events/event_cd.h>

#include <fstream>
#include <functional>
#include <thread>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>
#include <metavision/sdk/core/algorithms/event_frame_diff_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/event_frame_histo_generation_algorithm.h>
#include <metavision/hal/facilities/i_hw_identification.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>

#include <CL/cl.h>
#include <CL/cl.hpp>

#define _CRT_SECURE_NO_WARNINGS
#define PROGRAM_FILE "coordinate_processor.cl"

// #define ARRAY_SIZE 4096
#define NUM_KERNELS 1

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Maximum number of coordinates to process
const int ARRAY_SIZE = 16384;
const int MAX_COORDINATES = 10000;
const int WIDTH = 1280;
const int HEIGHT = 720;
const int MAX_HASH_SIZE = 16384; // Should be power of 2, larger than expected unique coordinates

/* Find a GPU or CPU associated with the first available platform

The `platform` structure identifies the first platform identified by the
OpenCL runtime. A platform identifies a vendor's installation, so a system
may have an NVIDIA platform and an AMD platform.

The `device` structure corresponds to the first accessible device
associated with the platform. Because the second parameter is
`CL_DEVICE_TYPE_GPU`, this device must be a GPU.
*/
/* Find a GPU or CPU associated with the first available platform */
cl_device_id create_device()
{

    cl_platform_id platform;
    cl_device_id dev;
    int err;

    /* Identify a platform */
    err = clGetPlatformIDs(1, &platform, NULL);
    if (err < 0)
    {
        perror("Couldn't identify a platform");
        exit(1);
    }

    /* Access a device */
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &dev, NULL);
    if (err == CL_DEVICE_NOT_FOUND)
    {
        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 1, &dev, NULL);
    }
    if (err < 0)
    {
        perror("Couldn't access any devices");
        exit(1);
    }

    return dev;
}

/* Create program from a file and compile it */
cl_program build_program(cl_context ctx, cl_device_id dev, const char *filename)
{

    cl_program program;
    FILE *program_handle;
    char *program_buffer, *program_log;
    size_t program_size, log_size;
    int err;

    /* Read program file and place content into buffer */
    program_handle = fopen(filename, "r");
    if (program_handle == NULL)
    {
        perror("Couldn't find the program file");
        exit(1);
    }
    fseek(program_handle, 0, SEEK_END);
    program_size = ftell(program_handle);
    rewind(program_handle);
    program_buffer = (char *)malloc(program_size + 1);
    program_buffer[program_size] = '\0';
    fread(program_buffer, sizeof(char), program_size, program_handle);
    fclose(program_handle);

    /* Create program from file */
    program = clCreateProgramWithSource(ctx, 1,
                                        (const char **)&program_buffer, &program_size, &err);
    if (err < 0)
    {
        perror("Couldn't create the program");
        exit(1);
    }
    free(program_buffer);

    /* Build program */
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err < 0)
    {

        /* Find size of log and print to std output */
        clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
                              0, NULL, &log_size);
        program_log = (char *)malloc(log_size + 1);
        program_log[log_size] = '\0';
        clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
                              log_size + 1, program_log, NULL);
        printf("%s\n", program_log);
        free(program_log);
        exit(1);
    }

    return program;
}
// this function will be associated to the camera callback
void count_events(const Metavision::EventCD *begin, const Metavision::EventCD *end)
{
    int counter = 0;

    // this loop allows us to get access to each event received in this callback
    for (const Metavision::EventCD *ev = begin; ev != end; ++ev)
    {
        ++counter; // count each event

        // print each event
        std::cout << "Event received: coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
                  << ", polarity: " << ev->p << std::endl;
    }

    // report
    std::cout << "There were " << counter << " events in this callback" << std::endl;
}

void print_timestamp_main(char *info, char *func)
{
    char print_info[20];

    auto now_b = std::chrono::system_clock::now();

    // Convert the current time to time since epoch
    auto duration_b = now_b.time_since_epoch();

    // Convert duration to milliseconds
    auto milliseconds_b = std::chrono::duration_cast<std::chrono::microseconds>(
                              duration_b)
                              .count();

    // Print the result
    std::cout << func << " : Current time in milliseconds in " << info << "call is: "
              << milliseconds_b << std::endl;
}

// main loop
int main(int argc, char *argv[])
{
    // std::string event_file_path, out_file_path, out_video_path;

    int nevents = ARRAY_SIZE;

    char file_name[100];
    int frame_count = 0;

    /* OpenCL structures */
    cl_device_id device;
    cl_context context;
    cl_program program;
    cl_kernel kernel;
    cl_command_queue queue;
    cl_event prof_event;
    cl_int i, j, err;
    size_t local_size, global_size;
    cl_ulong time_start, time_end, total_time;
    char kernel_names[NUM_KERNELS][24] =
        {"process_coordinates"};

    /* Data and buffers */
    int data[ARRAY_SIZE];
    printf("Data stored\n");
    for (int i = 0; i < ARRAY_SIZE; i++)
    {

        data[i] = 0;
    }
    printf("\n");

    /* Create device and determine local size */
    device = create_device();
    err = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE,
                          sizeof(local_size), &local_size, NULL);
    printf("local size : %d \n", local_size);
    local_size = 1024;
    if (err < 0)
    {
        perror("Couldn't obtain device information");
        exit(1);
    }

    /* Allocate and initialize output arrays */
    // num_groups = ARRAY_SIZE / local_size;

    // assign = (unsigned int *)malloc(32 * sizeof(unsigned int));
    // scalar_sum = (float *)malloc(num_groups * sizeof(float));

    /* Create a context */
    context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
    if (err < 0)
    {
        perror("Couldn't create a context");
        exit(1);
    }

    /* Build program */
    program = build_program(context, device, PROGRAM_FILE);

    // Prepare output buffers
    std::vector<int> repeatedCoords(MAX_COORDINATES);
    std::vector<int> uniqueCoords(MAX_COORDINATES);
    int repeatedCount = 0;
    int uniqueCount = 0;

    // Create OpenCL buffers
    // cl::Buffer inputBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
    //                        coordinates.size() * sizeof(int), coordinates.data());
    // cl::Buffer repeatedBuffer(context, CL_MEM_WRITE_ONLY,
    //                           repeatedCoords.size() * sizeof(int));
    // cl::Buffer uniqueBuffer(context, CL_MEM_WRITE_ONLY,
    //                         uniqueCoords.size() * sizeof(int));
    // cl::Buffer repeatedCountBuffer(context, CL_MEM_WRITE_ONLY, sizeof(int));
    // cl::Buffer uniqueCountBuffer(context, CL_MEM_READ_WRITE, sizeof(int));
    cl_mem inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(int), data, &err);
    cl_mem repeatedBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, ARRAY_SIZE * sizeof(int), NULL, &err);
    cl_mem uniqueBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, ARRAY_SIZE * sizeof(int), NULL, &err);
    cl_mem repeatedCountBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(int), NULL, &err);
    cl_mem uniqueCountBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &err);

    if (err < 0)
    {
        perror("Couldn't create a buffer");
        exit(1);
    };

    /* Create a command queue */
    queue = clCreateCommandQueue(context, device,
                                 CL_QUEUE_PROFILING_ENABLE, &err);
    if (err < 0)
    {
        perror("Couldn't create a command queue");
        exit(1);
    };

    kernel = clCreateKernel(program, kernel_names[0], &err);
    if (err < 0)
    {
        perror("Couldn't create a kernel");
        exit(1);
    };

    // Set kernel arguments
    // kernel.setArg(0, inputBuffer);
    // kernel.setArg(1, repeatedBuffer);
    // kernel.setArg(2, uniqueBuffer);
    // kernel.setArg(3, repeatedCountBuffer);
    // kernel.setArg(4, uniqueCountBuffer);
    // kernel.setArg(5, ARRAY_SIZE);
    // kernel.setArg(6, WIDTH);
    // kernel.setArg(7, HEIGHT);
    clSetKernelArg(kernel, 0, sizeof(cl_mem), &inputBuffer);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &repeatedBuffer);
    clSetKernelArg(kernel, 2, sizeof(cl_mem), &uniqueBuffer);
    clSetKernelArg(kernel, 3, sizeof(cl_mem), &repeatedCountBuffer);
    clSetKernelArg(kernel, 4, sizeof(cl_mem), &uniqueCountBuffer);
    clSetKernelArg(kernel, 5, sizeof(int), &ARRAY_SIZE);
    clSetKernelArg(kernel, 6, sizeof(int), &WIDTH);
    clSetKernelArg(kernel, 7, sizeof(int), &HEIGHT);

    // Execute kernel
    cl::NDRange globalSize(1024); // Adjust based on your device capabilities
    cl::NDRange localSize(1024);  // Adjust based on device local work group size
    global_size = 1024;
    local_size = 1024;
    // queue.enqueueNDRangeKernel(kernel, cl::NullRange, globalSize, localSize);

    err = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &global_size,
                                 &local_size, 0, NULL, &prof_event);
    if (err < 0)
    {
        perror("Couldn't enqueue the kernel");
        exit(1);
    }

    /* Finish processing the queue and get profiling information */
    clFinish(queue);

    /**************end of OpenCL kernel config ************************/
    Metavision::timestamp period_us = 50000, min_generation_period_us = 10000;

    Metavision::Camera cam; // create the camera

    if (argc >= 2)
    {
        // if we passed a file path, open it
        cam = Metavision::Camera::from_file(argv[1]);
    }
    else
    {
        // open the first available camera
        cam = Metavision::Camera::from_first_available();
    }

    const int camera_width = cam.geometry().width();
    const int camera_height = cam.geometry().height();
    const int npixels = camera_width * camera_height;

    // Instantiate event reslicer and define its slicing & event callbacks
    Metavision::EventBufferReslicerAlgorithm::Condition condition;
    condition = Metavision::EventBufferReslicerAlgorithm::Condition::make_n_events(nevents);
    Metavision::EventBufferReslicerAlgorithm reslicer(nullptr, condition);

    int data_index = 0;

    reslicer.set_on_new_slice_callback([&](Metavision::EventBufferReslicerAlgorithm::ConditionStatus status,
                                           Metavision::timestamp ts, std::size_t nevents)
                                       {
                                           // print_timestamp_main("start of reslicing ", "main: ");
                                           //    std::cout << "number of events : " << nevents << std::endl;
                                           //    // push event data to queue
                                           //    for (int i = 4000; i < 4095; i++)
                                           //    {
                                           //        std::cout << "," << data[i];
                                           //    }
                                           //    std::cout << std::endl;

                                           //    data_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(float), data, &err);
                                        //   inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(int), data, &err);
                                          printf("data at index 16383 : %d\n", data[16383]);
                                          inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(int), data, &err);
                                          if (err < 0)
                                            {
                                                perror("Couldn't create a buffer");
                                                exit(1);
                                            };

                                           /* Enqueue kernel */
                                           clSetKernelArg(kernel, 0, sizeof(cl_mem), &inputBuffer);
                                                                                      
                                           err = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &global_size,
                                                                        &local_size, 0, NULL, &prof_event);
                                       
                                           if (err < 0)
                                           {
                                               perror("Couldn't enqueue the kernel");
                                               exit(1);
                                           }

                                           /* Finish processing the queue and get profiling information */
                                           clFinish(queue);

                                        //    // Read back results
                                        //     err = clEnqueueReadBuffer(queue, uniqueBuffer, CL_TRUE, 0, 
                                        //         uniqueCoords.size() * sizeof(int), uniqueCoords.data(), 0, NULL, NULL);

                                        //     // err = clEnqueueReadBuffer(queue, repeatedBuffer, CL_TRUE, 0, 
                                        //     //     repeatedCoords.size() * sizeof(int), repeatedCoords.data(), 0, NULL, NULL);
                                            
                                        //     // err = clEnqueueReadBuffer(queue, repeatedCountBuffer, CL_TRUE, 0, 
                                        //     //     sizeof(int), &repeatedCount, 0, NULL, NULL);

                                        //     err = clEnqueueReadBuffer(queue, uniqueCountBuffer, CL_TRUE, 0,
                                        //         sizeof(int), &uniqueCount, 0, NULL, NULL);

                                        //     if (err < 0)
                                        //     {
                                        //         perror("Couldn't read the buffer");
                                        //         exit(1);
                                        //     }

                                        //     // std::cout << "\nSample Unique Coordinates:\n";
                                        //     // for (int i = 0; i < 2500; i+=2)
                                        //     // {
                                                
                                        //     //     std::cout  << uniqueCoords[i] << ","<< uniqueCoords[i+1] << std::endl;
                                        //     // }

                                           uniqueCount = 0;
                                           printf("finished processing\n");
                                           clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_START,
                                                                    sizeof(time_start), &time_start, NULL);
                                            clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END,
                                                                    sizeof(time_end), &time_end, NULL);
                                             total_time = time_end - time_start;

                                             printf("Total time = %lu\n\n", total_time);

                                        /* Deallocate event */
                                             clReleaseEvent(prof_event);
                                          
                                           /*************Start of cluster assignment kernel**************/ });

    auto aggregate_events_fct = [&](const Metavision::EventCD *begin, const Metavision::EventCD *end)
    {
        int counter = 0;

        // this loop allows us to get access to each event received in this callback
        for (const Metavision::EventCD *ev = begin; ev != end; ++ev)
        {
            ++counter; // count each event
            // if (data_index == 0)
            // {
            //     std::cout << "First Event received : coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
            //               << ", polarity: " << ev->p << std::endl;
            // }

            // print each event
            // std::cout << "Event received: coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
            //   << ", polarity: " << ev->p << std::endl;
            data[data_index] = ev->x;
            // std::cout << "data stored at" << data_index << ":" << data[data_index] << " data x:" << ev->x << std::endl;
            data_index++;
            data[data_index] = ev->y;
            // std::cout << "data stored at" << data_index << ":" << data[data_index] << " data y:" << ev->y << std::endl;

            data_index++;
            if (data_index > ARRAY_SIZE - 1)
            {
                data_index = 0;
                // std::cout << "Last Event received : coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
                //               << ", polarity: " << ev->p << std::endl;
            }

            // if (data_index == 4095)
            // {
            //     std::cout << "Last Event received : coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
            //               << ", polarity: " << ev->p << std::endl;
            // }
        }

        // report
        // std::cout << "There were " << counter << " events in this callback" << std::endl;
        // std::cout << "data_index " << data_index << std::endl;
    };

    // Set the event processing callback
    cam.cd().add_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end)
                          { reslicer.process_events(begin, end, aggregate_events_fct); });

    // start the camera
    cam.start();

    // keep running while the camera is on or the recording is not finished
    while (cam.is_running())
    {
        std::this_thread::yield();
    }

    // the recording is finished, stop the camera.
    // Note: we will never get here with a live camera
    cam.stop();

    /* Deallocate event */
    clReleaseEvent(prof_event);

    clReleaseKernel(kernel);

    // clReleaseMemObject(output_buffer);
    // clReleaseMemObject(data_buffer);
    clReleaseCommandQueue(queue);
    clReleaseProgram(program);
    clReleaseContext(context);
    return 0;

    // return 0;
}
