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

#define _CRT_SECURE_NO_WARNINGS
#define PROGRAM_FILE "reduction.cl"

#define ARRAY_SIZE 32768
#define NUM_KERNELS 2

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
    bool enable_histo, enable_diff, diff_allow_rollover, histo_packed = 0, disable_display;
    unsigned int histo_bit_size_neg = 4, histo_bit_size_pos = 4, diff_bit_size = 8;
    int nevents = ARRAY_SIZE;

    char file_name[100];
    int frame_count = 0;
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

    /* OpenCL structures */
    cl_device_id device;
    cl_context context;
    cl_program program;
    cl_kernel kernel[NUM_KERNELS];
    cl_command_queue queue;
    cl_event prof_event;
    cl_int i, j, err;
    size_t local_size, global_size;
    char kernel_names[NUM_KERNELS][20] =
        {"reduction_scalar", "reduction_vector"};

    /* Data and buffers */
    float data[ARRAY_SIZE];
    float sum, actual_sum, *scalar_sum, *vector_sum;
    cl_mem data_buffer, scalar_sum_buffer, vector_sum_buffer;
    cl_int num_groups;
    cl_ulong time_start, time_end, total_time;

    /* Initialize data */
    for (i = 0; i < ARRAY_SIZE; i++)
    {
        data[i] = 1.0f * i;
    }

    /* Create device and determine local size */
    device = create_device();
    err = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE,
                          sizeof(local_size), &local_size, NULL);
    printf("local size : %d \n", local_size);
    if (err < 0)
    {
        perror("Couldn't obtain device information");
        exit(1);
    }

    /* Allocate and initialize output arrays */
    num_groups = ARRAY_SIZE / local_size;
    scalar_sum = (float *)malloc(num_groups * sizeof(float));
    vector_sum = (float *)malloc(num_groups / 4 * sizeof(float));
    for (i = 0; i < num_groups; i++)
    {
        scalar_sum[i] = 0.0f;
    }
    for (i = 0; i < num_groups / 4; i++)
    {
        vector_sum[i] = 0.0f;
    }

    /* Create a context */
    context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
    if (err < 0)
    {
        perror("Couldn't create a context");
        exit(1);
    }

    /* Build program */
    program = build_program(context, device, PROGRAM_FILE);

    /* Create data buffer */
    data_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(float), data, &err);
    scalar_sum_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, num_groups * sizeof(float), scalar_sum, &err);
    vector_sum_buffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, num_groups * sizeof(float), vector_sum, &err);
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

    for (i = 0; i < NUM_KERNELS; i++)
    {

        /* Create a kernel */
        kernel[i] = clCreateKernel(program, kernel_names[i], &err);
        if (err < 0)
        {
            perror("Couldn't create a kernel");
            exit(1);
        };

        /* Create kernel arguments */
        err = clSetKernelArg(kernel[i], 0, sizeof(cl_mem), &data_buffer);
        if (i == 0)
        {
            global_size = ARRAY_SIZE;
            err |= clSetKernelArg(kernel[i], 1, local_size * sizeof(float), NULL);
            err |= clSetKernelArg(kernel[i], 2, sizeof(cl_mem), &scalar_sum_buffer);
        }
        else
        {
            global_size = ARRAY_SIZE / 4;
            err |= clSetKernelArg(kernel[i], 1, local_size * 4 * sizeof(float), NULL);
            err |= clSetKernelArg(kernel[i], 2, sizeof(cl_mem), &vector_sum_buffer);
        }
        if (err < 0)
        {
            perror("Couldn't create a kernel argument");
            exit(1);
        }

        /* Enqueue kernel */
        err = clEnqueueNDRangeKernel(queue, kernel[i], 1, NULL, &global_size,
                                     &local_size, 0, NULL, &prof_event);
        if (err < 0)
        {
            perror("Couldn't enqueue the kernel");
            exit(1);
        }

        /* Finish processing the queue and get profiling information */
        clFinish(queue);
        clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_START,
                                sizeof(time_start), &time_start, NULL);
        clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END,
                                sizeof(time_end), &time_end, NULL);
        total_time = time_end - time_start;

        /* Read the result */
        if (i == 0)
        {
            err = clEnqueueReadBuffer(queue, scalar_sum_buffer, CL_TRUE, 0,
                                      num_groups * sizeof(float), scalar_sum, 0, NULL, NULL);
            if (err < 0)
            {
                perror("Couldn't read the buffer");
                exit(1);
            }
            sum = 0.0f;
            for (j = 0; j < num_groups; j++)
            {
                sum += scalar_sum[j];
            }
            std::cout << "scalar sum : " << sum << std::endl;
        }
        else
        {
            err = clEnqueueReadBuffer(queue, vector_sum_buffer, CL_TRUE, 0,
                                      num_groups / 4 * sizeof(float), vector_sum, 0, NULL, NULL);
            if (err < 0)
            {
                perror("Couldn't read the buffer");
                exit(1);
            }
            sum = 0.0f;
            for (j = 0; j < num_groups / 4; j++)
            {
                sum += vector_sum[j];
            }
            std::cout << "vector sum : " << sum << std::endl;
        }

        /* Check result */
        printf("%s: ", kernel_names[i]);
        actual_sum = 1.0f * ARRAY_SIZE / 2 * (ARRAY_SIZE - 1);
        for (int i = 0; i < ARRAY_SIZE; i++)
        {
            actual_sum += data[i];
        }
        std::cout << "actual sum : " << actual_sum << std::endl;
        if (fabs(sum - actual_sum) > 0.01 * fabs(sum))
            printf("Check failed.\n");
        else
            printf("Check passed.\n");
        printf("Total time = %lu\n\n", total_time);

        /* Deallocate event */
        clReleaseEvent(prof_event);
    }

    // Instantiate event reslicer and define its slicing & event callbacks
    Metavision::EventBufferReslicerAlgorithm::Condition condition;
    condition = Metavision::EventBufferReslicerAlgorithm::Condition::make_n_events(nevents);
    Metavision::EventBufferReslicerAlgorithm reslicer(nullptr, condition);

    int data_index = 0;

    reslicer.set_on_new_slice_callback([&](Metavision::EventBufferReslicerAlgorithm::ConditionStatus status,
                                           Metavision::timestamp ts, std::size_t nevents)
                                       {
                                           // print_timestamp_main("start of reslicing ", "main: ");
                                           std::cout << "number of events : " << nevents << std::endl;
                                           // push event data to queue
                                              std::cout << "data values : "  << data[4095] << std::endl;
                                           data_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, ARRAY_SIZE * sizeof(float), data, &err);

                                           for (i = 0; i < NUM_KERNELS; i++)
        {

            /* Create a kernel */
            kernel[i] = clCreateKernel(program, kernel_names[i], &err);
            if (err < 0)
            {
                perror("Couldn't create a kernel");
                exit(1);
            };

            /* Create kernel arguments */
            err = clSetKernelArg(kernel[i], 0, sizeof(cl_mem), &data_buffer);
            if (i == 0)
            {
                global_size = ARRAY_SIZE;
                err |= clSetKernelArg(kernel[i], 1, local_size * sizeof(float), NULL);
                err |= clSetKernelArg(kernel[i], 2, sizeof(cl_mem), &scalar_sum_buffer);
            }
            else
            {
                global_size = ARRAY_SIZE / 4;
                err |= clSetKernelArg(kernel[i], 1, local_size * 4 * sizeof(float), NULL);
                err |= clSetKernelArg(kernel[i], 2, sizeof(cl_mem), &vector_sum_buffer);
            }
            if (err < 0)
            {
                perror("Couldn't create a kernel argument");
                exit(1);
            }

            /* Enqueue kernel */
            err = clEnqueueNDRangeKernel(queue, kernel[i], 1, NULL, &global_size,
                                         &local_size, 0, NULL, &prof_event);
            if (err < 0)
            {
                perror("Couldn't enqueue the kernel");
                exit(1);
            }

            /* Finish processing the queue and get profiling information */
            clFinish(queue);
            clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_START,
                                    sizeof(time_start), &time_start, NULL);
            clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END,
                                    sizeof(time_end), &time_end, NULL);
            total_time = time_end - time_start;

            /* Read the result */
            if (i == 0)
            {
                err = clEnqueueReadBuffer(queue, scalar_sum_buffer, CL_TRUE, 0,
                                          num_groups * sizeof(float), scalar_sum, 0, NULL, NULL);
                if (err < 0)
                {
                    perror("Couldn't read the buffer");
                    exit(1);
                }
                sum = 0.0f;
                for (j = 0; j < num_groups; j++)
                {
                    sum += scalar_sum[j];
                }
                std::cout << "scalar sum: " << sum << std::endl;
            }
            else
            {
                err = clEnqueueReadBuffer(queue, vector_sum_buffer, CL_TRUE, 0,
                                          num_groups / 4 * sizeof(float), vector_sum, 0, NULL, NULL);
                if (err < 0)
                {
                    perror("Couldn't read the buffer");
                    exit(1);
                }
                sum = 0.0f;
                for (j = 0; j < num_groups / 4; j++)
                {
                    sum += vector_sum[j];
                }

                std::cout << "vector sum: " << sum << std::endl;
            }

            // actual_sum = 1.0f * ARRAY_SIZE / 2 * (ARRAY_SIZE - 1);
            actual_sum = 0.0f;

            std::cout << "Time for calculating sum with for loop " << std::endl;

            print_timestamp_main("before for lood array sum", "resliceer callback : ");

            for (int i=0; i<ARRAY_SIZE ; i++){
            actual_sum += data[i];
            }
            print_timestamp_main("after for lood array sum", "resliceer callback : ");

            std::cout << "actual sum: " << actual_sum << std::endl;


            /* Check result */
            printf("%s: ", kernel_names[i]);
            

            

            if (fabs(sum - actual_sum) > 0.01 * fabs(sum))
            printf("Check failed.\n");
            else
            printf("Check passed.\n");
            printf("Total time = %lu\n\n", total_time);

            /* Deallocate event */
            clReleaseEvent(prof_event);
        }


                                           if (data_index > 4095)
                                           {
                                               data_index = 0;
                                           } });

    auto aggregate_events_fct = [&](const Metavision::EventCD *begin, const Metavision::EventCD *end)
    {
        int counter = 0;

        // this loop allows us to get access to each event received in this callback
        for (const Metavision::EventCD *ev = begin; ev != end; ++ev)
        {
            ++counter; // count each event

            // print each event
            // std::cout << "Event received: coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
            //   << ", polarity: " << ev->p << std::endl;
            data[data_index] = ev->x;

            if (data_index==0) {
                std::cout << "First Event received : coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
              << ", polarity: " << ev->p << std::endl;
            } 
            
            data_index++;
            // if (data_index > 4095)
            // {
            //     data_index = 0;
            // }
            
            if (data_index==4095) {
                std::cout << "Last Event received : coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
              << ", polarity: " << ev->p << std::endl;
            } 
        }

        // report
        // std::cout << "There were " << counter << " events in this callback" << std::endl;
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
    /* Deallocate resources */
    /* Deallocate resources */
    free(scalar_sum);
    free(vector_sum);
    for (i = 0; i < NUM_KERNELS; i++)
    {
        clReleaseKernel(kernel[i]);
    }
    clReleaseMemObject(scalar_sum_buffer);
    clReleaseMemObject(vector_sum_buffer);
    clReleaseMemObject(data_buffer);
    clReleaseCommandQueue(queue);
    clReleaseProgram(program);
    clReleaseContext(context);
    // return 0;
}
