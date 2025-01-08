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

#include <mutex>
#include <metavision/sdk/core/utils/mostrecent_timestamp_buffer.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>

// #include <CL/cl.h>
// #include <CL/cl.hpp>

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

// Structure to store coordinate information
typedef struct
{
    int x;
    int y;
    int count;
} CoordinateInfo;

// Function to check if coordinate already exists in array
int findCoordinate(CoordinateInfo *coords, int size, int x, int y)
{
    for (int i = 0; i < size; i++)
    {
        if (coords[i].x == x && coords[i].y == y)
        {
            return i;
        }
    }
    return -1;
}

void analyzeCoordinates(int *data)
{
    // Allocate memory for storing unique coordinates
    int maxUniqueCoords = ARRAY_SIZE;
    CoordinateInfo *coords = (CoordinateInfo *)malloc(maxUniqueCoords * sizeof(CoordinateInfo));
    int uniqueCount = 0;

    // Process each coordinate in the flattened array
    for (int i = 0; i < ARRAY_SIZE; i += 2)
    {
        int x = data[i];
        int y = data[i + 1];
        // Check if coordinate exists
        int existingIndex = findCoordinate(coords, uniqueCount, x, y);

        if (existingIndex != -1)
        {
            // Increment count if coordinate already exists
            coords[existingIndex].count++;
        }
        else
        {
            // Add new coordinate
            coords[uniqueCount].x = x;
            coords[uniqueCount].y = y;
            coords[uniqueCount].count = 1;
            uniqueCount++;
        }
    }

    // Print results
    printf("\nUnique Coordinates:%d\n", uniqueCount);

    // for (int i = 0; i < uniqueCount; i++)
    // {
    //     if (coords[i].count == 1)
    //     {
    //         printf("(%d, %d)\n", coords[i].x, coords[i].y);
    //     }
    // }

    // printf("\nRepeated Coordinates and their counts:\n");
    // for (int i = 0; i < uniqueCount; i++)
    // {
    //     if (coords[i].count > 1)
    //     {
    //         printf("(%d, %d) - Count: %d\n", coords[i].x, coords[i].y, coords[i].count);
    //     }
    // }

    // Free allocated memory
    free(coords);
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
    int data[ARRAY_SIZE * 2];

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

    cv::Mat heatmap, time_surface_gray;

    Metavision::MostRecentTimestampBuffer time_surface(camera_height, camera_width, 1);
    time_surface.set_to(0);

    // we use a mutex to control concurrent accesses to the time surface
    std::mutex frame_mutex;

    // create a variable where to store the latest timestamp
    Metavision::timestamp last_time = 0;

    int delta_ts = 10000;

    reslicer.set_on_new_slice_callback([&](Metavision::EventBufferReslicerAlgorithm::ConditionStatus status,
                                           Metavision::timestamp ts, std::size_t nevents)
                                       {
                                           // analyzeCoordinates(data);
                                           if (!time_surface.empty())
                                           {
                                               // std::cout << time_surface.rows() << std::endl;
                                               // std::cout << time_surface.cols() << std::endl;
                                               // std::cout << time_surface.channels() << std::endl;

                                               std::cout << " time surface size : "<<time_surface.size() << std::endl;

                                               // std::cout << time_surface.at(0, 0) << std::endl;

                                               std::unique_lock<std::mutex> lock(frame_mutex);
                                               // generate the time surface from MostRecentTimestampBuffer
                                               time_surface.generate_img_time_surface(last_time, delta_ts, time_surface_gray);
                                               // apply a colormap to the time surface and display the new frame
                                               // cv::applyColorMap(time_surface_gray, heatmap, cv::COLORMAP_JET);
                                               // window.show(heatmap);
                                               std::cout << time_surface_gray.size() << std::endl;
                                                  cv::imshow("Time Surface", time_surface_gray); 
                                                  cv::waitKey(20);
                                            //    window.show(time_surface_gray);
                                           }
                                       });

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
            // create a MostRecentTimestampBuffer to store the last timestamp of each pixel and initialize all elements to zero
            std::unique_lock<std::mutex> lock(frame_mutex);
            time_surface.at(ev->y, ev->x) = ev->t;
            last_time = ev->t;
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
        // if (!time_surface.empty()){
        //     std::unique_lock<std::mutex> lock(frame_mutex);
        //     cv::imshow("Time Surface", time_surface_gray);
        //     while(cv::waitKey(1) != 27) {
        //         // std::cout << "Waiting for key press" << std::endl;
        //     }
        //     lock.unlock();

        // }
        
        std::this_thread::yield();
    }

    // the recording is finished, stop the camera.
    // Note: we will never get here with a live camera
    cam.stop();

    return 0;

    // return 0;
}
