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

#include <vector>
#include <algorithm>
#include <cmath>




int circle3_[16][2] = {{0, 3}, {1, 3}, {2, 2}, {3, 1},
            {3, 0}, {3, -1}, {2, -2}, {1, -3},
            {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
            {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};
int circle4_[20][2] = {{0, 4}, {1, 4}, {2, 3}, {3, 2},
            {4, 1}, {4, 0}, {4, -1}, {3, -2},
            {2, -3}, {1, -4}, {0, -4}, {-1, -4},
            {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
            {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}};

// Maximum number of coordinates to process
const int ARRAY_SIZE = 16384;
const int MAX_COORDINATES = 10000;
const int WIDTH = 1280;
const int HEIGHT = 720;
const int MAX_HASH_SIZE = 16384; // Should be power of 2, larger than expected unique coordinates

#include <vector>
#include <algorithm>


#include <unordered_map>

#include <limits>


struct Corner {
    int x;
    int y;
    // float response;  // Corner response/strength
    int label;  // Label of the corner
};

class CornerFilter {
public:
    /**
     * Filter corners using a box filter approach
     * @param corners Vector of detected corners with their responses
     * @param image_width Width of the source image
     * @param image_height Height of the source image
     * @param box_size Size of the box filter window
     * @param threshold Minimum response threshold
     * @return Vector of filtered corners
     */
    static std::vector<Corner> filterCorners(
        const std::vector<Corner>& corners,
        int image_width,
        int image_height,
        int box_size,
        float threshold
    ) {
        if (corners.empty()) return std::vector<Corner>();

        // Sort corners by response in descending order
        // std::vector<Corner> sorted_corners = corners;
        // std::sort(sorted_corners.begin(), sorted_corners.end(),
        //     [](const Corner& a, const Corner& b) {
        //         return a.response > b.response;
        //     });

        // Create a mask image to mark selected corners
        cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);
        std::vector<Corner> filtered_corners;
        
        int half_box = box_size / 2;

        // Process corners from strongest to weakest
        for (const Corner& corner : corners) {
            // Skip if response is below threshold
            // if (corner.response < threshold) continue;

            // Check if there's already a stronger corner in the box neighborhood
            bool is_local_maximum = true;
            
            // Define box boundaries
            int start_x = std::max(0, corner.x - half_box);
            int end_x = std::min(image_width - 1, corner.x + half_box);
            int start_y = std::max(0, corner.y - half_box);
            int end_y = std::min(image_height - 1, corner.y + half_box);

            // Check if any pixel in the box is already marked
            for (int y = start_y; y <= end_y; y++) {
                for (int x = start_x; x <= end_x; x++) {
                    if (mask.at<uchar>(y, x) > 0) {
                        is_local_maximum = false;
                        break;
                    }
                }
                if (!is_local_maximum) break;
            }

            // If this corner is a local maximum, add it to filtered corners
            if (is_local_maximum) {
                Corner corner_;
                corner_.x = corner.x;
                corner_.y = corner.y;
                corner_.label = static_cast<int>(filtered_corners.size());
                // corner.label = static_cast<int>(filtered_corners.size());
                filtered_corners.push_back(corner_);
                // Mark the box region in the mask
                cv::rectangle(mask, 
                    cv::Point(start_x, start_y),
                    cv::Point(end_x, end_y),
                    cv::Scalar(255), -1);
            }
        }

        return filtered_corners;
    }
};

#include <vector>
#include <unordered_map>
#include <deque>
#include <cmath>
#include <limits>

struct TrackedCorner {
    int x;
    int y;
    // float response;
    int label;
    int frame_count;
    bool is_matched;
    int frames_since_last_detection;  // Number of frames since last seen
    std::deque<cv::Point> position_history;
    cv::Point2f velocity;  // Estimated velocity for prediction
};

class CornerTracker {
private:
    int next_label;
    float max_distance;
    int max_frames_to_keep;
    int history_length;
    int max_frames_to_skip;  // Maximum number of frames a corner can be missing
    std::vector<TrackedCorner> active_tracks;

    float calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void updatePositionHistory(TrackedCorner& corner) {
        corner.position_history.push_front(cv::Point(corner.x, corner.y));
        if (corner.position_history.size() > history_length) {
            corner.position_history.pop_back();
        }
    }

    cv::Point2f estimateVelocity(const TrackedCorner& corner) {
        if (corner.position_history.size() < 2) {
            return cv::Point2f(0, 0);
        }

        // Calculate average velocity from last few positions
        cv::Point2f total_velocity(0, 0);
        int count = 0;
        for (size_t i = 1; i < corner.position_history.size(); ++i) {
            cv::Point2f vel(
                corner.position_history[i-1].x - corner.position_history[i].x,
                corner.position_history[i-1].y - corner.position_history[i].y
            );
            total_velocity += vel;
            count++;
        }
        return count > 0 ? total_velocity * (1.0f / count) : cv::Point2f(0, 0);
    }

    cv::Point2f predictPosition(const TrackedCorner& corner) {
        return cv::Point2f(corner.x, corner.y) + corner.velocity;
    }

public:
    CornerTracker(float max_matching_distance = 30.0f, 
                  int max_frames = 30,
                  int history_size = 10,
                  int frames_to_skip = 10) 
        : next_label(0), 
          max_distance(max_matching_distance), 
          max_frames_to_keep(max_frames),
          history_length(history_size),
          max_frames_to_skip(frames_to_skip) {}

    std::vector<TrackedCorner> updateTrackedCorners(const std::vector<Corner>& current_corners) {
        // Convert current corners to tracked corners
        std::vector<TrackedCorner> current_detected;
        for (const auto& corner : current_corners) {
            TrackedCorner tracked;
            tracked.x = corner.x;
            tracked.y = corner.y;
            // tracked.response = corner.response;
            tracked.label = -1;
            tracked.frame_count = 0;
            tracked.is_matched = false;
            tracked.frames_since_last_detection = 0;
            current_detected.push_back(tracked);
        }

        // Reset matching flags
        for (auto& track : active_tracks) {
            track.is_matched = false;
        }

        // Match current detections with active tracks
        std::vector<bool> detection_matched(current_detected.size(), false);
        
        // First pass: Match based on proximity to predicted positions
        for (auto& track : active_tracks) {
            if (track.frames_since_last_detection > max_frames_to_skip) continue;

            cv::Point2f predicted_pos = predictPosition(track);
            float min_distance = max_distance;
            int best_match_idx = -1;

            for (size_t i = 0; i < current_detected.size(); ++i) {
                if (detection_matched[i]) continue;

                float dist = calculateDistance(
                    predicted_pos, 
                    cv::Point2f(current_detected[i].x, current_detected[i].y)
                );

                if (dist < min_distance) {
                    min_distance = dist;
                    best_match_idx = i;
                }
            }

            if (best_match_idx >= 0) {
                // Update track with new detection
                track.x = current_detected[best_match_idx].x;
                track.y = current_detected[best_match_idx].y;
                // track.response = current_detected[best_match_idx].response;
                track.is_matched = true;
                track.frames_since_last_detection = 0;
                track.frame_count++;
                updatePositionHistory(track);
                track.velocity = estimateVelocity(track);
                
                detection_matched[best_match_idx] = true;
            } else {
                // Update track with prediction
                cv::Point2f pred = predictPosition(track);
                track.x = pred.x;
                track.y = pred.y;
                track.frames_since_last_detection++;
                updatePositionHistory(track);
            }
        }

        // Create new tracks for unmatched detections
        for (size_t i = 0; i < current_detected.size(); ++i) {
            if (!detection_matched[i]) {
                TrackedCorner new_track = current_detected[i];
                new_track.label = next_label++;
                new_track.frame_count = 1;
                new_track.frames_since_last_detection = 0;
                new_track.velocity = cv::Point2f(0, 0);
                updatePositionHistory(new_track);
                active_tracks.push_back(new_track);
            }
        }

        // Remove old tracks
        active_tracks.erase(
            std::remove_if(
                active_tracks.begin(), 
                active_tracks.end(),
                [this](const TrackedCorner& c) {
                    return c.frames_since_last_detection > max_frames_to_skip || 
                           c.frame_count > max_frames_to_keep;
                }
            ),
            active_tracks.end()
        );

        return active_tracks;
    }

    void reset() {
        next_label = 0;
        active_tracks.clear();
    }
};

class CornerVisualizer {
public:
    static cv::Mat visualizeTrackedCorners(
        const cv::Mat& image,
        const std::vector<TrackedCorner>& corners
    ) {
        cv::Mat output = image.clone();
        
        for (const auto& corner : corners) {
            // Only visualize corners that have been tracked for a minimum number of frames
            if (corner.frame_count >= 3) {
                // Generate color based on label
                int hue = corner.label * 30 % 180;
                cv::Scalar color = cv::Scalar(hue, 255, 255);
                
                // Adjust transparency based on frames since last detection
                float alpha = std::max(0.3f, 
                    1.0f - (corner.frames_since_last_detection / 5.0f));
                cv::Scalar fade_color = color * alpha;
                
                // Draw corner point (smaller if predicted)
                int radius = corner.frames_since_last_detection == 0 ? 3 : 2;
                cv::circle(output, cv::Point(corner.x, corner.y), radius, fade_color, -1);
                
                // Draw label and status
                std::string label_text = std::to_string(corner.label);
                if (corner.frames_since_last_detection > 0) {
                    label_text += "(" + std::to_string(corner.frames_since_last_detection) + ")";
                }
                cv::putText(output, label_text,
                    cv::Point(corner.x + 5, corner.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, fade_color, 1);
                
                // Draw path
                if (corner.position_history.size() > 1) {
                    // Draw lines connecting historical positions
                    for (size_t i = 1; i < corner.position_history.size(); ++i) {
                        cv::line(output,
                            corner.position_history[i-1],
                            corner.position_history[i],
                            fade_color, 1, cv::LINE_AA);
                    }
                    
                    // Draw arrow for predicted movement
                    if (corner.frames_since_last_detection > 0) {
                        cv::Point prediction(
                            corner.x + corner.velocity.x,
                            corner.y + corner.velocity.y
                        );
                        cv::arrowedLine(output,
                            cv::Point(corner.x, corner.y),
                            prediction,
                            fade_color, 1, cv::LINE_AA);
                    }
                }
            }
        }
        
        return output;
    }
};


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
    Metavision::timestamp period_us = 10000, min_generation_period_us = 10000;

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

    cv::Mat corner_img(camera_height, camera_width, CV_8UC3);
    corner_img.setTo(cv::Scalar::all(0));


    cv::Mat filtered_corner_img(camera_height, camera_width, CV_8UC3);
    filtered_corner_img.setTo(cv::Scalar::all(0));

    Metavision::MostRecentTimestampBuffer time_surface(camera_height, camera_width, 1);
    time_surface.set_to(0);

    // we use a mutex to control concurrent accesses to the time surface
    std::mutex frame_mutex;

    // create a variable where to store the latest timestamp
    Metavision::timestamp last_time = 0;

    int delta_ts = 10000;

    int time_surface_flag = 0;

    std::vector<Corner> corners ;

    // Initialize tracker
    CornerTracker tracker(30.0f, 30, 10, 5);  // 30.0f is max matching distance

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
                                               std::cout << "Corner size : " << corners.size() << std::endl;


                                               std::vector<Corner> filtered = CornerFilter::filterCorners(
                                                                                                                corners,
                                                                                                                camera_width,
                                                                                                                camera_height,
                                                                                                                15,  // box size
                                                                                                                0.5f // threshold
                                                                                                            );

                                                std::cout << "Filtered corner size : " << filtered.size() << std::endl;

                                                //   for (const Corner& corner : filtered) {
                                                //     cv::circle(filtered_corner_img, cv::Point(corner.x, corner.y), 3, cv::Scalar(0, 0, 255), -1);
                                                //     cv::putText(filtered_corner_img, std::to_string(corner.label), cv::Point(corner.x, corner.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                                                //   }

                                                std::vector<TrackedCorner> tracked_corners = tracker.updateTrackedCorners(filtered);
                                                cv::Mat tracked_corners_img = CornerVisualizer::visualizeTrackedCorners(filtered_corner_img, tracked_corners);


                                                

                                               std::unique_lock<std::mutex> lock(frame_mutex);
                                               // generate the time surface from MostRecentTimestampBuffer
                                               time_surface.generate_img_time_surface(last_time, delta_ts, time_surface_gray);
                                               // apply a colormap to the time surface and display the new frame
                                               // cv::applyColorMap(time_surface_gray, heatmap, cv::COLORMAP_JET);
                                               // window.show(heatmap);
                                               std::cout << time_surface_gray.size() << std::endl;
                                                cv::imshow("Time Surface", time_surface_gray); 
                                                cv::imshow("Corner Image", corner_img);
                                                cv::imshow("Filtered Corner Image", filtered_corner_img);
                                                cv::imshow("Tracked Corner Image", tracked_corners_img);

                                                sprintf(file_name, "cluster_frame_combined%d.jpg", frame_count++);
                                                cv::imwrite(file_name, tracked_corners_img);
                                                cv::waitKey(20);
                                            //    window.show(time_surface_gray);
                                                time_surface_flag = 1;
                                                corner_img.setTo(cv::Scalar::all(0));
                                                filtered_corner_img.setTo(cv::Scalar::all(0));
                                                corners.clear();
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

        if (time_surface_flag == 1)
        {
            // time_surface_flag = 0;
            // cv::Mat corner_img(camera_height, camera_width, CV_8UC3);
            // corner_img.setTo(cv::Scalar::all(0));

            for (auto it = begin; it != end; ++it) {
                std::unique_lock<std::mutex> lock(frame_mutex);
                // time_surface.at(it->y, it->x) = it->t;
                // last_time                     = it->t;
                // std::cout << it->t << std::endl;

                // if (isFeature(it->x, it->y, &time_surface))
                // {
                //   std::cout << "Feature detected" << std::endl;
                // }

                // update SAE
                //   const int pol = e.polarity ? 1 : 0;
                //   time_surface.at(x, y) = e.ts.toSec();
                
                

                const int max_scale = 1;

                // only check if not too close to border
                const int cs = max_scale*4;
                if (it->x < cs || it->x >= 1280-cs ||
                    it->y < cs || it->y >= 720-cs)
                {
                    // std::cout << "Event received: coordinates (" << it->x << ", " << it->y << "), t: " << it->t
                    //       << ", polarity: " << it->p << std::endl;
                    break;
                }

                bool found_streak = false;

                for (int i=0; i<16; i++)
                {
                    for (int streak_size = 3; streak_size<=6; streak_size++)
                    {
                    // check that streak event is larger than neighbor
                    // std::cout << "init" << it->x+circle3_[i][0] << std::endl;
                    if (time_surface.at(it->y+circle3_[i][0], it->x+circle3_[i][1]) <  time_surface.at(it->y+circle3_[(i-1+16)%16][0], it->x+circle3_[(i-1+16)%16][1]))
                        continue;

                    // check that streak event is larger than neighbor
                    if (time_surface.at(it->y+circle3_[(i+streak_size-1)%16][0], it->x+circle3_[(i+streak_size-1)%16][1]) <          time_surface.at(it->y+circle3_[(i+streak_size)%16][0], it->x+circle3_[(i+streak_size)%16][1]))
                        continue;

                    // std::cout << "after compare" << it->y+circle3_[i][0] << std::endl;

                    double min_t = time_surface.at(it->y+circle3_[i][0], it->x+circle3_[i][1]);
                    for (int j=1; j<streak_size; j++)
                    {
                        const double tj = time_surface.at(it->y+circle3_[(i+j)%16][0], it->x+circle3_[(i+j)%16][1]);
                        if (tj < min_t)
                        min_t = tj;
                    }

                    bool did_break = false;
                    for (int j=streak_size; j<16; j++)
                    {
                        const double tj = time_surface.at(it->y+circle3_[(i+j)%16][0], it->x+circle3_[(i+j)%16][1]);

                        if (tj >= min_t)
                        {
                        did_break = true;
                        break;
                        }
                    }

                    if (!did_break)
                    {
                        found_streak = true;
                        break;
                    }

                    }
                    if (found_streak)
                    {
                    break;
                    }
                }

                if (found_streak)
                {
                    found_streak = false;
                    for (int i=0; i<20; i++)
                    {
                    for (int streak_size = 4; streak_size<=8; streak_size++)
                    {
                        // check that first event is larger than neighbor
                        if (time_surface.at(it->y+circle4_[i][0], it->x+circle4_[i][1]) <  time_surface.at(it->y+circle4_[(i-1+20)%20][0], it->x+circle4_[(i-1+20)%20][1]))
                        continue;

                        // check that streak event is larger than neighbor
                        if (time_surface.at(it->y+circle4_[(i+streak_size-1)%20][0], it->x+circle4_[(i+streak_size-1)%20][1]) <          time_surface.at(it->y+circle4_[(i+streak_size)%20][0], it->x+circle4_[(i+streak_size)%20][1]))
                        continue;

                        double min_t = time_surface.at(it->y+circle4_[i][0], it->x+circle4_[i][1]);
                        for (int j=1; j<streak_size; j++)
                        {
                        const double tj = time_surface.at(it->y+circle4_[(i+j)%20][0], it->x+circle4_[(i+j)%20][1]);
                        if (tj < min_t)
                            min_t = tj;
                        }

                        bool did_break = false;
                        for (int j=streak_size; j<20; j++)
                        {
                        const double tj = time_surface.at(it->y+circle4_[(i+j)%20][0], it->x+circle4_[(i+j)%20][1]);
                        if (tj >= min_t)
                        {
                            did_break = true;
                            break;
                        }
                        }

                        if (!did_break)
                        {
                        found_streak = true;
                        break;
                        }
                    }
                    if (found_streak)
                    {
                        break;
                    }
                    }
                }

                // return found_streak;
                if (found_streak)
                {
                    // std::cout << "Feature detected" << std::endl;
                    cv::circle(corner_img, cv::Point(it->x, it->y), 3, cv::Scalar(0, 0, 255), -1);
                    corners.push_back(Corner{it->x, it->y});
                    // window.show(corner_img);

                }

            
             }

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
