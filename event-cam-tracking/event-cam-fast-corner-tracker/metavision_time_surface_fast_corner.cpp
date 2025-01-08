/**********************************************************************************************************************
 * Copyright (c) Prophesee S.A.                                                                                       *
 *                                                                                                                    *
 * Licensed under the Apache License, Version 2.0 (the "License");                                                    *
 * you may not use this file except in compliance with the License.                                                   *
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0                                 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed   *
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.                      *
 * See the License for the specific language governing permissions and limitations under the License.                 *
 **********************************************************************************************************************/

// Example of using Metavision SDK Core API for visualizing Time Surface of events

#include <boost/program_options.hpp>
#include <mutex>
#include <metavision/sdk/core/utils/mostrecent_timestamp_buffer.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>

namespace po = boost::program_options;

 int circle3_[16][2] = {{0, 3}, {1, 3}, {2, 2}, {3, 1},
            {3, 0}, {3, -1}, {2, -2}, {1, -3},
            {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
            {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};
int circle4_[20][2] = {{0, 4}, {1, 4}, {2, 3}, {3, 2},
            {4, 1}, {4, 0}, {4, -1}, {3, -2},
            {2, -3}, {1, -4}, {0, -4}, {-1, -4},
            {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
            {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}};


bool isFeature(int x, int y,  Metavision::MostRecentTimestampBuffer* time_surface)
{
  // update SAE
//   const int pol = e.polarity ? 1 : 0;
//   time_surface.at(x, y) = e.ts.toSec();
  

  const int max_scale = 1;

  // only check if not too close to border
  const int cs = max_scale*4;
  if (x < cs || x >= 1280-cs ||
      y < cs || y >= 720-cs)
  {
    return false;
  }

  bool found_streak = false;

  for (int i=0; i<16; i++)
  {
    for (int streak_size = 3; streak_size<=6; streak_size++)
    {
      // check that streak event is larger than neighbor
      if (time_surface->at(x+circle3_[i][0], y+circle3_[i][1]) <  time_surface->at(x+circle3_[(i-1+16)%16][0], y+circle3_[(i-1+16)%16][1]))
        continue;

      // check that streak event is larger than neighbor
      if (time_surface->at(x+circle3_[(i+streak_size-1)%16][0], y+circle3_[(i+streak_size-1)%16][1]) <          time_surface->at(x+circle3_[(i+streak_size)%16][0], y+circle3_[(i+streak_size)%16][1]))
        continue;

      double min_t = time_surface->at(x+circle3_[i][0], y+circle3_[i][1]);
      for (int j=1; j<streak_size; j++)
      {
        const double tj = time_surface->at(x+circle3_[(i+j)%16][0], y+circle3_[(i+j)%16][1]);
        if (tj < min_t)
          min_t = tj;
      }

      bool did_break = false;
      for (int j=streak_size; j<16; j++)
      {
        const double tj = time_surface->at(x+circle3_[(i+j)%16][0], y+circle3_[(i+j)%16][1]);

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
        if (time_surface->at(x+circle4_[i][0], y+circle4_[i][1]) <  time_surface->at(x+circle4_[(i-1+20)%20][0], y+circle4_[(i-1+20)%20][1]))
          continue;

        // check that streak event is larger than neighbor
        if (time_surface->at(x+circle4_[(i+streak_size-1)%20][0], y+circle4_[(i+streak_size-1)%20][1]) <          time_surface->at(x+circle4_[(i+streak_size)%20][0], y+circle4_[(i+streak_size)%20][1]))
          continue;

        double min_t = time_surface->at(x+circle4_[i][0], y+circle4_[i][1]);
        for (int j=1; j<streak_size; j++)
        {
          const double tj = time_surface->at(x+circle4_[(i+j)%20][0], y+circle4_[(i+j)%20][1]);
          if (tj < min_t)
            min_t = tj;
        }

        bool did_break = false;
        for (int j=streak_size; j<20; j++)
        {
          const double tj = time_surface->at(x+circle4_[(i+j)%20][0], y+circle4_[(i+j)%20][1]);
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

  return found_streak;
}

int main(int argc, char *argv[]) {
    std::string serial;
    std::string cam_config_path;
    std::string event_file_path;
    uint32_t delta_ts;

    const std::string short_program_desc(
        "Example of using Metavision SDK Core API for visualizing Time Surface of events.\n");
    po::options_description options_desc("Options");

    // clang-format off
    options_desc.add_options()
        ("help,h", "Produce help message.")
        ("serial,s",              po::value<std::string>(&serial),"Serial ID of the camera. This flag is incompatible with flag '--input-event-file'.")
        ("input-event-file,i",    po::value<std::string>(&event_file_path), "Path to input event file (RAW or HDF5). If not specified, the camera live stream is used.")
        ("input-camera-config,j", po::value<std::string>(&cam_config_path), "Path to a JSON file containing camera config settings to restore a camera state. Only works for live cameras.")
        ("accumulation-time,a",   po::value<uint32_t>(&delta_ts)->default_value(10000), "Accumulation time for which to display the Time Surface.")
    ;
    // clang-format on

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(options_desc).run(), vm);
        po::notify(vm);
    } catch (po::error &e) {
        MV_LOG_ERROR() << short_program_desc;
        MV_LOG_ERROR() << options_desc;
        MV_LOG_ERROR() << "Parsing error:" << e.what();
        return 1;
    }

    if (vm.count("help")) {
        MV_LOG_INFO() << short_program_desc;
        MV_LOG_INFO() << options_desc;
        return 0;
    }

    Metavision::Camera camera;

    // if the filename is set, then read from the file
    if (!event_file_path.empty()) {
        if (!serial.empty()) {
            MV_LOG_ERROR() << "Options --serial and --input-event-file are not compatible.";
            return 1;
        }

        try {
            camera =
                Metavision::Camera::from_file(event_file_path, Metavision::FileConfigHints().real_time_playback(true));

        } catch (Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            return 2;
        }
    // otherwise, set the input source to a camera
    } else {
        try {
            if (!serial.empty()) {
                camera = Metavision::Camera::from_serial(serial);
            } else {
                camera = Metavision::Camera::from_first_available();
            }

            if (!cam_config_path.empty()) {
                camera.load(cam_config_path);
            }
        } catch (Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            return 3;
        }
    }

    // get camera resolution
    int camera_width  = camera.geometry().width();
    int camera_height = camera.geometry().height();

    // create a MostRecentTimestampBuffer to store the last timestamp of each pixel and initialize all elements to zero
    Metavision::MostRecentTimestampBuffer time_surface(camera_height, camera_width, 1);
    time_surface.set_to(0);

    // we use a mutex to control concurrent accesses to the time surface
    std::mutex frame_mutex;
    // create a variable where to store the latest timestamp
    Metavision::timestamp last_time = 0;

    // create cv::Mat to store the time surface and the heatmap
    cv::Mat heatmap, time_surface_gray;
    // update the time surface using a callback on CD events

    // to render the frames, we create a window using the Window class of the UI module
    Metavision::Window window("Metavision Time Surface", camera_width, camera_height,
                              Metavision::BaseWindow::RenderMode::BGR);

    // we set a callback on the windows to close it when the Escape or Q key is pressed
    window.set_keyboard_callback(
        [&window](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods) {
            if (action == Metavision::UIAction::RELEASE &&
                (key == Metavision::UIKeyEvent::KEY_ESCAPE || key == Metavision::UIKeyEvent::KEY_Q)) {
                window.set_close_flag();
            }
        });

    cv::Mat corner_img(camera_height, camera_width, CV_8UC3);

    camera.cd().add_callback([&time_surface, &frame_mutex, &last_time, &camera_height, &camera_width, &corner_img](const Metavision::EventCD *ev_begin,
                                                                       const Metavision::EventCD *ev_end) {


        // cv::Mat corner_img(camera_height, camera_width, CV_8UC3);
        corner_img.setTo(cv::Scalar::all(0));

        for (auto it = ev_begin; it != ev_end; ++it) {
            std::unique_lock<std::mutex> lock(frame_mutex);
            time_surface.at(it->y, it->x) = it->t;
            last_time                     = it->t;
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
            if (it->x < cs || it->x >= 720-cs ||
                it->y < cs || it->y >= 1280-cs)
            {
                break;
            }

            bool found_streak = false;

            for (int i=0; i<16; i++)
            {
                for (int streak_size = 3; streak_size<=6; streak_size++)
                {
                // check that streak event is larger than neighbor
                // std::cout << "init" << it->x+circle3_[i][0] << std::endl;
                if (time_surface.at(it->x+circle3_[i][0], it->y+circle3_[i][1]) <  time_surface.at(it->x+circle3_[(i-1+16)%16][0], it->y+circle3_[(i-1+16)%16][1]))
                    continue;

                // check that streak event is larger than neighbor
                if (time_surface.at(it->x+circle3_[(i+streak_size-1)%16][0], it->y+circle3_[(i+streak_size-1)%16][1]) <          time_surface.at(it->x+circle3_[(i+streak_size)%16][0], it->y+circle3_[(i+streak_size)%16][1]))
                    continue;

                // std::cout << "after compare" << it->x+circle3_[i][0] << std::endl;

                double min_t = time_surface.at(it->x+circle3_[i][0], it->y+circle3_[i][1]);
                for (int j=1; j<streak_size; j++)
                {
                    const double tj = time_surface.at(it->x+circle3_[(i+j)%16][0], it->y+circle3_[(i+j)%16][1]);
                    if (tj < min_t)
                    min_t = tj;
                }

                bool did_break = false;
                for (int j=streak_size; j<16; j++)
                {
                    const double tj = time_surface.at(it->x+circle3_[(i+j)%16][0], it->y+circle3_[(i+j)%16][1]);

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
                    if (time_surface.at(it->x+circle4_[i][0], it->y+circle4_[i][1]) <  time_surface.at(it->x+circle4_[(i-1+20)%20][0], it->y+circle4_[(i-1+20)%20][1]))
                    continue;

                    // check that streak event is larger than neighbor
                    if (time_surface.at(it->x+circle4_[(i+streak_size-1)%20][0], it->y+circle4_[(i+streak_size-1)%20][1]) <          time_surface.at(it->x+circle4_[(i+streak_size)%20][0], it->y+circle4_[(i+streak_size)%20][1]))
                    continue;

                    double min_t = time_surface.at(it->x+circle4_[i][0], it->y+circle4_[i][1]);
                    for (int j=1; j<streak_size; j++)
                    {
                    const double tj = time_surface.at(it->x+circle4_[(i+j)%20][0], it->y+circle4_[(i+j)%20][1]);
                    if (tj < min_t)
                        min_t = tj;
                    }

                    bool did_break = false;
                    for (int j=streak_size; j<20; j++)
                    {
                    const double tj = time_surface.at(it->x+circle4_[(i+j)%20][0], it->y+circle4_[(i+j)%20][1]);
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
                // window.show(corner_img);

            }


        }



        
    });

    

    // create cv::Mat to store the time surface and the heatmap
    // cv::Mat heatmap, time_surface_gray;

    // start the camera
    camera.start();

    // keep running until the camera is off, the recording is finished or the escape key was pressed
    while (camera.is_running() && !window.should_close()) {
        if (!time_surface.empty()) {
            // std::cout << time_surface.rows() << std::endl;
            // std::cout << time_surface.cols() << std::endl;
            // std::cout << time_surface.channels() << std::endl;
            
            // std::cout << time_surface.size() << std::endl;

            // std::cout << time_surface.at(0, 0) << std::endl;

            // std::unique_lock<std::mutex> lock(frame_mutex);
            // generate the time surface from MostRecentTimestampBuffer
            // time_surface.generate_img_time_surface(last_time, delta_ts, time_surface_gray);
            // apply a colormap to the time surface and display the new frame
            // cv::applyColorMap(time_surface_gray, heatmap, cv::COLORMAP_JET);
            window.show(corner_img);
            // std::cout << time_surface_gray.size() << std::endl;
            // window.show(time_surface_gray);
        }
        // we poll events (keyboard, mouse etc.) from the system with a 20ms sleep to avoid using 100% of a CPU's core
        // and we push them into the window where the callback on the escape key will ask the windows to close
        static constexpr std::int64_t kSleepPeriodMs = 20;
        Metavision::EventLoop::poll_and_dispatch(kSleepPeriodMs);
    }

    camera.stop();
}
