#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/events/event_cd.h>


#include "AEClustering.h"
#include <opencv2/opencv.hpp>

AEClustering *eclustering(new AEClustering);

int camera_width  = 0;
int camera_height = 0;
int npixels       = 0;

char file_name[100];
int frame_count=0;

void print_timestamp_main(char * info, char * func )
{
    char print_info[20];

    auto now_b = std::chrono::system_clock::now();

    // Convert the current time to time since epoch
    auto duration_b = now_b.time_since_epoch();

    // Convert duration to milliseconds
    auto milliseconds_b
        = std::chrono::duration_cast<std::chrono::microseconds>(
            duration_b)
            .count();

    // Print the result
    std::cout << func <<" : Current time in milliseconds in "<< info  <<  "call is: "
        << milliseconds_b << std::endl;
}



// this function will be associated to the camera callback
void count_events(const Metavision::EventCD *begin, const Metavision::EventCD *end) {
    int counter = 0;
    std::deque<double> ev_data(4, 0.0);
    cv::Mat cluster_img(camera_height, camera_width, CV_8UC3);
    cluster_img.setTo(cv::Scalar::all(0));
    
    // print_timestamp_main("before event data update", "count events callback");
    // this loop allows us to get access to each event received in this callback
    for (const Metavision::EventCD *ev = begin; ev != end; ++ev) {
        ++counter; // count each event

        // print each event
        // std::cout << "Event received: coordinates (" << ev->x << ", " << ev->y << "), t: " << ev->t
        //           << ", polarity: " << ev->p << std::endl;
        // ev_data[0] = (ev->t)/1000000.0;       
        // ev_data[1] = ev->x;
        // ev_data[2] = ev->y;
        // ev_data[3] = ev->p;
        // // std::cout << "timestamp into buffer: " << ev_data[0] << std::endl;
        // eclustering->update(ev_data);
        if ((counter%256)==0){ //downsampling sample by 32
            ev_data[0] = (ev->t)/1000000.0;       
            ev_data[1] = ev->x;
            ev_data[2] = ev->y;
            ev_data[3] = ev->p;
            // std::cout << "timestamp into buffer: " << ev_data[0] << std::endl;
            eclustering->update(ev_data);
        }

    }
    // print_timestamp_main("after event data update", "count events callback");

    // int minN = eclustering->getMinN(); 
    // // std::cout << "Event cluster min N : " << minN << std::endl; 
    // // std::cout << "cluster queue size : " << eclustering->clusters.size() << std::endl; 
    // int clusterID=0;
        
    //     for (auto cc : eclustering->clusters){
            
    //         // std::cout << "Event cluster numbers : " << cc.getN() << std::endl;
            
    //         if (cc.getN() >= minN){
    //             Eigen::VectorXd cen(cc.getClusterCentroid());
                               
    //             // std::cout << "Centroid size : " << cen.size() << std::endl;
    //             // std::cout << "(" << cen[0] << " , " << cen[1] << ")" << "," ; 
    //             cv::circle(cluster_img,cv::Point(cen[0],cen[1]), 1, cv::Scalar(0,255,0,255), -1);
               
    //         }
            

    //     }
    //     // std::cout<<std::endl;

    //     sprintf(file_name, "cluster_frame_combined%d.jpg", frame_count+1);
    //     // cv::imwrite(file_name, cluster_img);
    //     cv::imshow("Image", cluster_img);
    //     cv::waitKey(1);
    //     cluster_img.release();

    // // report
    // // std::cout << "There were " << counter << " events in this callback" << std::endl;
    // frame_count++;
}

// main loop
int main(int argc, char *argv[]) {
    Metavision::Camera cam; // create the camera

    if (argc >= 2) {
        // if we passed a file path, open it
        cam = Metavision::Camera::from_file(argv[1]);
    } else {
        // open the first available camera
        cam = Metavision::Camera::from_first_available();
    }

    camera_width  = cam.geometry().width();
    camera_height = cam.geometry().height();
    npixels       = camera_width * camera_height;

    // to analyze the events, we add a callback that will be called periodically to give access to the latest events
    cam.cd().add_callback(count_events);

    // start the camera
    cam.start();

    // keep running while the camera is on or the recording is not finished
    while (cam.is_running()) {}

    // the recording is finished, stop the camera.
    // Note: we will never get here with a live camera
    cam.stop();
}
