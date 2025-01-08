#include "../include/optics/optics.hpp"
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
/**
 * Reads in the data.csv file into a vector of points
 * @return vector of points
 *
 */
using namespace std;
struct Point{
    double x, y;
    Point() : x(0.0), y(0.0) {}
    Point(double x, double y) : x(x), y(y) {}
};
typedef std::array<int, 2> point; //A list of N cartesian coordinates makes a point

vector<point> readcsv(int start, int last) {
    vector<point> points;
    char file_name[100];
    string line;
    int frame_count = 0;
    for (int i=start; i<=last; i++){
        // sprintf(file_name, "event_raw_data_5_11_2024_side_90_no_noise_3/event_raw_data%d.csv", start+frame_count);
        sprintf(file_name, "event_cam_data_capture_camera_bias_25/event_raw_data%d.csv", start+frame_count);
        
        ifstream file(file_name);

        cout << "read file : " << i << endl;

        while (getline(file, line)) {
            stringstream lineStream(line);
            string bit;
            int x, y;
            // cout << "read line" << endl;
            getline(lineStream, bit, ',');
            
            // cout << bit << endl;
            x = stoi(bit);
            getline(lineStream, bit, ',');
            y = stoi(bit);
            std::array<int,2> event_data = {x, y};
            

            points.push_back(event_data);
        }
        frame_count++;
    }

    return points;

}    


vector<point> readcsv() {
    vector<point> points;
    string line;
    ifstream file("dikesh_event_data/S1_session1_mov2_frame17_cam3.csv");

    cout << "read file" << endl;

    while (getline(file, line)) {
        stringstream lineStream(line);
        string bit;
        int x, y;
        // cout << "read line" << endl;
        getline(lineStream, bit, ',');
        
        // cout << bit << endl;
        x = stoi(bit);
        getline(lineStream, bit, ',');
        y = stoi(bit);
        std::array<int,2> event_data = {x, y};
        

        points.push_back(event_data);
    }

    // ifstream file2("camera/event_raw_data22.csv");

    // cout << "read file" << endl;

    // while (getline(file2, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file3("camera/event_raw_data23.csv");

    // cout << "read file3" << endl;

    // while (getline(file3, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file4("camera/event_raw_data24.csv");

    // cout << "read file4" << endl;

    // while (getline(file4, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file5("camera/event_raw_data25.csv");

    // cout << "read file3" << endl;

    // while (getline(file5, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file6("camera/event_raw_data26.csv");

    // cout << "read file6" << endl;

    // while (getline(file6, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file7("camera/event_raw_data27.csv");

    // cout << "read file" << endl;

    // while (getline(file7, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file8("camera/event_raw_data28.csv");

    // cout << "read file" << endl;

    // while (getline(file8, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file9("camera/event_raw_data29.csv");

    // cout << "read file9" << endl;

    // while (getline(file9, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file10("camera/event_raw_data30.csv");

    // cout << "read file4" << endl;

    // while (getline(file10, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file11("camera/event_raw_data31.csv");

    // cout << "read file3" << endl;

    // while (getline(file11, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    // ifstream file12("camera/event_raw_data32.csv");

    // cout << "read file6" << endl;

    // while (getline(file12, line)) {
    //     stringstream lineStream(line);
    //     string bit;
    //     int x, y;
    //     // cout << "read line" << endl;
    //     getline(lineStream, bit, ',');
        
    //     // cout << bit << endl;
    //     x = stoi(bit);
    //     getline(lineStream, bit, ',');
    //     y = stoi(bit);
    //     std::array<int,2> event_data = {x, y};
        

    //     points.push_back(event_data);
    // }

    return points;
}

void clustering_test_1(){
	static const int N = 2;
	// typedef std::array<double, N> point; //A list of N cartesian coordinates makes a point
    int start = 3080;
    int end = 3090 ;

    char reahability_file_name[100]; 
    sprintf(reahability_file_name, "ReachabilityPlot_event_%d_%d", start, end);

    char cluster_file_name[100]; 
    sprintf(cluster_file_name, "Clusters2d_events_%d_%d", start, end);

	// vector<point> points = readcsv(start, end);
    vector<point> points = readcsv();
    const int size = points.size();
    cout << size << endl;
    const int size_c = 9548;
	auto reach_dists = optics::compute_reachability_dists<size_c>( points, 2, 10 );
	/*for( const auto& x : reach_dists){
		std::cout << x.to_string() << "; ";
	}*/

	auto clusters = optics::get_cluster_indices(reach_dists, 10);
    cout << "Cluster size : " << clusters.size() << endl;
    for (int j=0; j< clusters.size(); j++){
        //  cout << "cluster " << j << " size : " << clusters[j].size() << endl;

        // for (int i = 0; i < clusters[j].size() ; i++){
        //     cout << "cluster " << j << " components : " << clusters[j][i] << endl;

        // }

    }
   
    

    auto img = optics::draw_reachability_plot( reach_dists );
	img.save( reahability_file_name);
    auto cluster_points = optics::get_cluster_points(reach_dists, 10, points);

    vector<double> sumX, sumY ;
    vector<vector<double>> centroids ;

    vector<double> deviationsX, deviationsY;
    vector<vector <double>> deviations ; 

    for (int i=0; i < cluster_points.size(); i++){
       sumX.push_back(0.0);
       sumY.push_back(0.0);
       centroids.push_back({0.0, 0.0});

       deviationsX.push_back(0.0);
       deviationsY.push_back(0.0);

       deviations.push_back({0.0, 0.0});


    }


    cout << "cluster points size : " << cluster_points.size() << endl ;
    for (int j = 0; j < cluster_points.size() ; j++){
        // cout << " cl " << j << " " << cluster_points[j].size() << endl;
        for (int i=0; i < cluster_points[j].size(); i++){
            // cout << cluster_points[j][i][0] << "," << cluster_points[j][i][1]  << endl;
            sumX[j] += cluster_points[j][i][0];
            sumY[j] += cluster_points[j][i][1];
            
        }
        // cout << "Centroid X : " << sumX[j] / cluster_points[j].size() << endl;
        // cout << "Centroid Y : " << sumY[j] / cluster_points[j].size() << endl;
        centroids[j][0]=(sumX[j] / cluster_points[j].size());
        centroids[j][1]=(sumY[j] / cluster_points[j].size());


    }

    for (int j = 0; j < cluster_points.size() ; j++){
        // cout << " cl " << j << " " << cluster_points[j].size() << endl;
        for (int i=0; i < cluster_points[j].size(); i++){
            // cout << cluster_points[j][i][0] << "," << cluster_points[j][i][1]  << endl;
            // sumX[j] += (cluster_points[j][i][0] - );
            // sumY[j] += cluster_points[j][i][1];
            double deviation_diff_x = cluster_points[j][i][0] - centroids[j][0];
            deviationsX[j] += deviation_diff_x * deviation_diff_x;

            double deviation_diff_y = cluster_points[j][i][1] - centroids[j][1];
            deviationsY[j] += deviation_diff_y * deviation_diff_y;
            
        }
        // cout << "Centroid size : " << centroids[j].size() << endl;
        // cout << "Deviation X  : " << deviationsX[j]/cluster_points[j].size()<< endl;
        // cout << "Deviation Y  : " << deviationsY[j]/cluster_points[j].size()<< endl;
        deviations[j][0] = deviationsX[j]/cluster_points[j].size();
        deviations[j][1] = deviationsY[j]/cluster_points[j].size();
        cout << j << "," <<  cluster_points[j].size() << "," << centroids[j][0] << "," << centroids[j][1] << "," << deviations[j][0] << "," << deviations[j][1] << endl;
        

    }


    //calculate disperse or deviations
    

    
    
   
	auto img_events_cluster = optics::draw_2d_clusters(cluster_points);

	img_events_cluster.save(cluster_file_name);
	
}

int clustering_test_1(int start, int end){
	static const int N = 2;
	// typedef std::array<double, N> point; //A list of N cartesian coordinates makes a point
    // int start = 1960;
    // int end = 1970 ;

    char reahability_file_name[100]; 
    sprintf(reahability_file_name, "ReachabilityPlot_event_%d_%d", start, end);

    char cluster_file_name[100]; 
    sprintf(cluster_file_name, "Clusters2d_events_%d_%d", start, end);

	vector<point> points = readcsv(start, end);
    const int size = points.size();
    cout << size << endl;
    const int size_c =  3520;
    if (size != size_c){
          std::cout << "size not equal" << std::endl;
          return 0;     
    }
	auto reach_dists = optics::compute_reachability_dists<size_c>( points, 2, 10 );
	/*for( const auto& x : reach_dists){
		std::cout << x.to_string() << "; ";
	}*/

	auto clusters = optics::get_cluster_indices(reach_dists, 10);
    cout << "Cluster size : " << clusters.size() << endl;
    for (int j=0; j< clusters.size(); j++){
        //  cout << "cluster " << j << " size : " << clusters[j].size() << endl;

        // for (int i = 0; i < clusters[j].size() ; i++){
        //     cout << "cluster " << j << " components : " << clusters[j][i] << endl;

        // }

    }
   
    

    // auto img = optics::draw_reachability_plot( reach_dists );
	// img.save( reahability_file_name);
    auto cluster_points = optics::get_cluster_points(reach_dists, 10, points);

    vector<double> sumX, sumY ;
    vector<vector<double>> centroids ;

    vector<double> deviationsX, deviationsY;
    vector<vector <double>> deviations ; 

    for (int i=0; i < cluster_points.size(); i++){
       sumX.push_back(0.0);
       sumY.push_back(0.0);
       centroids.push_back({0.0, 0.0});

       deviationsX.push_back(0.0);
       deviationsY.push_back(0.0);

       deviations.push_back({0.0, 0.0});


    }


    cout << "cluster points size : " << cluster_points.size() << endl ;
    for (int j = 0; j < cluster_points.size() ; j++){
        // cout << " cl " << j << " " << cluster_points[j].size() << endl;
        for (int i=0; i < cluster_points[j].size(); i++){
            // cout << cluster_points[j][i][0] << "," << cluster_points[j][i][1]  << endl;
            sumX[j] += cluster_points[j][i][0];
            sumY[j] += cluster_points[j][i][1];
            
        }
        // cout << "Centroid X : " << sumX[j] / cluster_points[j].size() << endl;
        // cout << "Centroid Y : " << sumY[j] / cluster_points[j].size() << endl;
        centroids[j][0]=(sumX[j] / cluster_points[j].size());
        centroids[j][1]=(sumY[j] / cluster_points[j].size());


    }

    for (int j = 0; j < cluster_points.size() ; j++){
        // cout << " cl " << j << " " << cluster_points[j].size() << endl;
        for (int i=0; i < cluster_points[j].size(); i++){
            // cout << cluster_points[j][i][0] << "," << cluster_points[j][i][1]  << endl;
            // sumX[j] += (cluster_points[j][i][0] - );
            // sumY[j] += cluster_points[j][i][1];
            double deviation_diff_x = cluster_points[j][i][0] - centroids[j][0];
            deviationsX[j] += deviation_diff_x * deviation_diff_x;

            double deviation_diff_y = cluster_points[j][i][1] - centroids[j][1];
            deviationsY[j] += deviation_diff_y * deviation_diff_y;
            
        }
        // cout << "Centroid size : " << centroids[j].size() << endl;
        // cout << "Deviation X  : " << deviationsX[j]/cluster_points[j].size()<< endl;
        // cout << "Deviation Y  : " << deviationsY[j]/cluster_points[j].size()<< endl;
        deviations[j][0] = deviationsX[j]/cluster_points[j].size();
        deviations[j][1] = deviationsY[j]/cluster_points[j].size();
        cout << j << "," <<  cluster_points[j].size() << "," << centroids[j][0] << "," << centroids[j][1] << "," << deviations[j][0] << "," << deviations[j][1] << endl;
        

    }


    //calculate disperse or deviations
    

    
    
   
	auto img_events_cluster = optics::draw_2d_clusters(cluster_points);

	img_events_cluster.save(cluster_file_name);

    return 0;
	
}

int main()
{
    // clustering_test_1();
    int start = 10;
    for (int i= 10; i<=130 ; i++){
        
        int end = start + 10;
        clustering_test_1(start, end);
        start = end;

        
    }
    return 0;
}
