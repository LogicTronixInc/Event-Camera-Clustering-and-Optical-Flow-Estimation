# about
Fast Implementation of DBSCAN using Kdtree for acceleration. The use case is clustering point cloud(PCL library used).


# Compiling
```
cd build && cmake .. && make -j 
```
## Usage

## Converting event data to .pcd file
- Run `./metavision_xyz_capture` to generate the xyz file for event data
- Next run `pcl_xyz2pcd <event_data.xyz>` to get the pcd for corresponding event frame

## Running the application
```
./pcl_cluster <event_data.pcd>
```
