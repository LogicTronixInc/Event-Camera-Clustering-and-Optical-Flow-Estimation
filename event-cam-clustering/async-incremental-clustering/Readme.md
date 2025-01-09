# Event Camera Incremental Clustering

Requirements :
- OpenEB SDK available at https://github.com/prophesee-ai/openeb

## Compiling the project
- In source code directory, create a build folder if not present.
- Change directory to build directory, run `cd build` in terminal to move to build directory
- Run cmake .. to generate make files
- Run make to build the project
```
mkdir build
cd build
cmake ..
make
```

## Running the application 
- Application can take input from raw event data file or with available event camera
- Run application with raw event data file from build directory
```
./metavision_sdk_get_started2_events_clustering traffic_data.raw
```
