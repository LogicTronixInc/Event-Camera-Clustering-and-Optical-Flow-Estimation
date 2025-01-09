
# Event Camera Tracking with Estimation

Requirements :
- OpenEB SDK available at https://github.com/prophesee-ai/openeb

## Compiling the project
- In source code directory, create a build folder if not present.
- Change directory to build directory, runnding cd build in terminal
- Run cmake .. to generate make files
- Run make to build the project
```
cd build
cmake ..
make
```

## Running the application 
- Application can take input from raw event data file or with available event camera
- For running application with raw event data file from build directory
```
./metavision_sdk_get_started5_opencl_store traffic_data.raw
```
