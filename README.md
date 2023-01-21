# Human arm kinematics package
This repository is a working repository for my final masters degree. The aim is to take data from IMU sensors and process it to estimate the position of a human arm in order to control an exoesqueleton. 

## Dependences
**LpSensorLib**
Install LpSensorLib [OPENMAT BINARIES](https://lp-research.com/support/)
libLpSensor depends on libbluetooth.so
- `sudo apt-get update`
- `sudo apt-get install libbluetooth-dev`
- `sudo dpkg -i liblpsensor-1.3.5-Linux.deb`
- `dpkg -L liblpsensor`

**LIBSERIALPORT**
- `sudo apt-get update`
- `sudo apt-get install libserialport-dev`

## Compilation
To compile the hole project make shall be used. The available options are as follows:
- Do `make` in the root directory of the package (where the `Makefile` is located) and the application will be compiled into an executable file named `app`.
- Do `make lib` to build the library into a single file named  `libhumanak.so`.
- Do `make test` to build the library, build the tests file and execute the tests battery.
- Do `make clean` to remove all the compilation files and extra folders.

## Documentation
To generate de automatic documentation of the repository doxygen can be used. In the root folder of the package execute `doxygen Doxyfile` and a new `html` folder should be created. In here go to the `files` html file and there all the files and the different descriptions of functions can be seen. 
