# humanArmkinematics
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

## Stored Data

In the folder `test/tst_data` some csv data files shall be found with different data to be use in different ways.

| Data                                              | Description |
| :--                                               | ----------- |
| data1_randomImuData.csv                           | Some random IMU data |
| data2_zRotations.csv                              | IMUs quaternions rotations over z axis | 
| data3_yRotations.csv                              | IMUs quaternions rotations over y axis | 
| data4_xRotations.csv                              | IMUs quaternions rotations over x axis | 
| data5_tst_cal_004_onArmArbitraryMotions.csv       | Measures of the IMUs while mounted on the arm and performing arbitrary motions | 
| data6_tst_cal_004_onArmArbitraryMotions.csv       | Measures of the IMUs while mounted on the arm and performing arbitrary motions | 
| data7_tst_cal_004_onArmArbitraryMotions.csv       | Measures of the IMUs while mounted on the arm and performing arbitrary motions (Longer period) | 