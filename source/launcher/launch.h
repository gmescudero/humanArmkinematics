/**
 * @file launch.h
 * @author Germán Moreno Escudero
 * @brief This file is used to manage the initialization and cleaning of all pack resources
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdbool.h>
#include "errors.h"

/**
 * @brief Initialize all required resources for Human Arm Kinematics package
 * 
 * @param log (input) Initialize log file flag
 * @param csv (input) Initialize csv file flag
 * @return ERROR_CODE 
 */
ERROR_CODE hak_initialize(bool log, bool csv);
/**
 * @brief Remove all used resources for Human Arm Kinematics package
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE hak_terminate();
/**
 * @brief Set network configuration
 * 
 * @param ip (input) Ip of the target server
 * @param port (input) Port of the target server
 */
void hak_network_setup(char *ip, unsigned port);


/**
 * @brief A program to record IMU sensors data into a CSV file
 * 
 * @details
 *      The program performs the following steps:
 *      - Set up database logging to CSV
 *      - Search ans initialize IMU sensors connected to COM ports
 *      - Measure noise for the given ammount of iterations
 *      - Attack IMU data retrieve callback 
 *      - Loop while time window is not complete
 *      - End
 * 
 * @param imus_num (input) Number of IMU sensors to record data from
 * @param time (input) Total monitoring time in seconds
 * @param measureNoiseIterations (input) Number of iterations for noise measuring function. Set to 0 to deactivate
 * @return ERROR_CODE 
 */
ERROR_CODE hak_record_imus_data(int imus_num, double time, int measureNoiseIterations);

/**
 * @brief A program that calibrates through the use of automatic two axes calib and measures elbow angles
 * 
 * @details
 *      The program performs the following steps:
 *      - Set up database logging to CSV
 *      - Search ans initialize 2 IMU sensors connected to COM ports
 *      - Attach IMU data retrieve callback 
 *      - Loop while gathering IMU data of arbitrary motions to be used for calibration
 *      - Run Two Axes Automatic calibration algorithm to obtain rotation vectors of the elbow
 *      - Loop during the requested ammount of time measuring the elbow angles
 *      - End
 * 
 * @param time (input) Total monitoring time in seconds
 * @param shoulder (input) Compute shoulder angles when true
 * @param elbow (input) Compute elbow angles when true
 * @return ERROR_CODE 
 */
ERROR_CODE hak_two_axes_auto_calib_and_kinematics(double time, bool shoulder, bool elbow);

/**
 * @brief A program that calibrates using a predefined pose and uses it on kinematics
 * 
 * @details
 *      The program performs the following steps:
 *      - Set up database logging to CSV
 *      - Search ans initialize 2 IMU sensors connected to COM ports
 *      - Attach IMU data retrieve callback 
 *      - Get IMU data when in T-pose 
 *      - Set calibration transformation
 *      - Loop during the requested ammount of time performing the kinematics
 *      - End
 * 
 * @param time (input) Total monitoring time in seconds
 * @param shoulder (input) Compute shoulder angles when true
 * @return ERROR_CODE 
 */
ERROR_CODE hak_static_calib_kinematics(double time, bool shoulder);

/**
 * @brief A program that calibrates through the use of automatic two axes calib and measures elbow angles until the program is closed
 * 
 * @details
 *      The program performs the following steps:
 *      - Remove CSV logging
 *      - Reduce log file logging level
 *      - Search ans initialize 2 IMU sensors connected to COM ports
 *      - Attach IMU data retrieve callback 
 *      - Loop while gathering IMU data of arbitrary motions to be used for calibration
 *      - Run Two Axes Automatic calibration algorithm to obtain rotation vectors of the elbow
 *      - Loop forever correcting calibration and computing arm position
 *      - End
 * 
 * @param time (input) Total monitoring time in seconds
 * @param shoulder (input) Compute shoulder angles when true
 * @param elbow (input) Compute elbow angles when true
 * @param net (input) Send current position over the net
 * @return ERROR_CODE 
 */
ERROR_CODE hak_two_axes_auto_calib_and_kinematics_forever(bool shoulder, bool elbow, bool net); 



/*TODO*/
ERROR_CODE hak_laidig2017();