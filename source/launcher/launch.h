/**
 * @file boot.h
 * @author Germán Moreno Escudero
 * @brief This file is used to manage the initialization and cleaning of all pack resources
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "errors.h"

/**
 * @brief Initialize all required resources for Human Arm Kinematics package
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE hak_initialize();
/**
 * @brief Remove all used resources for Human Arm Kinematics package
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE hak_terminate();



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