/**
 * @file general.h
 * @author German Moreno Escudero
 * @brief This module includes generic functions and constants
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __functions_h__
#define __functions_h__

#include "errors.h"

/**
 * @brief The maximum string lenght of a COM port
 */
#define COM_PORTS_LENGTH (64)
/**
 * @brief Maximum number of COM ports to be discovered
 */
#define COM_PORTS_MAX_NUM (4)
/**
 * @brief The maximum number of values to log to the csv file
 */
#define CSV_FILE_VALUES_NUMBER (100)
/**
 * @brief The maximum length of a CSV header
 */
#define CSV_HEADER_MAX_LENGTH (64)
/**
 * @brief Length of the log file name
 */
#define LOG_FILE_NAME_LENGTH (128)
/**
 * @brief Length of the csv file name
 */
#define CSV_FILE_NAME_LENGTH (128)

/**
 * @brief COM ports structure
 */
typedef struct COM_PORTS_STRUCT {
    /**
     * @brief Number of COM ports
     */
    unsigned int ports_number;
    /**
     * @brief List of port names
     */
    char ports_names[COM_PORTS_MAX_NUM][COM_PORTS_LENGTH];
} COM_PORTS; 

typedef enum TRACE_LEVEL_ENUM {
    NONE = 0,
    ERROR,
    WARNING,
    INFO, // Default
    DEBUG,

    NUMBER_OF_LEVELS
} TRACE_LEVEL;

/**
 * @brief Macro that gets the maximum value among two given
 */
#ifndef MAX
#define MAX(a,b) ((a>b)?a:b)
#endif
/**
 * @brief Macro that gets the minimum value among two given
 */
#ifndef MIN
#define MIN(a,b) ((a<b)?a:b)
#endif
/**
 * @brief Retrieve all COM ports available in the system
 * 
 * @param discoveredPorts Output structure containing retrieved COM ports data
 * @return true If the COM ports discovery performs correctly even if there are not any COM ports available
 * @return false If any error appears while discovering COM ports
 */
ERROR_CODE com_ports_list(COM_PORTS *discoveredPorts);

/**
 * @brief Sleep with seconds precision
 * 
 * @param seconds (input) Seconds to wait
 */
void sleep_s(int seconds);
/**
 * @brief Sleep with milliseconds precision
 * 
 * @param millis (input) Milliseconds to wait
 */
void sleep_ms(int millis);
/**
 * @brief Get the current system timestamp in milliseconds
 * 
 * @return double: current system timestamp in milliseconds
 */
double sys_timestamp_get();

/**
 * @brief Set up log file 
 * 
 * @return ERROR_CODE: RET_OK on success
 */
ERROR_CODE log_file_initalize();
/**
 * @brief Set a non default log file name
 * 
 * @param name (input) Non default log file name
 * @return ERROR_CODE 
 */
ERROR_CODE log_file_name_set(char name[LOG_FILE_NAME_LENGTH]);
/**
 * @brief Set a non default csv file name
 * 
 * @param name (input) Non default csv file name
 * @return ERROR_CODE 
 */
ERROR_CODE csv_file_name_set(char name[CSV_FILE_NAME_LENGTH]);
/**
 * @brief Set the tracing level to moderate the ammount of data print
 * 
 * @param lvl The new trace level
 * @param file_lvl The new trace level for log file
 * @return ERROR_CODE: RET_OK on success and RET_ARG_ERROR when invalid level given
 */
ERROR_CODE trace_level_set(TRACE_LEVEL lvl, TRACE_LEVEL file_lvl);

/**
 * @brief Log a debug string
 * 
 * @param text The text to be formated
 * @param ... The formating args
 */
void dbg_str(const char *text, ...);
/**
 * @brief Log a informative string
 * 
 * @param text The text to be formated
 * @param ... The formating args
 */
void log_str(const char *text, ...);
/**
 * @brief Log a warning string
 * 
 * @param text The text to be formated
 * @param ... The formating args
 */
void wrn_str(const char *text, ...);
/**
 * @brief Log an error string
 * 
 * @param text The text to be formated
 * @param ... The formating args
 */
void err_str(const char *text, ...);

/**
 * @brief Set the first line of a csv file with a set of given headers
 * 
 * @param headers (input) Array of headers
 * @param data_num (input) The number of data instances
 */
void csv_headers_set(const char headers[CSV_FILE_VALUES_NUMBER][CSV_HEADER_MAX_LENGTH], int data_num);

/**
 * @brief Log a set of values to the csv file
 * 
 * @param data (input) Array of values
 */
void csv_log(const double data[CSV_FILE_VALUES_NUMBER]);

#endif /* __functions_h__ */