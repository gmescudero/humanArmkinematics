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
#define COM_PORTS_MAX_NUM (7)

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

/**
 * @brief Retrieve all COM ports available in the system
 * 
 * @param discoveredPorts Output structure containing retrieved COM ports data
 * @return true If the COM ports discovery performs correctly even if there are not any COM ports available
 * @return false If any error appears while discovering COM ports
 */
ERROR_CODE com_ports_list(COM_PORTS *discoveredPorts);

typedef enum TRACE_LEVEL_ENUM {
    NONE = 0,
    ERROR,
    WARNING,
    INFO, // Default
    DEBUG,

    NUMBER_OF_LEVELS
} TRACE_LEVEL;


/**
 * @brief Set the tracing level to moderate the ammount of data print
 * 
 * @param lvl The new trace level
 * @return ERROR_CODE: RET_OK on success and RET_ARG_ERROR when invalid level given
 */
ERROR_CODE trace_level_set(TRACE_LEVEL lvl);

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

#endif /* __functions_h__ */