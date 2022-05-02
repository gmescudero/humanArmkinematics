/**
 * @file errors.h
 * @author German Moreno Escudero
 * @brief Error managing pack
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __errors_h__
#define __errors_h__

/**
 * @brief Type for error codes
 */
typedef enum ERROR_CODE_ENUM {
    /**
     * @brief Successful execution
     */
    RET_OK = 0,
    /**
     * @brief Generic error during execution
     */
    RET_ERROR,
    /**
     * @brief Input or Output value of a function is invalid
     */
    RET_ARG_ERROR,
    /**
     * @brief Nothing has been done 
     */
    RET_NO_EXEC,
}ERROR_CODE;



#endif /* __errors_h__ */