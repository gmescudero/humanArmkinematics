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
typedef int ERROR_CODE;

/**
 * @brief Correct execution
 */
#define RET_OK (0)

/**
 * @brief Generic error during execution
 */
#define RET_ERROR (-1)

/**
 * @brief Input or Output value of a function is invalid
 */
#define RET_ARG_ERROR (-2)

/**
 * @brief Procedure could not be done 
 */
#define RET_NO_EXEC (-3)

#endif /* __errors_h__ */