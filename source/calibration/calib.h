
#ifndef __calib_h__
#define __calib_h__

#include "errors.h"


#define DEFAULT_ROT_AXIS_CALIB_WINDOW  (3)
#define DEFAULT_ROT_AXIS_CALIB_STEP_SZ (0.3)
#define DEFAULT_ROT_AXIS_CALIB_MIN_VEL (3e-1)

/**
 * @brief Calibrate a rotation axis for a 1 DOF joint knowing its angular velocity
 * 
 * @param omegaR (input) Current angular velocity
 * @param rotationV (input/output) Rotation vector
 * @return ERROR_CODE:
 *  - RET_OK on success
 *  - RET_ERROR otherwise
 */
ERROR_CODE cal_automatic_rotation_axis_calibrate(
    double omegaR[3],
    double rotationV[3]);

#endif /* __calib_h__ */