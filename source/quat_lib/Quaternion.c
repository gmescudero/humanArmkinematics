// Copyright (C) 2019 Martin Weigel <mail@MartinWeigel.com>
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

/**
 * @file    Quaternion.c
 * @brief   A basic quaternion library written in C
 * @date    2019-11-28
 */
#include "Quaternion.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>

void Quaternion_set(double w, double v1, double v2, double v3, Quaternion* output)
{
    assert(output != NULL);
    output->w = w;
    output->v[0] = v1;
    output->v[1] = v2;
    output->v[2] = v3;
}

void Quaternion_setIdentity(Quaternion* q)
{
    assert(q != NULL);
    Quaternion_set(1, 0, 0, 0, q);
}

void Quaternion_copy(Quaternion* q, Quaternion* output)
{
    Quaternion_set(q->w, q->v[0], q->v[1], q->v[2], output);
}

bool Quaternion_equal(Quaternion* q1, Quaternion* q2)
{
    bool equalW  = fabs(q1->w - q2->w) <= QUATERNION_EPS;
    bool equalV0 = fabs(q1->v[0] - q2->v[0]) <= QUATERNION_EPS;
    bool equalV1 = fabs(q1->v[1] - q2->v[1]) <= QUATERNION_EPS;
    bool equalV2 = fabs(q1->v[2] - q2->v[2]) <= QUATERNION_EPS;
    return equalW && equalV0 && equalV1 && equalV2;
}

void Quaternion_fprint(FILE* file, Quaternion* q)
{
    if (NULL == file)
    {
        printf("(%.3f, %.3f, %.3f, %.3f)", q->w, q->v[0], q->v[1], q->v[2]);
    }
    else
    {
        fprintf(file, "(%.3f, %.3f, %.3f, %.3f)",
            q->w, q->v[0], q->v[1], q->v[2]);
    }
}


void Quaternion_fromAxisAngle(double axis[3], double angle, Quaternion* output)
{
    assert(output != NULL);
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
    output->w = cos(angle / 2.0);
    double c = sin(angle / 2.0);
    output->v[0] = c * axis[0];
    output->v[1] = c * axis[1];
    output->v[2] = c * axis[2];
}

double Quaternion_toAxisAngle(Quaternion* q, double output[3])
{
    assert(output != NULL);
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
    double angle = 2.0 * acos(q->w);
    double divider = sqrt(1.0 - q->w * q->w);

    if(divider != 0.0) {
        // Calculate the axis
        output[0] = q->v[0] / divider;
        output[1] = q->v[1] / divider;
        output[2] = q->v[2] / divider;
    } else {
        // Arbitrary normalized axis
        output[0] = 1;
        output[1] = 0;
        output[2] = 0;
    }
    return angle;
}

void Quaternion_fromXRotation(double angle, Quaternion* output)
{
    assert(output != NULL);
    double axis[3] = {1.0, 0, 0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromYRotation(double angle, Quaternion* output)
{
    assert(output != NULL);
    double axis[3] = {0, 1.0, 0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromZRotation(double angle, Quaternion* output)
{
    assert(output != NULL);
    double axis[3] = {0, 0, 1.0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromEulerZYX(double eulerZYX[3], Quaternion* output)
{
    assert(output != NULL);
    // Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double cy = cos(eulerZYX[2] * 0.5);
    double sy = sin(eulerZYX[2] * 0.5);
    double cr = cos(eulerZYX[0] * 0.5);
    double sr = sin(eulerZYX[0] * 0.5);
    double cp = cos(eulerZYX[1] * 0.5);
    double sp = sin(eulerZYX[1] * 0.5);

    output->w = cy * cr * cp + sy * sr * sp;
    output->v[0] = cy * sr * cp - sy * cr * sp;
    output->v[1] = cy * cr * sp + sy * sr * cp;
    output->v[2] = sy * cr * cp - cy * sr * sp;
}

void Quaternion_toEulerZYX(Quaternion* q, double output[3])
{
    assert(output != NULL);

    // Roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q->w * q->v[0] + q->v[1] * q->v[2]);
    double cosr_cosp = +1.0 - 2.0 * (q->v[0] * q->v[0] + q->v[1] * q->v[1]);
    output[0] = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = +2.0 * (q->w * q->v[1] - q->v[2] * q->v[0]);
    if (fabs(sinp) >= 1)
        output[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        output[1] = asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q->w * q->v[2] + q->v[0] * q->v[1]);
    double cosy_cosp = +1.0 - 2.0 * (q->v[1] * q->v[1] + q->v[2] * q->v[2]);
    output[2] = atan2(siny_cosp, cosy_cosp);
}

void Quaternion_conjugate(Quaternion* q, Quaternion* output)
{
    assert(output != NULL);
    output->w = q->w;
    output->v[0] = -q->v[0];
    output->v[1] = -q->v[1];
    output->v[2] = -q->v[2];
}

double Quaternion_norm(Quaternion* q)
{
    assert(q != NULL);
    return sqrt(q->w*q->w + q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2]);
}

void Quaternion_normalize(Quaternion* q, Quaternion* output)
{
    assert(output != NULL);
    double len = Quaternion_norm(q);
    Quaternion_set(
        q->w / len,
        q->v[0] / len,
        q->v[1] / len,
        q->v[2] / len,
        output);
}

void Quaternion_multiply(Quaternion* q1, Quaternion* q2, Quaternion* output)
{
    assert(output != NULL);
    Quaternion result;

    /*
    Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
             a*e - b*f - c*g - d*h
        + i (b*e + a*f + c*h- d*g)
        + j (a*g - b*h + c*e + d*f)
        + k (a*h + b*g - c*f + d*e)
    */
    result.w =    q1->w   *q2->w    - q1->v[0]*q2->v[0] - q1->v[1]*q2->v[1] - q1->v[2]*q2->v[2];
    result.v[0] = q1->v[0]*q2->w    + q1->w   *q2->v[0] + q1->v[1]*q2->v[2] - q1->v[2]*q2->v[1];
    result.v[1] = q1->w   *q2->v[1] - q1->v[0]*q2->v[2] + q1->v[1]*q2->w    + q1->v[2]*q2->v[0];
    result.v[2] = q1->w   *q2->v[2] + q1->v[0]*q2->v[1] - q1->v[1]*q2->v[0] + q1->v[2]*q2->w   ;

    *output = result;
}

void Quaternion_rotate(Quaternion* q, double v[3], double output[3])
{
    assert(output != NULL);
    double result[3];

    double ww = q->w * q->w;
    double xx = q->v[0] * q->v[0];
    double yy = q->v[1] * q->v[1];
    double zz = q->v[2] * q->v[2];
    double wx = q->w * q->v[0];
    double wy = q->w * q->v[1];
    double wz = q->w * q->v[2];
    double xy = q->v[0] * q->v[1];
    double xz = q->v[0] * q->v[2];
    double yz = q->v[1] * q->v[2];

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
    // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
    // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

    result[0] = ww*v[0] + 2*wy*v[2] - 2*wz*v[1] +
                xx*v[0] + 2*xy*v[1] + 2*xz*v[2] -
                zz*v[0] - yy*v[0];
    result[1] = 2*xy*v[0] + yy*v[1] + 2*yz*v[2] +
                2*wz*v[0] - zz*v[1] + ww*v[1] -
                2*wx*v[2] - xx*v[1];
    result[2] = 2*xz*v[0] + 2*yz*v[1] + zz*v[2] -
                2*wy*v[0] - yy*v[2] + 2*wx*v[1] -
                xx*v[2] + ww*v[2];

    // Copy result to output
    output[0] = result[0];
    output[1] = result[1];
    output[2] = result[2];
}

void Quaternion_slerp(Quaternion* q1, Quaternion* q2, double t, Quaternion* output)
{
    Quaternion result;

    // Based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    double cosHalfTheta = q1->w*q2->w + q1->v[0]*q2->v[0] + q1->v[1]*q2->v[1] + q1->v[2]*q2->v[2];

    // if q1=q2 or qa=-q2 then theta = 0 and we can return qa
    if (abs(cosHalfTheta) >= 1.0) {
        Quaternion_copy(q1, output);
        return;
    }

    double halfTheta = acos(cosHalfTheta);
    double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
    // If theta = 180 degrees then result is not fully defined
    // We could rotate around any axis normal to q1 or q2
    if (fabs(sinHalfTheta) < QUATERNION_EPS) {
        result.w = (q1->w * 0.5 + q2->w * 0.5);
        result.v[0] = (q1->v[0] * 0.5 + q2->v[0] * 0.5);
        result.v[1] = (q1->v[1] * 0.5 + q2->v[1] * 0.5);
        result.v[2] = (q1->v[2] * 0.5 + q2->v[2] * 0.5);
    }

    // Calculate Quaternion
    double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
    double ratioB = sin(t * halfTheta) / sinHalfTheta;
    result.w = (q1->w * ratioA + q2->w * ratioB);
    result.v[0] = (q1->v[0] * ratioA + q2->v[0] * ratioB);
    result.v[1] = (q1->v[1] * ratioA + q2->v[1] * ratioB);
    result.v[2] = (q1->v[2] * ratioA + q2->v[2] * ratioB);

    *output = result;
}

/*********************************************************************************************************************/
#include "constants.h"
#include "vector3.h"
#include "constants.h"
#include "general.h"

ERROR_CODE quaternion_exponential(Quaternion q, Quaternion *q_exp) {
    ERROR_CODE status;
    double norm;
    double aux;

    // Check arguments
    if (NULL == q_exp) return RET_ARG_ERROR;

    status = vector3_norm(q.v,&norm);
    if (RET_OK == status) {
        if (QUATERNION_EPS > norm) {
            q_exp->w    = exp(q.w);
            q_exp->v[0] = 0.0;
            q_exp->v[1] = 0.0;
            q_exp->v[2] = 0.0;
        }
        else {
            aux = exp(q.w)*sin(norm)/norm;
            q_exp->w = exp(q.w)*cos(norm);
            q_exp->v[0] = q.v[0]*aux;
            q_exp->v[1] = q.v[1]*aux;
            q_exp->v[2] = q.v[2]*aux;
        }
    }

    return status;
}

ERROR_CODE quaternion_ang_vel_apply(Quaternion q, double T, double ang_vel[3], Quaternion *q_rot) {
    ERROR_CODE status = RET_OK;
    Quaternion q_w;
    Quaternion q_exp;
    Quaternion q_result;

    // Check arguments
    if (-QUATERNION_EPS > T) return RET_ARG_ERROR;
    if (NULL == ang_vel) return RET_ARG_ERROR;
    if (NULL == q_rot) return RET_ARG_ERROR;

    if (QUATERNION_EPS > T) {
        Quaternion_copy(&q, q_rot);
        wrn_str("Time period is 0 for quaternion angular velocity apply");
    }
    else {
        q_w.w = 0.0;
        q_w.v[0] = T*ang_vel[0]*0.5;
        q_w.v[1] = T*ang_vel[1]*0.5;
        q_w.v[2] = T*ang_vel[2]*0.5;
    
        status = quaternion_exponential(q_w,&q_exp);
        if (RET_OK == status) {
            Quaternion_multiply(&q_exp, &q, &q_result);
        }
        if (RET_OK == status) {
            Quaternion_copy(&q_result, q_rot);
        }
    }
    return status;
}

void quaternion_buffer_build(Quaternion q, double buffer[4]) {
    buffer[0] = q.w;
    buffer[1] = q.v[0];
    buffer[2] = q.v[1];
    buffer[3] = q.v[2];
}

void quaternion_float_buffer_build(Quaternion q, float buffer[4]) {
    buffer[0] = (float)q.w;
    buffer[1] = (float)q.v[0];
    buffer[2] = (float)q.v[1];
    buffer[3] = (float)q.v[2];
}

void quaternion_from_buffer_build(double buffer[4], Quaternion *q) {
    q->w    = buffer[0];
    q->v[0] = buffer[1];
    q->v[1] = buffer[2];
    q->v[2] = buffer[3];
}

void quaternion_from_float_buffer_build(float buffer[4], Quaternion *q) {
    q->w    = (double)buffer[0];
    q->v[0] = (double)buffer[1];
    q->v[1] = (double)buffer[2];
    q->v[2] = (double)buffer[3];
}

void quaternion_print(Quaternion q, const char *name) {
    if (NULL != name)
        log_str("Quaternion %s: %f, [%f, %f, %f]",name, q.w, q.v[0], q.v[1], q.v[2]);
    else
        log_str("Quaternion: %f, [%f, %f, %f]", q.w, q.v[0], q.v[1], q.v[2]);
}

void quaternion_fromEulerZXY(double eulerZXY[3], Quaternion* output)
{
    assert(output != NULL);

    double cy = cos(eulerZXY[0] * 0.5);
    double sy = sin(eulerZXY[0] * 0.5);
    double cx = cos(eulerZXY[1] * 0.5);
    double sx = sin(eulerZXY[1] * 0.5);
    double cz = cos(eulerZXY[2] * 0.5);
    double sz = sin(eulerZXY[2] * 0.5);

    output->w    = cy*cx*cz - sy*sx*sz;
    output->v[0] = cy*sx*cz - sy*cx*sz;
    output->v[1] = sy*cx*cz + cy*sx*sz;
    output->v[2] = cy*cx*sz + sy*sx*cz;
}

void quaternion_toEulerZXY(Quaternion* q, double output[3]) {
    assert(output != NULL);

    // y-axis rotation
    double sy_cx = 2.0*(-q->w*q->v[1] + q->v[0]*q->v[2]);
    double cy_cx = 1.0 - 2.0*(q->v[0]*q->v[0] + q->v[1]*q->v[1]);
    output[0] = -atan2(sy_cx,cy_cx);

    // x-axis rotation
    double sx = -2.0*(q->v[1]*q->v[2] + q->w*q->v[0]);
    if (fabs(sx) > 1.0-QUATERNION_EPS)
        output[1] = -copysign(M_PI / 2, sx); // use 90 degrees if out of range
    else
        output[1] = -asin(sx);

    // z-axis rotation
    double sz_cx = 2.0*(-q->w*q->v[2] + q->v[0]*q->v[1]);
    double cz_cx = 1.0 - 2.0*(q->v[0]*q->v[0] + q->v[2]*q->v[2]);
    output[2] = -atan2(sz_cx,cz_cx);
}

ERROR_CODE quaternion_between_two_vectors_compute(double v1[3], double v2[3], Quaternion *output) {
    ERROR_CODE status = RET_OK;
    double dotVal;
    double crossVal[3];

    // Get dot product
    status = vector3_dot(v1, v2, &dotVal);
    // Get cross product
    if (RET_OK == status) status = vector3_cross(v1,v2,crossVal);
    // Normalize cross product to serve as rotation axis
    if (RET_OK == status && 1.0-QUATERNION_EPS > fabs(dotVal)) status = vector3_normalize(crossVal, crossVal);
    // Use the rotation axis obtained and the arccosine of the dot value to compute the rotation quaternion
    if (RET_OK == status) Quaternion_fromAxisAngle(crossVal,acos(dotVal),output);

    return status;
} 