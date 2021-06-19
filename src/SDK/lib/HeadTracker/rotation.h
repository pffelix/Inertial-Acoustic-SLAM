// rotation.h: Rotation between vectors
// modified from: https://stackoverflow.com/questions/23166898/efficient-way-to-calculate-a-3x3-rotation-matrix-from-the-rotation-defined-by-tw
// using Rodrigues rotation formula, post of Nico Schlömer: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/476311#476311
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "debugging.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <float.h>

/* -------------------------------------------------------------------- */
/* Main function */

/**
 * Calculate a rotation matrix from 2 normalized vectors.
 *
 * v1 and v2 must be unit length.
 */
void rotation_between_vecs_to_mat3(float** m, const float* v1, const float* v2);


/* -------------------------------------------------------------------- */
/* Math Lib declarations */

static void unit_m3(float** m);
static float dot_v3v3(const float* a, const float* b);
static float normalize_v3(float* n);
static void cross_v3_v3v3(float* r, const float* a, const float* b);
static void mul_v3_v3fl(float* r, const float* a, float f);
static void ortho_v3_v3(float* p, const float* v);
static void axis_angle_normalized_to_mat3_ex(
        float** mat, const float* axis,
        const float angle_sin, const float angle_cos);



#ifdef __cplusplus
};
#endif