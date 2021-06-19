// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "rotation.h"


void rotation_between_vecs_to_mat3(float** m, const float* v1, const float* v2)
{
    float axis[3];
    /* avoid calculating the angle */
    float angle_sin;
    float angle_cos;

    cross_v3_v3v3(axis, v1, v2);

    angle_sin = normalize_v3(axis);
    angle_cos = dot_v3v3(v1, v2);

    if (angle_sin > FLT_EPSILON) {
axis_calc:
        axis_angle_normalized_to_mat3_ex(m, axis, angle_sin, angle_cos);
    }
    else {
        /* Degenerate (co-linear) vectors */
        if (angle_cos > 0.0f) {
            /* Same vectors, zero rotation... */
            unit_m3(m);
        }
        else {
            /* Colinear but opposed vectors, 180 rotation... */
            ortho_v3_v3(axis, v1);
            normalize_v3(axis);
            angle_sin =  0.0f;  /* sin(M_PI) */
            angle_cos = -1.0f;  /* cos(M_PI) */
            goto axis_calc;
        }
    }
}


/* -------------------------------------------------------------------- */
/* Math Lib */

static void unit_m3(float** m)
{
    m[0][0] = m[1][1] = m[2][2] = 1.0;
    m[0][1] = m[0][2] = 0.0;
    m[1][0] = m[1][2] = 0.0;
    m[2][0] = m[2][1] = 0.0;
}

static float dot_v3v3(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross_v3_v3v3(float* r, const float* a, const float* b)
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

static void mul_v3_v3fl(float* r, const float* a, float f)
{
    r[0] = a[0] * f;
    r[1] = a[1] * f;
    r[2] = a[2] * f;
}

static float normalize_v3_v3(float* r, const float* a)
{
    float d = dot_v3v3(a, a);

    if (d > 1.0e-35f) {
        d = sqrtf(d);
        mul_v3_v3fl(r, a, 1.0f / d);
    }
    else {
        d = r[0] = r[1] = r[2] = 0.0f;
    }

    return d;
}

static float normalize_v3(float* n)
{
    return normalize_v3_v3(n, n);
}

static int axis_dominant_v3_single(const float* vec)
{
    const float x = fabsf(vec[0]);
    const float y = fabsf(vec[1]);
    const float z = fabsf(vec[2]);
    return ((x > y) ?
           ((x > z) ? 0 : 2) :
           ((y > z) ? 1 : 2));
}

static void ortho_v3_v3(float* p, const float* v)
{
    const int axis = axis_dominant_v3_single(v);

    switch (axis) {
        case 0:
            p[0] = -v[1] - v[2];
            p[1] =  v[0];
            p[2] =  v[0];
            break;
        case 1:
            p[0] =  v[1];
            p[1] = -v[0] - v[2];
            p[2] =  v[1];
            break;
        case 2:
            p[0] =  v[2];
            p[1] =  v[2];
            p[2] = -v[0] - v[1];
            break;
    }
}

/* axis must be unit length */
static void axis_angle_normalized_to_mat3_ex(
	float** mat, const float* axis,
        const float angle_sin, const float angle_cos)
{
    float nsi[3], ico;
    float n_00, n_01, n_11, n_02, n_12, n_22;

    ico = (1.0f - angle_cos);
    nsi[0] = axis[0] * angle_sin;
    nsi[1] = axis[1] * angle_sin;
    nsi[2] = axis[2] * angle_sin;

    n_00 = (axis[0] * axis[0]) * ico;
    n_01 = (axis[0] * axis[1]) * ico;
    n_11 = (axis[1] * axis[1]) * ico;
    n_02 = (axis[0] * axis[2]) * ico;
    n_12 = (axis[1] * axis[2]) * ico;
    n_22 = (axis[2] * axis[2]) * ico;

    mat[0][0] = n_00 + angle_cos;
    mat[0][1] = n_01 + nsi[2];
    mat[0][2] = n_02 - nsi[1];
    mat[1][0] = n_01 - nsi[2];
    mat[1][1] = n_11 + angle_cos;
    mat[1][2] = n_12 + nsi[0];
    mat[2][0] = n_02 + nsi[1];
    mat[2][1] = n_12 - nsi[0];
    mat[2][2] = n_22 + angle_cos;
}