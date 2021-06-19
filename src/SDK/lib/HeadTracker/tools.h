// Copyright 2020, Felix Pfreundtner, All rights reserved.
// tools.h: Toolkits for algorithm excecution.

#pragma once
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "params.h"
#include "debugging.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	// read wave file with ffmpeg
	// void read_wav_ffmpeg(float * buf, long buf_N);

	// convert char to unsigned long
	unsigned long str_to_ulong(char* str);

	// concatenate two chars to new char (make space for the new string: char * str = malloc(strlen(str1) + strlen(str2) + 1 (delimiter) + 1 (end)))
	void char_concatenate(const char* str1, const char* str2, const char* delimiter, char* str);
	
	// concatenate multiple chars to new char (make space for the new string: char * str = malloc(strlen(str1) + strlen(str2) + str_N - 1 (delimiter) ... + 1 (end)))
	void char_concatenatemulti(const char** strs, int str_N, const char* delimiter, char* str);

	// convert absolute value in decibel value (energy)
	float indb(float x_abs);

	// convert decibel value (energy) in absolute value 
	float inabs(float x_db);

	// convert bin number in upsampled bin number
	long bin_to_up(long bin_raw, int Nup);

	// convert bin number in correlated bin number
	long bin_to_cor(long bin_raw);

	// maximum of two float values
	float maximum(float x1, float x2);

	// minimum of two float values
	float minimum(float x1, float x2);
	
	// maximum of two long values
	long maximum_long(long x1, long x2);

	// minimum of two long values
	long minimum_long(long x1, long x2);

	// maximum of two int values
	int maximum_int(int x1, int x2);

	// minimum of two int values
	int minimum_int(int x1, int x2);

	// maximum of pointer array with Nchannels
	float maximum_ptr(float** x, long x_N, int Nchannels);

	// absolute maximum of pointer array with Nchannels
	float absmaximum_ptr(float** x, long x_N, int Nchannels);

	// absolute of flaot value (2 x faster than fabsf)
	float myAbs(float x);

	// absolute of long value
	long abs_long(long x);

	// convert milliseconds to samples
	long msec_to_N(float msec, long fs);

	// convert samples to metre
	float N_to_m(long N, long fs, float sos);

	// convert seconds to cm of time of flight with speed of sound sos
	float sec_to_cm(float sec, float sos);

	// convert metre to ms with speed of sound sos
	float m_to_msec(float m, float sos);

	// convert seconds to m of time of flight with speed of sound sos
	void sec_to_m(float sec, float sos, float* m);

	// convert m to seconds of time of flight with speed of sound sos
	void m_to_sec(float m, float sos, float* sec);

	// convert degree to radian
	float deg_to_rad(float deg);

	// convert radian to degree
	float rad_to_deg(float rad);

	// normalize complex struct
	void cpx_normalize (struct cpx* x, long N);

	// absolute of complex number
	float cpx_abs(float* a_real, float* a_imag);

	// multiply complex numbers
	void cpx_multiply(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag);

	// divide complex numbers
	void cpx_divide(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag);

	// add complex numbers
	void cpx_add(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag);

	// substract complex numbers
	void cpx_substract(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag);
	
	// complex conjugate of imaginary part of complex number
	float cpx_conj(float* a_imag);

	//  get maximum of complex struct
	struct cpx cpx_max(struct cpx* x, long N);

	// check whether number is power of 2
	bool is_power_of_2(long x);

	// calculate next power of 2 to number
	unsigned long next_power_of_2(unsigned long n);
	
	// calculate modulus a % b
	long modulus(long a, long b);

	// calculate modulus float a % b
	void modulusf(float a, long b, float* y);

	// derive smallest distance between bin n and modulo length N
	long modulus_diff(long n, long N);

	// correct x by limit
	void correct_limit(float* x, float limit_low, float limit_high);

	// correct x by limit with modulus
	void correct_limit_circular(float* x, float limit_low, float limit_high);

	// arcus-cotangens for input range x=[0, pi]
	float acot_0_to_pi(float x);

	// arcus-cotangens for input range x=[-pi/2, pi/2]
	float acot(float x);

	// ask for permission to locked variable
	void ask_lock(bool* lock);

	// close permission on locked variable
	void close_lock(bool* lock);

#ifdef __cplusplus
};
#endif