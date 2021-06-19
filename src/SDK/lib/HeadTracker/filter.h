// filter.h: lowpass, highpass, bandpass and notch FIR filters using standard windows
// Copyright 2020, Felix Pfreundtner, All rights reserved.
// adapted from http://www.iowahills.com/Example%20Code/WindowedFIRFilterWebCode.txt


#pragma once
#include "params.h"
#include "debugging.h"
#include <math.h>
#include <new.h>     // defines new(std::nothrow)



#ifdef __cplusplus
extern "C" {
#endif


	// float functions
	////////////////////////////////////////////////////////////////////////////////////////////////
	// generate fir filter coefficients for specified window that can be used for convolution
	void basic_fir(float *FirCoeff, long NumTaps, enum TPassTypeName PassType, float OmegaC, float BW, enum TWindowType WindowType, float WinBeta);
	
	// window data with specified window
	void window_data(float *x, long N, enum TWindowType WindowType, float Alpha, float Beta, bool UnityGain);
	
	// return kaiser window applied into diretion: 0=both, -1=left, 1=right
	float* kaiserf(long N, bool twosided, float beta, bool unity_gain);

	// window signal x with kaiser window applied into diretion: 0=both, -1=left, 1=right
	void kaiserf_apply(float* x, float* WinCoeff, long N, int direction, float stepsize, bool x_integrate);

	// bessel function of given order N
	float bessel(float N);

	// sinc function
	float sinc(float x);




	// double functions
	////////////////////////////////////////////////////////////////////////////////////////////////
	void basic_fir_double(double *FirCoeff, int NumTaps, enum TPassTypeName PassType, double OmegaC, double BW, enum TWindowType WindowType, double WinBeta);
	void window_data_double(double *x, int N, enum TWindowType WindowType, double Alpha, double Beta, bool UnityGain);
	double bessel_double(double x);
	double sinc_double(double x);


#ifdef __cplusplus
};
#endif