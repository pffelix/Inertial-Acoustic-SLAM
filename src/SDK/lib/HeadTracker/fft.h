// fft.h: FFT, fast convolution and fast correlation routines. Uses various C FFT libraries and native Radix 2 implementation.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "debugging.h"
#include "nativefft.h"
#include "debugging.h"
#include "Windows.h"

#ifdef __cplusplus
extern "C" 
{
#endif

	// Fast 1D Fourier transform with selected library using (fast) float real array in INTEL FFT Pack format. Assumes that time signal is real input
	// passed in array x must be two times large as x_N to allow expansion to complex values
	// Flag=1: Forward (real input), Flag=-1 inverse backward (complex input) Fourier transformation
	// Pack format info: https://software.intel.com/en-us/ipp-dev-reference-packed-formats
	/*
	If fft_library:
		intelmkl -> IntelMKL libarary is used in case x_N is power of 2, if no power of 2 JUCE intern FFT library is used)
		native -> Radix 2 implementation is used

	Warning: 
		intelmkl -> delivers only right-sided spectrum, if x_N is even -> 1.st bin DC, ceil(N/2) bins after it rightside spectrum, left side can be derived by taking the first floor(N/2) bins of right sided spectrum and append them inverse symetric with complex conjugate for imaginary part
		intelmkl -> full spectrum could be set by ignore_negative_freqs = false, but leads to artifcats when the derived signal is directly inserted in inverse fourier transfrom (in the end of singal articats) -> reason: wrong implementation of the parameter in the JUCE framework
		intelmkl -> Y calculation from X And H cannot be written only until right sided spectrum, as output of intelmkl requires empty output value (a certain program internal number) of left sighted spectrum too as input for inverse transformation
		native -> delivers a group delay, such that impulse is shifted to middle of time signal
	*/
	void fast_fft(float *x, long x_N, struct ffts *fft, int flag, char * fft_library);

	// Fast fft convolution algorithm using (fast) float real array in INTEL FFT Pack format. Assumes that time signal is real input. 
	// fft_type can be "filter" for normal convolution between x and h, "ir" with pre-set inverse impulse h, "cor" for crosscorrelation between x and h
	void fast_fft_convolution(float* x, long x_N, float* h, long h_N, float* y, long y_N, long y_Nshift, struct ffts* fft, char* fft_library, char* fft_type, long fft_flim_low, long fft_flim_high, long sampling_rate);
	// Full 1D Fourier transform with selected library using (slower) self defined complex C struct cpx of params.h. Assumes that time signal is real input
	// Flag=1: Forward (real input), Flag=-1 inverse backward (complex input) Fourier transformation
	// Shift of output signal about y_Nshift bins to the left can be applied to derive FIR filtered signal with same length as input
	// frequency limits in Hz can be applied where cross-correlation is performed
	/*
	If fft_library:
		intelmkl -> IntelMKL libarary is used in case x_N is power of 2, if no power of 2 JUCE intern FFT library is used)
		native -> Radix 2 implementation is used

	Warning: 
		intelmkl -> delivers only right-sided spectrum, if x_N is even -> 1.st bin DC, ceil(N/2) bins after it rightside spectrum, left side can be derived by taking the first floor(N/2) bins of right sided spectrum and append them inverse symetric with complex conjugate for imaginary part
		intelmkl -> full spectrum could be set by ignore_negative_freqs = false, but leads to artifcats when the derived signal is directly inserted in inverse fourier transfrom (in the end of singal articats) -> reason: wrong implementation of the parameter in the JUCE framework
		intelmkl -> Y calculation from X And H cannot be written only until right sided spectrum, as output of intelmkl requires empty output value (a certain program internal number) of left sighted spectrum too as input for inverse transformation
		native -> delivers a group delay, such that impulse is shifted to middle of time signal
	*/
	void full_fft(struct cpx *x, long x_N, int flag, char * fft_library); // (depreciated)


	// Full fft crosscorrelation algorithm using (slower) self defined complex C struct cpx of params.h. Assumes that time signal is real input
	void full_fft_convolution(float* x, long x_N, float* h, long h_N, float* y, char* fft_library); // (depreciated)

	// Fast fft crosscorrelation algorithm  using (fast) float real array in INTEL FFT Pack format. Assumes that time signal is real input
	void fast_fft_crosscorrelation(float* x, long x_N, float* h, long h_N, float* y, long y_N, struct ffts *fft, char* fft_library, char* cor_type, long fft_flim_low, long fft_flim_high, long sampling_rate);

	// Full fft crosscorrelation algorithm  using (slower) self defined C complex struct cpx of params.h. Assumes that time signal is real input
	void full_fft_crosscorrelation(float* x, long x_N, float* h, long h_N, float* y, char* fft_library); // (depreciated)


#ifdef __cplusplus
};
#endif
