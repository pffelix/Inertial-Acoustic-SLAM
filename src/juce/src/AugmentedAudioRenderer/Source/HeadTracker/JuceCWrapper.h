#pragma once
//#include "JuceHeader.h"
#ifdef __cplusplus
extern "C"
{
#endif
	// init JUCE C Wrapper
	void JuceCWrapperInit(int maximalFftOrder_dynamic, long Nmax, int Channelmax, double samplingRate);

	// calculate forward FFT with JUCE
	void JuceCWrapperFFTForward(float* x, long x_N, bool ignore_negative_freqs);

	// calculate inverse FFT with JUCE
	void JuceCWrapperFFTInverse(float* x, long x_N);

	// read from WAV file
	void JuceCWrapperWAVRead(const char* filepath, float** output, float* sample_rate);

	// write to WAV file
	void JuceCWrapperWAVWrite(float** x, int Nchannels, long Nsamples, const char * filepath, const char* metadata, double samplingRate);

	// close JUCE C Wrapper
	void JuceCWrapperClose();


#ifdef __cplusplus
}
#endif