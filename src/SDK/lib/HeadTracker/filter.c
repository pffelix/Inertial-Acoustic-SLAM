// Copyright 2020, Felix Pfreundtner, All rights reserved.
/*
July 15, 2015
Iowa Hills Software LLC
http://www.iowahills.com

If you find a problem with this code, please leave us a note on:
http://www.iowahills.com/feedbackcomments.html

Source: ~Projects\Common\basic_fir_doubleFilterCode.cpp

This generic FIR filter code is described in most textbooks.
e.g. Discrete Time Signal Processing, Oppenheim and Shafer

A nice paper on this topic is:
http://dea.brunel.ac.uk/cmsp/Home_Saeed_Vaseghi/Chapter05-DigitalFilters.pdf

This code first generates either a low pass, high pass, band pass, or notch
impulse response for a rectangular window. It then applies a window to this
impulse response.

There are several windows available, including the Kaiser, Sinc, Hanning,
Blackman, and Hamming. Of these, the Kaiser and Sinc are probably the most useful
for FIR filters because their sidelobe levels can be controlled with the Beta parameter.

This is a typical function call:
basic_fir_double(FirCoeff, NumTaps, PassType, OmegaC, BW, wtKAISER, Beta);
basic_fir_double(FirCoeff, 33, LPF, 0.2, 0.0, wtKAISER, 3.2);
33 tap, low pass, corner frequency at 0.2, BW=0 (ignored in the low pass code),
Kaiser window, Kaiser Beta = 3.2

These variables should be defined similar to this:
double FirCoeff[MAXNUMTAPS];
int NumTaps;                        NumTaps can be even or odd, but must be less than the FirCoeff array size.
TPassTypeName PassType;             PassType is an enum defined in the header file. LPF, HPF, BPF, or NOTCH
double OmegaC  0.0 < OmegaC < 1.0   The filters corner freq, or center freq if BPF or NOTCH (in percent of nyquist frequency fs/2)
double BW      0.0 < BW < 1.0       The filters band width if BPF or NOTCH (in percent of nyquist frequency fs/2)
TWindowType WindowType;             WindowType is an enum defined in the header to be one of these.
                                    wtNONE, wtKAISER, wtSINC, wtHANNING, .... and others.
double Beta;  0 <= Beta <= 10.0     Beta is used with the Kaiser, Sinc, and Sine windows only.
                                    It controls the transition BW and sidelobe level of the filters.


If you want to use it, Kaiser originally defined Beta as follows.
He derived its value based on the desired sidelobe level, dBAtten.
double dBAtten, Beta, Beta1=0.0, Beta2=0.0;
if(dBAtten < 21.0)dBAtten = 21.0;
if(dBAtten > 50.0)Beta1 = 0.1102 * (dBAtten - 8.7);
if(dBAtten >= 21.0 && dBAtten <= 50.0) Beta2 = 0.5842 * pow(dBAtten - 21.0, 0.4) + 0.07886 * (dBAtten - 21.0);
Beta = Beta1 + Beta2;

For the number of required tabs check out:
https://dsp.stackexchange.com/questions/31066/how-many-taps-does-an-fir-filter-need/31077
*/

//---------------------------------------------------------------------------


//#pragma hdrstop     // for Embarcadero's C++ Builder

#include "filter.h"

//#pragma package(smart_init)  // for C++ Builder

#define M_2PI  6.28318530717958647692  // M_PI should be in the math.h file
#define M_2PIf  6.28318530717958647692f  // M_PI should be in the math.h file

//---------------------------------------------------------------------------

// These are the various windows definitions. These windows can be used for either
// FIR filter design or with an FFT for spectral analysis.
// Sourced verbatim from: ~MyDocs\Code\Common\FFTFunctions.cpp
// For definitions, see this article:  http://en.wikipedia.org/wiki/Window_function

// This function has 6 inputs
// Data is the array, of length N, containing the data to to be windowed. 
// This data is either a FIR filter sinc pulse, or the data to be analyzed by an fft.
 
// WindowType is an enum defined in the header file, which is at the bottom of this file.
// e.g. wtKAISER, wtSINC, wtHANNING, wtHAMMING, wtBLACKMAN, ...

// Alpha sets the width of the flat top.
// Windows such as the Tukey and Trapezoid are defined to have a variably wide flat top.
// As can be seen by its definition, the Tukey is just a Hanning window with a flat top.
// Alpha can be used to give any of these windows a partial flat top, except the Flattop and Kaiser.
// Alpha = 0 gives the original window. (i.e. no flat top)
// To generate a Tukey window, use a Hanning with 0 < Alpha < 1
// To generate a Bartlett window (triangular), use a Trapezoid window with Alpha = 0.
// Alpha = 1 generates a rectangular window in all cases. (except the Flattop and Kaiser)


// Beta is used with the Kaiser, Sinc, and Sine windows only.
// These three windows are primarily used for FIR filter design, not spectral analysis.
// In FIR filter design, Beta controls the filter's transition bandwidth and the sidelobe levels.
// The code ignores Beta except in the Kaiser, Sinc, and Sine window cases.

// UnityGain controls whether the gain of these windows is set to unity.
// Only the Flattop window has unity gain by design. The Hanning window, for example, has a gain
// of 1/2.  UnityGain = true will set the gain of all these windows to 1.
// Then, when the window is applied to a signal, the signal's energy content is preserved.
// Don't use this with FIR filter design however. Since most of the energy in an FIR sinc pulse
// is in the middle of the window, the window needs a peak amplitude of one, not unity gain.
// Setting UnityGain = true will simply cause the resulting FIR filter to have excess gain.

// If using these windows for FIR filters, start with the Kaiser, Sinc, or Sine windows and
// adjust Beta for the desired transition BW and sidelobe levels (set Alpha = 0).
// While the FlatTop is an excellent window for spectral analysis, don't use it for FIR filter design.
// It has a peak amplitude of ~ 4.7 which causes the resulting FIR filter to have about this much gain.
// It works poorly for FIR filters even if you adjust its peak amplitude.
// The Trapezoid also works poorly for FIR filter design.

// If using these windows with an fft for spectral analysis, start with the Hanning, Gauss, or Flattop.
// When choosing a window for spectral analysis, you must trade off between resolution and amplitude accuracy.
// The Hanning has the best resolution while the Flatop has the best amplitude accuracy.
// The Gauss is midway between these two for both accuracy and resolution.
// These three were the only windows available in the HP 89410A Vector Signal Analyzer. Which is to say,
// unless you have specific windowing requirements, use one of these 3 for general purpose signal analysis.
// Set UnityGain = true when using any of these windows for spectral analysis to preserve the signal's enegy level.


// float functions
////////////////////////////////////////////////////////////////////////////////////////////////

void basic_fir(float *FirCoeff, long NumTaps, enum TPassTypeName PassType, float OmegaC, float BW, enum TWindowType WindowType, float WinBeta)
{
	 long j;
	 float Arg, OmegaLow, OmegaHigh;

	 switch(PassType)
	  {
	   case LPF:
		for(j=0; j<NumTaps; j++)
		 {
		  Arg = (float)j - (float)(NumTaps-1) / 2.0f;
		  FirCoeff[j] = OmegaC * sinc(OmegaC * Arg * M_PIf);
		 }
		break;

	   case HPF:
		if(NumTaps % 2 == 1) // Odd tap counts
		 {
		  for(j=0; j<NumTaps; j++)
		   {
			Arg = (float)j - (float)(NumTaps-1) / 2.0f;
			FirCoeff[j] = sinc(Arg * M_PIf) - OmegaC * sinc(OmegaC * Arg * M_PIf);
		   }
		 }

		else  // Even tap counts
		  {
		   for(j=0; j<NumTaps; j++)
			{
			 Arg = (float)j - (float)(NumTaps-1) / 2.0f;
			 if(Arg == 0.0f)FirCoeff[j] = 0.0f;
			 else FirCoeff[j] = cosf(OmegaC * Arg * M_PIf) / M_PIf / Arg  + cosf(Arg * M_PIf);
			}
		  }
	   break;

	   case BPF:
		OmegaLow  = OmegaC - BW/2.0f;
		OmegaHigh = OmegaC + BW/2.0f;
		for(j=0; j<NumTaps; j++)
		 {
		  Arg = (float)j - (float)(NumTaps-1) / 2.0f;
		  if(Arg == 0.0f)FirCoeff[j] = 0.0f;
		  else FirCoeff[j] =  ( cosf(OmegaLow * Arg * M_PIf) - cosf(OmegaHigh * Arg * M_PIf) ) / M_PIf / Arg;
		 }
	   break;

	   case NOTCH:  // If NumTaps is even for Notch filters, the response at Pi is attenuated.
		OmegaLow  = OmegaC - BW/2.0f;
		OmegaHigh = OmegaC + BW/2.0f;
		for(j=0; j<NumTaps; j++)
		 {
		  Arg = (float)j - (float)(NumTaps-1) / 2.0f;
		  FirCoeff[j] =  sinc(Arg * M_PIf) - OmegaHigh * sinc(OmegaHigh * Arg * M_PIf) - OmegaLow * sinc(OmegaLow * Arg * M_PIf);
		 }
	   break;
	  }

	 // window_data can be used to window data before an FFT. When used for FIR filters we set
	 // Alpha = 0.0 to prevent a flat top on the window and 
	 // set UnityGain = false to prevent the window gain from getting set to unity.
	 window_data(FirCoeff, NumTaps, WindowType, 0.0f, WinBeta, false);

}

//---------------------------------------------------------------------------


void window_data(float *x, long N, enum TWindowType WindowType, float Alpha, float Beta, bool UnityGain)
{
	 if(WindowType == wtNONE) return;

	 long j, M, TopWidth;
	 float dM, *WinCoeff;

	 if(WindowType == wtKAISER ||  WindowType == wtFLATTOP )Alpha = 0.0f;

	 if(Alpha < 0.0f)Alpha = 0.0f;
	 if(Alpha > 1.0f)Alpha = 1.0f;

	 if(Beta < 0.0f)Beta = 0.0f;
	 if(Beta > 10.0f)Beta = 10.0f;

	 WinCoeff  =  (float*)malloc((N + 2) * sizeof(float)); // new(std::nothrow) float[N+2];
	 if(WinCoeff == NULL)
	  {
	   console_log(PRIO_WARNING, "Failed to allocate memory in FFTFunctions::WindowFFTData() ");
	   return;
	  }

	 TopWidth = (long)( Alpha * (float)N );
	 if(TopWidth%2 != 0)TopWidth++;
	 if(TopWidth > N)TopWidth = N;
	 M = N - TopWidth;
	 dM = (float)(M + 1);


	 // Calculate the window for N/2 points, then fold the window over (at the bottom).
	 // TopWidth points will be set to 1.
	 if(WindowType == wtKAISER)
	  {
	   float Arg;
	   for(j=0; j<M; j++)
		{
		 Arg = Beta * sqrtf(1.0f - powf( ((float)(2*j+2) - dM) / dM, 2.0f) );
		 WinCoeff[j] = bessel(Arg) / bessel(Beta);
		}
	  }

	 else if(WindowType == wtSINC)  // Lanczos
	  {
	   for(j=0; j<M; j++)WinCoeff[j] = sinc((float)(2*j+1-M)/dM * M_PIf );
	   for(j=0; j<M; j++)WinCoeff[j] = powf(WinCoeff[j], Beta);
	  }

	 else if(WindowType == wtSINE)  // Hanning if Beta = 2
	  {
	   for(j=0; j<M/2; j++)WinCoeff[j] = sinf((float)(j+1) * M_PIf / dM);
	   for(j=0; j<M/2; j++)WinCoeff[j] = powf(WinCoeff[j], Beta);
	  }

	 else if(WindowType == wtHANNING)
	  {
	   for(j=0; j<M/2; j++)WinCoeff[j] = 0.5f - 0.5f * cosf((float)(j+1) * M_2PIf / dM);
	  }

	 else if(WindowType == wtHAMMING)
	  {
	   for(j=0; j<M/2; j++)
	   WinCoeff[j] = 0.54f - 0.46f * cosf((float)(j+1) * M_2PIf / dM);
	  }

	 else if(WindowType == wtBLACKMAN)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.42f
		 - 0.50f * cosf((float)(j+1) * M_2PIf / dM)
		 + 0.08f * cosf((float)(j+1) * M_2PIf * 2.0f / dM);
		}
	  }


	 // See: http://www.bth.se/fou/forskinfo.nsf/0/130c0940c5e7ffcdc1256f7f0065ac60/$file/ICOTA_2004_ttr_icl_mdh.pdf
	 else if(WindowType == wtFLATTOP)
	  {
	   for(j=0; j<=M/2; j++)
		{
		 WinCoeff[j] = 1.0f
		 - 1.93293488969227f * cosf((float)(j+1) * M_2PIf / dM)
		 + 1.28349769674027f * cosf((float)(j+1) * M_2PIf * 2.0f / dM)
		 - 0.38130801681619f * cosf((float)(j+1) * M_2PIf * 3.0f / dM)
		 + 0.02929730258511f * cosf((float)(j+1) * M_2PIf * 4.0f / dM);
		}
	  }


	 else if(WindowType == wtBLACKMAN_HARRIS)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.35875f
		 - 0.48829f * cosf((float)(j+1) * M_2PIf / dM)
		 + 0.14128f * cosf((float)(j+1) * M_2PIf * 2.0f / dM)
		 - 0.01168f * cosf((float)(j+1) * M_2PIf * 3.0f / dM);
		}
	  }

	 else if(WindowType == wtBLACKMAN_NUTTALL)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.3535819f
		 - 0.4891775f * cosf((float)(j+1) * M_2PIf / dM)
		 + 0.1365995f * cosf((float)(j+1) * M_2PIf * 2.0f / dM)
		 - 0.0106411f * cosf((float)(j+1) * M_2PIf * 3.0f / dM);
		}
	  }

	 else if(WindowType == wtNUTTALL)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.355768f
		 - 0.487396f * cosf((float)(j+1) * M_2PIf / dM)
		 + 0.144232f * cosf((float)(j+1) * M_2PIf * 2.0f / dM)
		 - 0.012604f * cosf((float)(j+1) * M_2PIf * 3.0f / dM);
		}
	  }

	 else if(WindowType == wtKAISER_BESSEL)
	  {
	   for(j=0; j<=M/2; j++)
		{
		 WinCoeff[j] = 0.402f
		 - 0.498f * cosf(M_2PIf * (float)(j+1) / dM)
		 + 0.098f * cosf(2.0f * M_2PIf * (float)(j+1) / dM)
		 + 0.001f * cosf(3.0f * M_2PIf * (float)(j+1) / dM);
		}
	  }

	 else if(WindowType == wtTRAPEZOID) // Rectangle for Alpha = 1  Triangle for Alpha = 0
	  {
	   long K = M/2;
	   if(M%2)K++;
	   for(j=0; j<K; j++)WinCoeff[j] = (float)(j+1) / (float)K;
	  }


	 // This definition is from http://en.wikipedia.org/wiki/Window_function (Gauss Generalized normal window)
	 // We set their p = 2, and use Alpha in the numerator, instead of Sigma in the denominator, as most others do.
	 // Alpha = 2.718 puts the Gauss window response midway between the Hanning and the Flattop (basically what we want).
	 // It also gives the same BW as the Gauss window used in the HP 89410A Vector Signal Analyzer.
	 // Alpha = 1.8 puts it quite close to the Hanning.
	 else if(WindowType == wtGAUSS)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = ((float)(j+1) - dM/2.0f) / (dM/2.0f) * 2.7183f;
		 WinCoeff[j] *= WinCoeff[j];
		 WinCoeff[j] = expf(-WinCoeff[j]);
		}
	  }

	 else // Error.
	  {
	   console_log(PRIO_WARNING, "Incorrect window type in WindowFFTData");
	   free(WinCoeff);
	   return;
	  }

	 // Fold the coefficients over.
	 for(j=0; j<M/2; j++)WinCoeff[N-j-1] = WinCoeff[j];

	 // This is the flat top if Alpha > 0. Cannot be applied to a Kaiser or Flat Top.
	 if(WindowType != wtKAISER &&  WindowType != wtFLATTOP)
	  {
	   for(j=M/2; j<N-M/2; j++)WinCoeff[j] = 1.0f;
	  }

	 // This will set the gain of the window to 1. Only the Flattop window has unity gain by design. 
	 if(UnityGain)
	  {
	   float Sum = 0.0f;
	   for(j=0; j<N; j++)Sum += WinCoeff[j];
	   Sum /= (float)N;
	   if(Sum != 0.0f)for(j=0; j<N; j++)WinCoeff[j] /= Sum;
	  }

	 // Apply the window to the data.
	 for(j=0; j<N; j++)x[j] *= WinCoeff[j];
	 free(WinCoeff);

}



float* kaiserf(long N, bool twosided, float beta, bool unity_gain) {
	long j, M;
	float dM, * WinCoeff;
	float Arg;
	long Nmiddle;

	// if not twosided extend length as flank has to be longer to match signal length (not memory efficiently implemented)
	if (!twosided) {
		N = N * 2;
	}

	M = N;
	dM = (float)(M + 1);
	Nmiddle = (long)floorf((float)N / 2.0f);

	WinCoeff = (float*)malloc((N + 2) * sizeof(float)); // new(std::nothrow) float[N+2];

	for (j = 0; j < M; j++)
	{
		Arg = beta * sqrtf(1.0f - powf(((float)(2 * j + 2) - dM) / dM, 2.0f));
		WinCoeff[j] = bessel(Arg) / bessel(beta);
	}

	// Fold the coefficients over.
	for (j = 0; j < M / 2; j++) {
		WinCoeff[N - j - 1] = WinCoeff[j];
	}

	// This will set the gain of the window to 1.
	if (unity_gain)
	{
		float Sum = 0.0f;
		for (j = 0; j < N; j++) {
			Sum += WinCoeff[j];
		}
		Sum /= (float)N;
		if (Sum != 0.0f) {
			for (j = 0; j < N; j++)WinCoeff[j] /= Sum;
		}
	}
	return WinCoeff;
}

void kaiserf_apply(float* x, float* WinCoeff, long N, int direction, float stepsize, bool x_integrate) {
	long j, n;

	// Apply the window to the data.
	if (direction == -1) {
		for (j = 0; j < N; j++) {
			n = j;
			if (stepsize != 0.0f) {
				n = (long)roundf((float)n * stepsize);
			}
			if (x_integrate) {
				x[j] *= WinCoeff[n];
			}
			else {
				x[j] = WinCoeff[n];
			}
		}
	}
	if (direction == 0) {
		for (j = 0; j < N; j++) {
			n = j;
			if (stepsize != 0.0f) {
				n = (long)roundf((float)n * stepsize);
			}
			if (x_integrate) {
				x[j] *= WinCoeff[n];
			}
			else {
				x[j] = WinCoeff[n];
			}
		}
	}
	if (direction == 1) {
		for (j = 0; j < N; j++) {
			n = j + N;
			if (stepsize != 0.0f) {
				n = (long)roundf((float)j * stepsize) + N;
			}
			if (x_integrate) {
				x[j] *= WinCoeff[n];
			}
			else {
				x[j] = WinCoeff[n];
			}
		}
	}
}


// This gets used with the Kaiser window.
float bessel(float N)
{
	float Sum=0.0f, NtoIpower;
	long i, j, Factorial;
	for(i=1; i<10; i++)
	{
		NtoIpower = powf(N/2.0f, (float)i);
		Factorial = 1;
		for(j=1; j<=i; j++)Factorial *= j;
		Sum += powf(NtoIpower / (float)Factorial, 2.0f);
	}
	return(1.0f + Sum);
}

//-----------------------------------------------------------------------------

// This gets used with the Sinc window and various places in the basic_fir_double function.
float sinc(float x)
{
	 if(x > -1.0E-5f && x < 1.0E-5f)return(1.0f);
	 return(sinf(x)/x);
}


// double functions
////////////////////////////////////////////////////////////////////////////////////////////////


// This first calculates the impulse response for a rectangular window.
// It then applies the windowing function of choice to the impulse response.
void basic_fir_double(double *FirCoeff, int NumTaps, enum TPassTypeName PassType, double OmegaC, double BW, enum TWindowType WindowType, double WinBeta)
{
	 int j;
	 double Arg, OmegaLow, OmegaHigh;

	 switch(PassType)
	  {
	   case LPF:
		for(j=0; j<NumTaps; j++)
		 {
		  Arg = (double)j - (double)(NumTaps-1) / 2.0;
		  FirCoeff[j] = OmegaC * sinc_double(OmegaC * Arg * M_PI);
		 }
		break;

	   case HPF:
		if(NumTaps % 2 == 1) // Odd tap counts
		 {
		  for(j=0; j<NumTaps; j++)
		   {
			Arg = (double)j - (double)(NumTaps-1) / 2.0;
			FirCoeff[j] = sinc_double(Arg * M_PI) - OmegaC * sinc_double(OmegaC * Arg * M_PI);
		   }
		 }

		else  // Even tap counts
		  {
		   for(j=0; j<NumTaps; j++)
			{
			 Arg = (double)j - (double)(NumTaps-1) / 2.0;
			 if(Arg == 0.0)FirCoeff[j] = 0.0;
			 else FirCoeff[j] = cos(OmegaC * Arg * M_PI) / M_PI / Arg  + cos(Arg * M_PI);
			}
		  }
	   break;

	   case BPF:
		OmegaLow  = OmegaC - BW/2.0;
		OmegaHigh = OmegaC + BW/2.0;
		for(j=0; j<NumTaps; j++)
		 {
		  Arg = (double)j - (double)(NumTaps-1) / 2.0;
		  if(Arg == 0.0)FirCoeff[j] = 0.0;
		  else FirCoeff[j] =  ( cos(OmegaLow * Arg * M_PI) - cos(OmegaHigh * Arg * M_PI) ) / M_PI / Arg ;
		 }
	   break;

	   case NOTCH:  // If NumTaps is even for Notch filters, the response at Pi is attenuated.
		OmegaLow  = OmegaC - BW/2.0;
		OmegaHigh = OmegaC + BW/2.0;
		for(j=0; j<NumTaps; j++)
		 {
		  Arg = (double)j - (double)(NumTaps-1) / 2.0;
		  FirCoeff[j] =  sinc_double(Arg * M_PI) - OmegaHigh * sinc_double(OmegaHigh * Arg * M_PI) - OmegaLow * sinc_double(OmegaLow * Arg * M_PI);
		 }
	   break;
	  }

	 // window_data_double can be used to window data before an FFT. When used for FIR filters we set
	 // Alpha = 0.0 to prevent a flat top on the window and 
	 // set UnityGain = false to prevent the window gain from getting set to unity.
	 window_data_double(FirCoeff, NumTaps, WindowType, 0.0, WinBeta, false);

}

//---------------------------------------------------------------------------


// This gets used with the Kaiser window.
double bessel_double(double x)
{
	 double Sum=0.0, XtoIpower;
	 int i, j, Factorial;
	 for(i=1; i<10; i++)
	  {
	   XtoIpower = pow(x/2.0, (double)i);
	   Factorial = 1;
	   for(j=1; j<=i; j++)Factorial *= j;
	   Sum += pow(XtoIpower / (double)Factorial, 2.0);
	  }
	 return(1.0 + Sum);
}

//-----------------------------------------------------------------------------

// This gets used with the Sinc window and various places in the basic_fir_double function.
double sinc_double(double x)
{
	 if(x > -1.0E-5 && x < 1.0E-5)return(1.0);
	 return(sin(x)/x);
}

//---------------------------------------------------------------------------



void window_data_double(double *x, int N, enum TWindowType WindowType, double Alpha, double Beta, bool UnityGain)
{
	 if(WindowType == wtNONE) return;

	 int j, M, TopWidth;
	 double dM, *WinCoeff;

	 if(WindowType == wtKAISER ||  WindowType == wtFLATTOP )Alpha = 0.0;

	 if(Alpha < 0.0)Alpha = 0.0;
	 if(Alpha > 1.0)Alpha = 1.0;

	 if(Beta < 0.0)Beta = 0.0;
	 if(Beta > 10.0)Beta = 10.0;

	 WinCoeff  =  (double*)malloc((N + 2) * sizeof(double)); // new(std::nothrow) double[N+2];
	 if(WinCoeff == NULL)
	  {
	   console_log(PRIO_WARNING, "Failed to allocate memory in FFTFunctions::WindowFFTData() ");
	   return;
	  }

	 TopWidth = (int)( Alpha * (double)N );
	 if(TopWidth%2 != 0)TopWidth++;
	 if(TopWidth > N)TopWidth = N;
	 M = N - TopWidth;
	 dM = (double)(M + 1);


	 // Calculate the window for N/2 points, then fold the window over (at the bottom).
	 // TopWidth points will be set to 1.
	 if(WindowType == wtKAISER)
	  {
	   double Arg;
	   for(j=0; j<M; j++)
		{
		 Arg = Beta * sqrt(1.0 - pow( ((double)(2*j+2) - dM) / dM, 2.0) );
		 WinCoeff[j] = bessel_double(Arg) / bessel_double(Beta);
		}
	  }

	 else if(WindowType == wtSINC)  // Lanczos
	  {
	   for(j=0; j<M; j++)WinCoeff[j] = sinc_double((double)(2*j+1-M)/dM * M_PI );
	   for(j=0; j<M; j++)WinCoeff[j] = pow(WinCoeff[j], Beta);
	  }

	 else if(WindowType == wtSINE)  // Hanning if Beta = 2
	  {
	   for(j=0; j<M/2; j++)WinCoeff[j] = sin((double)(j+1) * M_PI / dM);
	   for(j=0; j<M/2; j++)WinCoeff[j] = pow(WinCoeff[j], Beta);
	  }

	 else if(WindowType == wtHANNING)
	  {
	   for(j=0; j<M/2; j++)WinCoeff[j] = 0.5 - 0.5 * cos((double)(j+1) * M_2PI / dM);
	  }

	 else if(WindowType == wtHAMMING)
	  {
	   for(j=0; j<M/2; j++)
	   WinCoeff[j] = 0.54 - 0.46 * cos((double)(j+1) * M_2PI / dM);
	  }

	 else if(WindowType == wtBLACKMAN)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.42
		 - 0.50 * cos((double)(j+1) * M_2PI / dM)
		 + 0.08 * cos((double)(j+1) * M_2PI * 2.0 / dM);
		}
	  }


	 // See: http://www.bth.se/fou/forskinfo.nsf/0/130c0940c5e7ffcdc1256f7f0065ac60/$file/ICOTA_2004_ttr_icl_mdh.pdf
	 else if(WindowType == wtFLATTOP)
	  {
	   for(j=0; j<=M/2; j++)
		{
		 WinCoeff[j] = 1.0
		 - 1.93293488969227 * cos((double)(j+1) * M_2PI / dM)
		 + 1.28349769674027 * cos((double)(j+1) * M_2PI * 2.0 / dM)
		 - 0.38130801681619 * cos((double)(j+1) * M_2PI * 3.0 / dM)
		 + 0.02929730258511 * cos((double)(j+1) * M_2PI * 4.0 / dM);
		}
	  }


	 else if(WindowType == wtBLACKMAN_HARRIS)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.35875
		 - 0.48829 * cos((double)(j+1) * M_2PI / dM)
		 + 0.14128 * cos((double)(j+1) * M_2PI * 2.0 / dM)
		 - 0.01168 * cos((double)(j+1) * M_2PI * 3.0 / dM);
		}
	  }

	 else if(WindowType == wtBLACKMAN_NUTTALL)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.3535819
		 - 0.4891775 * cos((double)(j+1) * M_2PI / dM)
		 + 0.1365995 * cos((double)(j+1) * M_2PI * 2.0 / dM)
		 - 0.0106411 * cos((double)(j+1) * M_2PI * 3.0 / dM);
		}
	  }

	 else if(WindowType == wtNUTTALL)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = 0.355768
		 - 0.487396 * cos((double)(j+1) * M_2PI / dM)
		 + 0.144232 * cos((double)(j+1) * M_2PI * 2.0 / dM)
		 - 0.012604 * cos((double)(j+1) * M_2PI * 3.0 / dM);
		}
	  }

	 else if(WindowType == wtKAISER_BESSEL)
	  {
	   for(j=0; j<=M/2; j++)
		{
		 WinCoeff[j] = 0.402
		 - 0.498 * cos(M_2PI * (double)(j+1) / dM)
		 + 0.098 * cos(2.0 * M_2PI * (double)(j+1) / dM)
		 + 0.001 * cos(3.0 * M_2PI * (double)(j+1) / dM);
		}
	  }

	 else if(WindowType == wtTRAPEZOID) // Rectangle for Alpha = 1  Triangle for Alpha = 0
	  {
	   int K = M/2;
	   if(M%2)K++;
	   for(j=0; j<K; j++)WinCoeff[j] = (double)(j+1) / (double)K;
	  }


	 // This definition is from http://en.wikipedia.org/wiki/Window_function (Gauss Generalized normal window)
	 // We set their p = 2, and use Alpha in the numerator, instead of Sigma in the denominator, as most others do.
	 // Alpha = 2.718 puts the Gauss window response midway between the Hanning and the Flattop (basically what we want).
	 // It also gives the same BW as the Gauss window used in the HP 89410A Vector Signal Analyzer.
	 // Alpha = 1.8 puts it quite close to the Hanning.
	 else if(WindowType == wtGAUSS)
	  {
	   for(j=0; j<M/2; j++)
		{
		 WinCoeff[j] = ((double)(j+1) - dM/2.0) / (dM/2.0) * 2.7183;
		 WinCoeff[j] *= WinCoeff[j];
		 WinCoeff[j] = exp(-WinCoeff[j]);
		}
	  }

	 else // Error.
	  {
	   console_log(PRIO_WARNING, "Incorrect window type in WindowFFTData");
	   free(WinCoeff);
	   return;
	  }

	 // Fold the coefficients over.
	 for(j=0; j<M/2; j++)WinCoeff[N-j-1] = WinCoeff[j];

	 // This is the flat top if Alpha > 0. Cannot be applied to a Kaiser or Flat Top.
	 if(WindowType != wtKAISER &&  WindowType != wtFLATTOP)
	  {
	   for(j=M/2; j<N-M/2; j++)WinCoeff[j] = 1.0;
	  }

	 // This will set the gain of the window to 1. Only the Flattop window has unity gain by design. 
	 if(UnityGain)
	  {
	   double Sum = 0.0;
	   for(j=0; j<N; j++)Sum += WinCoeff[j];
	   Sum /= (double)N;
	   if(Sum != 0.0)for(j=0; j<N; j++)WinCoeff[j] /= Sum;
	  }

	 // Apply the window to the data.
	 for(j=0; j<N; j++)x[j] *= WinCoeff[j];
	 free(WinCoeff);

}

