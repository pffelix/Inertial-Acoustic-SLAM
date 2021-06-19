// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "tools.h"

//
//void read_wav_ffmpeg(float * buf, long buf_N)
//{
//	// Create a 20 ms audio buffer (assuming Fs = 44.1 kHz)
//	//int16_t buf_int[40000] = { 0 }; // buffer
//	//int n;                // buffer index
//
//	// Open WAV file with FFmpeg and read raw samples via the pipe.
///*	FILE *pipein;
//	pipein = _popen("ffmpeg -i whistle.wav -f s16le -ac 1 -", "r");
//	fread(buf_int, 2, buf_N, pipein);
//	_pclose(pipein);*/
//
//	//// Print the sample values in the buffer to a CSV file
//	//FILE *csvfile;
//	//csvfile = fopen_s(&csvfile, "samples.csv", "w");
//	//for (int n = 0; n < buf_N; ++n) {
//	//	fprintf(csvfile, "%d\n", buf_int[n]);
//	//};
//	//fclose(csvfile);
//
//	//memcpy(buf_ptr, &buf, buf_N);
//};

unsigned long str_to_ulong(char* str)
{
    unsigned long mult = 1;
    unsigned long re = 0;
    long len = (long)strlen(str);
    for(int i = len -1 ; i >= 0 ; i--)
    {
        re = re + ((int)str[i] -48)*mult;
        mult = mult*10;
    }
    return re;
};

void char_concatenate(const char* str1, const char* str2, const char* delimiter, char* str) {
	strcpy(str, str1); //copy str1 into the new str
	strcat(str, delimiter); // add delimiter
	strcat(str, str2); // add str2
}

void char_concatenatemulti(const char** strs, int str_N, const char* delimiter, char* str) {
	strcpy(str, strs[0]); // copy str1 into the new str
	for (int i = 1; i < str_N; i++) {
		if (strncmp(strs[i], ".", 1) && strncmp(strs[i - 1], "", 1) && strncmp(&str[strlen(str) - 1], "/", 1)) {
			strcat(str, delimiter); // add delimiter
		}
		strcat(str, strs[i]); // add other strings
	}
}

float indb(float x_abs) {
	float x_db;
	x_db = log10f(x_abs) * 10;
	return x_db;
};


float inabs(float x_db) {
	float x_abs;
	x_abs = powf(10, (x_db / 10));
	return x_abs;
};

long bin_to_up(long bin_raw, int Nup){
    long bin_up = bin_raw * Nup;
    return bin_up;
};


long bin_to_cor(long bin_raw){
    long bin_cor = bin_raw * 2;
    return bin_cor;
};

float maximum(float x1, float x2){
    float y;
    if(x1 > x2){
        y = x1;
    }
    else{
        y = x2;
    };
	return y;
}



float minimum(float x1, float x2){
    float y;
    if(x1 < x2){
        y = x1;
    }
    else{
        y = x2;
    };
	return y;
}

long maximum_long(long x1, long x2){
    long y;
    if(x1 > x2){
        y = x1;
    }
    else{
        y = x2;
    };
	return y;
}

long minimum_long(long x1, long x2){
    long y;
    if(x1 < x2){
        y = x1;
    }
    else{
        y = x2;
    };
	return y;
}

int maximum_int(int x1, int x2){
    int y;
    if(x1 > x2){
        y = x1;
    }
    else{
        y = x2;
    };
	return y;
}

int minimum_int(int x1, int x2){
    int y;
    if(x1 < x2){
        y = x1;
    }
    else{
        y = x2;
    };
	return y;
}

float maximum_ptr(float** x, long x_N, int Nchannels) {
	float y = 0.0f;
	for (int m = 0; m < Nchannels; m++) {
		for (long n = 0; n < x_N; n++) {
			if (y < x[m][n]) {
				y = x[m][n];
			}
		}
	}
	return y;
}

float absmaximum_ptr(float** x, long x_N, int Nchannels) {
	float y = 0.0f;
	float xabs_n;
	for (int m = 0; m < Nchannels; m++) {
		for (long n = 0; n < x_N; n++) {
			xabs_n = myAbs(x[m][n]);
			if (y < xabs_n) {
				y = xabs_n;
			}
		}
	}
	return y;
}

float myAbs(float x)
{
  // copy and re-interpret as 32 bit integer
  int casted = *(int*) &x;
  // clear highest bit
  casted &= 0x7FFFFFFF;

  // re-interpret as float
  return *(float*)&casted;
}

long abs_long(long x) {
	long y = 0l;
	if (x < 0) {
		y = -x;
	};
	return y;
}

long msec_to_N(float msec, long fs) {
	long n = (long)(msec  * (float)fs / 1000.0f);
	return n;
};

float N_to_m(long N, long fs, float sos) {
	float m;
	m = (float)N / (float)fs * sos;
	return m;
}
float sec_to_cm(float sec, float sos) {
	float cm;
	cm = sec * sos * 100.0f;
	return cm;
}

float m_to_msec(float m, float sos) {
	float msec;
	msec = m / sos * 1000.0f;
	return msec;
}

void sec_to_m(float sec, float sos, float* m) {
	*m = sec * sos;
}



void m_to_sec(float m, float sos, float* sec) {
	*sec = m / sos;
}

float deg_to_rad(float deg) {
	float rad;
	rad = deg * DEG_TO_RAD;
	return rad;
}

float rad_to_deg(float rad) {
	float deg;
	deg = rad * RAD_TO_DEG;
	return deg;
}

void cpx_normalize (struct cpx* x, long N) {
	struct cpx max_x = cpx_max(x, N);
	if (max_x.real != 0.0f) {
		for (int n = 0; n < N; n++) {
			x[n].real = x[n].real / max_x.real;
		}
	}
	if (max_x.imag != 0.0f) {
		for (int n = 0; n < N; n++) {
			x[n].imag = x[n].imag / max_x.imag;
		}
	}

};

float cpx_abs(float* a_real, float* a_imag) {
	float c;
	c = sqrtf(*a_real * *a_real + *a_imag * *a_imag);
	return c;
}

void cpx_multiply(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag) {
	*c_real = *a_real * *b_real - *a_imag * *b_imag;
	*c_imag = *a_real * *b_imag + *a_imag * *b_real;
}

void cpx_divide(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag) {
	float denominator;
	float b_imag_conj;
	denominator = *b_real * *b_real + *b_imag * *b_imag;
	b_imag_conj = -*b_imag;
	cpx_multiply(a_real, a_imag, b_real, &b_imag_conj, c_real, c_imag);
	*c_real = *c_real / denominator;
	*c_imag = *c_imag / denominator;
}

void cpx_add(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag) {
	*c_real = *a_real + *b_real;
	*c_imag = *a_imag + *b_imag;
}

void cpx_substract(float* a_real, float* a_imag, float* b_real, float* b_imag, float* c_real, float* c_imag) {
	*c_real = *a_real - *b_real;
	*c_imag = *a_imag - *b_imag;
}


float cpx_conj(float* a_imag) {
	float a_imag_conj;
	a_imag_conj = -*a_imag;
	return a_imag_conj;
}

struct cpx cpx_max(struct cpx *x, long N) {
	struct cpx max_x;
	max_x.real = x[0].real;
	max_x.imag = x[0].imag;
	for (long n = 1; n < N ; n++) {
		if (x[n].real > max_x.real) {
			max_x.real = x[n].real;
		}
	}
	for (long n = 1; n < N ; n++) {
		if (x[n].imag > max_x.imag) {
			max_x.imag = x[n].imag;
		}
	}
	return max_x;
};



bool is_power_of_2(long x) {
	if (x <= 0) {
		return false;
	}
	else {
		return ((x & (x - 1)) == 0);
	}
}

unsigned long next_power_of_2(unsigned long n) { 
	bool shiftmethod = true;
	if (shiftmethod) {
		n--;
		n |= n >> 1;
		n |= n >> 2;
		n |= n >> 4;
		n |= n >> 8;
		n |= n >> 16;
		n++;
	}
	else {
		n = (long)pow(2, ceil(log(n)/log(2)));
	}
    return n; 
} 

long modulus(long a, long b){
	if (b == 0) {
		return a;
	}
	else {
		if (b < 0) { //you can check for b == 0 separately and do what you want
			return (-modulus(-a, -b));
		}
		long ret = a % b;
		if (ret < 0) {
			ret += b;
		}
		return ret;
	}
}

void modulusf(float a, long b, float * y){
	if (b != 0) {
		long ret_int;
		float ret_decimal;
		ret_int = modulus((long)floorf(a), b);
		ret_decimal = (a - floorf(a));
		*y = (float)ret_int + ret_decimal;
	}
}


long modulus_diff(long n, long N){
	long diff;
	diff = modulus(n, N);
	if (diff > N - diff) {
		diff = N - diff;
	}
	return diff;
}

void correct_limit(float* x, float limit_low, float limit_high) {
		if (*x < limit_low) {
			*x = limit_low;
		}
		if (*x > limit_high) {
			*x = limit_high;
		}
}

void correct_limit_circular(float* x, float limit_low, float limit_high) {
		if (*x < limit_low || *x > limit_high) {
			modulusf(*x, (long)limit_high, x);

			if (limit_low != 0.0f) {
				*x = *x + limit_low;
			}
		}
}

float acot_0_to_pi(float x) {
	float y;
	y = PI / 2.0f - atanf(x);

	return y;
}

float acot(float x) {
	float y;
	y = acot_0_to_pi(x);
	if (x < 0) {
		y = y - PI;
	}

	return y;
}


void ask_lock(bool* lock) {
	// if variable is locked
	if (*lock) {
		// wait until variable is unlocked
		while (*lock);
	}
	// lock variable
	*lock = true;
}

void close_lock(bool* lock) {
	// unlock variable
	*lock = false;
}