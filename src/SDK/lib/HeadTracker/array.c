// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "array.h"
#include <stdlib.h>

// array declarations (free() needed)
/////////////////////////////////////////////////



float * ar_declare(long N) {
	float *x = (float*)malloc(N * sizeof(float));
	return x;
};

double * ar_declare_double(long N) {
	double *x = (double*)malloc(N * sizeof(double));
	return x;
};


float * ar_zeros(long N) {
	float *x = ar_declare(N);
	//memset(x, 0, N * sizeof(float));
	for (long n = 0; n < N; n++) {
		x[n] = 0.0f;
	}
	return x;
};

double * ar_zeros_double(long N) {
	double *x = ar_declare_double(N);
	//memset(x, 0, N * sizeof(float));
	for (long n = 0; n < N; n++) {
		x[n] = 0.0;
	}
	return x;
};

void ar_set(float* x, long N, float value) {
	for (long n = 0; n < N; n++) {
		x[n] = value;
	}
}


float * ar_value(long N, float value) {
	float *x = ar_declare(N);
	for (long n = 0; n < N; n++) {
		x[n] = value;
	}
	return x;
};

float* ar_linear(float start, double incr, long N) {
	// double* x_long = (double*)malloc(N * sizeof(double));
	float* x = ar_declare(N);
	// x_long[0] = start;
	if (x) {
		x[0] = start;
		for (long n = 1; n < N; n++)
		{
			x[n] = start + (float)(incr * n);
			//x[n] = x_long[n];

		};
		//free(x_long);
	}
	return x;
};


float* ar_linearsquare(float start, double incr, long N) {
	float* x = ar_linear(start, incr, N);
	for (long n = 0; n < N; n++) {
		x[n] = x[n] * x[n];
	}
	return x;
}

float* ar_linearsquareroot(float start, double incr, long N) {
	float* x = ar_linear(start, incr, N);
	for (long n = 0; n < N; n++) {
		x[n] = sqrtf(x[n]);
	}
	return x;
}


float* ar_linear_fast(float start, double incr, long N) {
	double* x_long = (double*)malloc(N * sizeof(double));
	float* x = ar_declare(N);
	if (x_long && x) {
		x_long[0] = start;
		x[0] = start;
		for (long n = 1; n < N; n++)
		{
			x_long[n] = x_long[n-1] + incr;
			x[n] = (float)x_long[n];

		};
	}

	return x;
};

float* ar_linear_int(float start, int incr, long N) {
	// double* x_long = (double*)malloc(N * sizeof(double));
	float* x = ar_declare(N);
	// x_long[0] = start;
	if (x) {
		x[0] = start;
		for (long n = 1; n < N; n++)
		{
			x[n] = start + (float)(incr * n);
			//x[n] = x_long[n];

		};
		//free(x_long);
	}
	return x;
};

float* ar_linearsquare_int(float start, int incr, long N) {
	float* x = ar_linear_int(start, incr, N);
	for (long n = 0; n < N; n++) {
		x[n] = x[n] * x[n];
	}
	return x;
}


float* ar_linearsquareroot_int(float start, int incr, long N) {
	float* x = ar_linear_int(start, incr, N);
	for (long n = 0; n < N; n++) {
		x[n] = sqrtf(x[n]);
	}
	return x;
}


double* ar_linear_double(double start, double incr, long N) {
	// double* x_long = (double*)malloc(N * sizeof(double));
	double* x = (double*)malloc(N * sizeof(double));
	if (x) {
		// x_long[0] = start;
		x[0] = start;
		for (long n = 1; n < N; n++)
		{
			x[n] = (double)(start + incr * (double)n);
			//x[n] = x_long[n];

		};
		//free(x_long);
	}
	return x;
};


float** ar_declare2d(long M, long N) {
	float** x = (float**)malloc(M * sizeof(float*));
	if (x) {
		for (long m = 0; m < M; m++) {
			x[m] = (float*)malloc(N * sizeof(float));
		}
	}
	return x;
};

float** ar_zeros2d(long M, long N) {
	float** x = ar_declare2d(M, N);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] = 0.0f;
		}
	}
	return x;
};

float** ar_value2d(long M, long N, float value) {
	float** x = ar_declare2d(M, N);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] = value;
		}
	}
	return x;
};

float** ar_valuediag2d(long M, float value) {
	float** x = ar_zeros2d(M, M);
	for (long m = 0; m < M; m++) {
		x[m][m] = value;
	}
	return x;
};

void ar_set2d(float** x, long M, long N, float value) {
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] = value;
		}
	}
}

void ar_free2d(float** x, long M, long N) {
	UNUSED(N);
	for (long m = 0; m < M; m++) {
		free(x[m]);
	}
	free(x);
};

void ar_free2dl(long** x, long M, long N) {
	UNUSED(N);
	for (long m = 0; m < M; m++) {
		free(x[m]);
	}
	free(x);
};


void ar_free2d_char(const char** x, long M, long N) {
	UNUSED(N);
	for (long m = 0; m < M; m++) {
		free((char*)x[m]);
	}
	free((char**)x);
};

float*** ar_declare3d(long M, long N, long O) {
	float*** x = (float***)malloc(M * sizeof(float**));
	if (x) {
		for (long m = 0; m < M; m++) {
			x[m] = (float**)malloc(N * sizeof(float*));
			if (x[m]) {
				for (long n = 0; n < N; n++) {
					x[m][n] = (float*)malloc(O * sizeof(float));
				}
			}
		}
	}
	return x;
};

float*** ar_zeros3d(long M, long N, long O) {
	float*** x = ar_declare3d(M, N, O);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			for (long o = 0; o < O; o++) {
				x[m][n][o] = 0.0f;
			}
		}
	}
	return x;
};

float*** ar_value3d(long M, long N, long O, float value) {
	float*** x = ar_declare3d(M, N, O);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			for (long o = 0; o < O; o++) {
				x[m][n][o] = value;
			}
		}
	}
	return x;
};


void ar_free3d(float*** x, long M, long N, long O) {
	UNUSED(O);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			free(x[m][n]);
		}
	}
	free(x);
};

void ar_free3d_char(const char*** x, long M, long N, long O) {
	UNUSED(O);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			free((char*)x[m][n]);
		}
	}
	free((char***)x);
};


long * ar_declarel(long N) {
	long *x = (long*)malloc(N * sizeof(long));
	return x;
};

long** ar_declare2dl(long M, long N) {
	long** x = (long**)malloc(M * sizeof(long*));
	if (x) {
		for (long m = 0; m < M; m++) {
			x[m] = (long*)malloc(N * sizeof(long));
		}
	}
	return x;
};

long*** ar_declare3dl(long M, long N, long O) {
	long*** x = (long***)malloc(M * sizeof(long**));
	if (x) {
		for (long m = 0; m < M; m++) {
			x[m] = (long**)malloc(N * sizeof(long*));
			if (x[m]) {
				for (long n = 0; n < N; n++) {
					x[m][n] = (long*)malloc(O * sizeof(long));
				}
			}
		}
	}
	return x;
};

long**** ar_declare4dl(long M, long N, long O, long P) {
	long**** x = (long****)malloc(M * sizeof(long***));
	if (x) {
		for (long m = 0; m < M; m++) {
			x[m] = (long***)malloc(N * sizeof(long**));
			if (x[m]) {
				for (long n = 0; n < N; n++) {
					x[m][n] = (long**)malloc(O * sizeof(long*));
					if (x[m][n]) {
						for (long o = 0; o < O; o++) {
							x[m][n][o] = (long*)malloc(P * sizeof(long));
						}
					}
				}
			}
		}
	}
	return x;
};

long * ar_zerosl(long N) {
	long *x = ar_declarel(N);
	//memset(x, 0, N * sizeof(long));
	for (long n = 0; n < N; n++) {
		x[n] = 0l;
	}
	return x;
};


long** ar_zeros2dl(long M, long N) {
	long** x = ar_declare2dl(M, N);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] = 0l;
		}
	}
	return x;
};

long*** ar_zeros3dl(long M, long N, long O) {
	long*** x = ar_declare3dl(M, N, O);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			for (long o = 0; o < O; o++) {
				x[m][n][o] = 0l;
			}
		}
	}
	return x;
};

long**** ar_zeros4dl(long M, long N, long O, long P) {
	long**** x = ar_declare4dl(M, N, O, P);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			for (long o = 0; o < O; o++) {
				for (long p = 0; p < P; p++) {
					x[m][n][o][p] = 0l;
				}
			}
		}
	}
	return x;
};

void ar_setl(long* x, long N, long value) {
	for (long n = 0; n < N; n++) {
		x[n] = value;
	}
};

void ar_set2dl(long** x, long M, long N, long value) {
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] = value;
		}
	}
}

int* ar_declarei(int N) {
	int *x = (int*)malloc(N * sizeof(int));
	return x;
}

int* ar_valuei(int N, int value) {
	int *x = ar_declarei(N);
	for (int n = 0; n < N; n++) {
		x[n] = value;
	}
	return x;
}

int* ar_lineari(int start, int incr, int N) {
	// double* x_long = (double*)malloc(N * sizeof(double));
	int* x = ar_declarei(N);
	// x_long[0] = start;
	if (x) {
		x[0] = start;
		for (int n = 1; n < N; n++)
		{
			x[n] = start + incr * n;
			//x[n] = x_long[n];

		};
		//free(x_long);
	}
	return x;
};

int** ar_declare2di(int M, int N) {
	int** x = (int**)malloc(M * sizeof(int*));
	if (x) {
		for (int m = 0; m < M; m++) {
			x[m] = (int*)malloc(N * sizeof(int));
		}
	}
	return x;
};

void ar_seti(int* x, int N, int value) {
	for (int n = 0; n < N; n++) {
		x[n] = value;
	}
};

void ar_set2di(int** x, int M, int N, int value) {
	for (int m = 0; m < M; m++) {
		for (int n = 0; n < N; n++) {
			x[m][n] = value;
		}
	}
};

bool** ar_declare2db(int M, int N) {
	bool** x = (bool**)malloc(M * sizeof(bool*));
	if (x) {
		for (int m = 0; m < M; m++) {
			x[m] = (bool*)malloc(N * sizeof(bool));
		}
	}
	return x;
};

bool** ar_value2db(int M, int N, bool value) {
	bool** x = ar_declare2db(M, N);
	for (int m = 0; m < M; m++) {
		for (int n = 0; n < N; n++) {
			x[m][n] = value;
		}
	}
	return x;
};


void ar_set2db(bool** x, int M, int N, bool value) {
	for (int m = 0; m < M; m++) {
		for (int n = 0; n < N; n++) {
			x[m][n] = value;
		}
	}
}


// array end results (no free() needed)
/////////////////////////////////////////////////


long ar_maxn(float* x, long N) {
	float max_amp = x[0];
	long max_n = 0;
	for (long n = 1; n < N; n++) {
		if (x[n] > max_amp) {
			max_amp = x[n];
			max_n = n;
		}
	}
	return max_n;
};

float ar_max2d(float** x, long M, long N) {
	float max_amp = x[0][0];
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			if (x[m][n] > max_amp) {
				max_amp = x[m][n];
			}
		}
	}
	return max_amp;
}

float ar_max3d(float*** x, long M, long N, long O) {
	float max_amp = x[0][0][0];
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			for (long o = 0; o < O; o++) {
				if (x[m][n][o] > max_amp) {
					max_amp = x[m][n][o];
					//if (max_amp > 360.0f) {
						//int aa = 0;
					//}
				}
			}
		}
	}
	return max_amp;
}

long ar_absmaxn(float* x, long N) {
	float max_amp = fabsf(x[0]);
	float xabs_n;
	long max_N = 0;
	for (long n = 1; n < N; n++) {
		xabs_n = fabsf(x[n]);
		if (xabs_n > max_amp) {
			max_amp = xabs_n;
			max_N = n;
		}
	}
	return max_N;
};

int ar_maxi(int* x, int N) {
	int max_amp = x[0];
	for (long n = 1; n < N; n++) {
		if (x[n] > max_amp) {
			max_amp = x[n];
		}
	}
	return max_amp;
}


float ar_sum(float* x, long N) {
	float y = 0.0f;
	for (long n = 0; n < N; n++) {
		y += x[n];
	}
	return y;
};



float ar_sumsquared(float* x, long N) {
	float y = 0.0f;
	for (long n = 0; n < N; n++) {
		y += x[n] * x[n];
	}
	return y;
};



float ar_sumabs(float *x, long N) {
	float y = 0.0f;
	for (long n = 0; n < N; n++) {
		y += fabsf(x[n]);
	}
	return y;
};

bool * ar_declareb(int N) {
	bool *x = (bool*)malloc(N * sizeof(bool));
	return x;
};


bool * ar_valueb(int N, bool value) {
	bool *x = ar_declareb(N);
	for (int n = 0; n < N; n++) {
		x[n] = value;
	}
	return x;
};

void ar_setb(bool* x, int N, bool value) {
	for (int n = 0; n < N; n++) {
		x[n] = value;
	}
};

// array reshapes (no free() needed)
/////////////////////////////////////////////////

void ar_absmax(float* x, long N, long* max_n, float* max_amp) {
	*max_amp = fabsf(x[0]);
	float xabs_n;
	for (long n = 1; n < N; n++) {
		xabs_n = fabsf(x[n]);
		if (xabs_n > *max_amp) {
			*max_amp = xabs_n;
			*max_n = n;
		}
	}
}


void ar_sumabsaverage(float* x, long N, float* y) {
	*y = ar_sumabs(x, N) / (float)N;
};

void sample_mean_positive(float* x, long* N, float* y, long* Npositive) {
	long n;
	*Npositive = 0;
	*y = 0.0f;
	for (n = 0; n < *N; n++) {
		if (x[n] > 0.0f) {
			*y += x[n];
			*Npositive += 1;
		}
	}
	if (*Npositive > 0) {
		*y = *y / (float)*Npositive;
	}
}

void sample_mean_flipped(float* x, long* N, float* y) {
	long n;
	*y = 0.0f;
	for (n = 0; n < *N; n++) {
		if (x[n] > 0.0f) {
			*y += x[n];
		}
		else {
			*y -= x[n];
		}
	}
	*y = *y / (float)*N;
}

void ar_max(float* x, long N, long* max_n, float *max_amp) {
	*max_amp = x[0];
	*max_n = 0;
	for (long n = 1; n < N; n++) {
		if (x[n] > *max_amp) {
			*max_amp = x[n];
			*max_n = n;
		}
	}
};

void ar_Nmaxi(float* x, int N, int* max_n, float* max_amp, int Nmax) {
	int n;
	int i, j, k;
	ar_seti(max_n, Nmax, 0);
	ar_set(max_amp, Nmax, 0.0f);
	// go through all samples n
	for (n = 1; n < N; n++) {
		// go through all Nmax maxima i
		for (i = 0; i < Nmax; i++) {
			// if current sample bigger than one of the Nmax maxima i
			if (x[n] > max_amp[i]) {
				// shift all prior maxima after newly found maxima j, 1 to the right and delete lowest maxima
				for (j = Nmax - 1; j > i; j--) {
					k = j - 1;
					max_n[j] = max_n[k];
					max_amp[j] = max_amp[k];
				}
				// assign newly found maxima
				max_n[i] = n;
				max_amp[i] = x[n];
				// finish loop
				break;
			}
		}
	}
};

void ar_Nmax2di(float** x, int M, int N, int* max_m, int* max_n, float* max_amp, int Nmax) {
	int n, m;
	int i, j, k;
	ar_seti(max_m, Nmax, 0);
	ar_seti(max_n, Nmax, 0);
	ar_set(max_amp, Nmax, 0.0f);
	// go through all samples m
	for (m = 0; m < M; m++) {
		// go through all samples n
		for (n = 0; n < N; n++) {
			// go through all Nmax maxima i
			for (i = 0; i < Nmax; i++) {
				// if current sample bigger than one of the Nmax maxima i
				if (x[m][n] > max_amp[i]) {
					// shift all prior maxima after newly found maxima j, 1 to the right and delete lowest maxima
					for (j = Nmax - 1; j > i; j--) {
						k = j - 1;
						max_m[j] = max_m[k];
						max_n[j] = max_n[k];
						max_amp[j] = max_amp[k];
					}
					// assign newly found maxima
					max_m[i] = m;
					max_n[i] = n;
					max_amp[i] = x[m][n];
					// finish loop
					break;
				}
			}
		}
	}
};

void ar_min(float* x, long N, long* min_N, float *min_amp) {
	*min_amp = x[0];
	*min_N = 0;
	for (long n = 1; n < N; n++) {
		if (x[n] < *min_amp) {
			*min_amp = x[n];
			*min_N = n;
		}
	}
};

void ar_square(float* x, long N){
	for (long n = 0; n < N; n++) {
		x[n] = x[n] * x[n];
	}
}

void ar_multiply(float* x, long N, float value) {
	for (long n = 0; n < N; n++) {
		x[n] *= value;
	}
}
void ar_multiply2d(float** x, long M, long N, float value) {
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] *= value;
		}
	}
}

void ar_divide(float* x, long N, float value) {
	for (long n = 0; n < N; n++) {
		x[n] /= value;
	}
}

void ar_divide2d(float** x, long M, long N, float value) {
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			x[m][n] /= value;
		}
	}
}


void ar_norm(float *x, long N, float *y) {
	float x_max = x[ar_maxn(x, N)];
	for (long n = 0; n < N; n++) {
		y[n] = x[n] / x_max;
	}
};

void ar_smean(float* x1, float* x2, long N, float* y) {
	for (long n = 0; n < N; n++) {
		y[n] = (x1[n] + x2[n]) / 2.0f;
	}
}

void ar_mvalue(float* x, long N, float value, float *y) {
	for (long n = 0; n < N; n++) {
		y[n] = x[n] * value;
	}
}

// composed matrix operations: (no free() needed)
//////////////////////////////////////////////////

extern inline void ar_tp(float *x, long N, float *y) { //float * 
	//float *y = ar_declare(N);
	long N_mid = ((long)ceilf(((float)N) / 2));
	float x_tmp;
	for (long n = 0; n < N_mid; n++) {
		x_tmp = x[n];
		y[n] = x[N - 1 - n];
		y[N - 1 - n] = x_tmp;
	}
	//return y;
};

extern inline void ar_neg(float *x, long N, float *y) { //float * 
	//float *y = ar_declare(N);
	for (long n = 0; n < N; n++) {
		y[n] = -x[n];
	}
	//return y;
};


extern inline void ar_s(float * x1, float * x2, long N, float *y) { //float * 
	//float *y = ar_declare(N);
	for (long n = 0; n < N; n++) {
		y[n] = x1[n] + x2[n];
	}
	//return y;
};


extern inline void ar_tp2d(float **x, long M, long N, float **y) { //float ** 
	//float **y = ar_zeros2d(M, N);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			y[n][m] = x[m][n];
		}
	}
	//return y;
};

extern inline void ar_neg2d(float **x, long M, long N, float **y) { //float ** 
	//float **y = ar_zeros2d(M, N);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
			y[m][n] = -x[m][n];
		}
	}
	//return y;
};


extern inline void ar_m2d(float ** x1, float ** x2, long M1, long N1M2, long N2, float **y) { //float ** 
	//float **y = ar_zeros2d(M, N);
	for (long m = 0; m < M1; m++) {
		for (long n = 0; n < N2; n++) {
			y[m][n] = 0.0f;
			for (long k = 0; k < N1M2; k++) {
				y[m][n] += x1[m][k] * x2[k][n];
			}
		}
	}
	//return y;
};

extern inline void ar_s2d(float ** x1, float ** x2, long M, long N, float **y) { //float ** 
	//float **y = ar_zeros2d(M, N);
	for (long m = 0; m < M; m++) {
		for (long n = 0; n < N; n++) {
				y[m][n] = x1[m][n] + x2[m][n];
		}
	}
	//return y;
};

extern inline void ar_m2d1d(float ** x1, float * x2, long M, long N, float *y) { //float * 
	//float *y = ar_zeros(M);
	for (long m = 0; m < M; m++) {
		y[m] = 0.0f;
		for (long k = 0; k < N; k++) {
			y[m] += x1[m][k] * x2[k];
		}
	}
	//return y;
};

//float ** ar_m1d2d(float * v, int V, float ** x, long M, long N) {
//	int m, n, k;
//	float **y = ar_zeros2d(M, N);
//	for (m = 0; m < M; m++)
//	{
//		for (n = 0; n < N; n++)
//		{
//			for (k = 0; k < N; k++)
//				y[m][n] += x1[m][k] * x2[k][n];
//		}
//	}
//	return y;
//};


extern inline void ar_inv2d(float** x, long MN, float** y) {
	// Source: https://www.codewithc.com/c-program-gauss-jordon-method-find-inverse-matrix/

	float tmp;
	int m,n,k;
							
	for (m = 0; m < MN; m++) {
		for (n = 0; n < MN; n++) {
			if (m == n) {
				y[m][n] = 1.0f;
			}
			else {
				y[m][n] = 0.0f;
			}
		}
	}

	for(k=0;k<MN;k++)									 
	{														
		tmp=x[k][k];										
		for(n=0;n<MN;n++)								
		{
			x[k][n]/=tmp;									
			y[k][n]/=tmp;									

		}													
		for(m=0;m<MN;m++)									
		{
			tmp = x[m][k];									
			for(n=0;n<MN;n++)							
			{												
				if (m == k) {
					break;
				}
				x[m][n] -= x[k][n]*tmp;						
				y[m][n] -= y[k][n]*tmp;						
			}
		}
	}
}


/////////////////////////////////////////////////
extern inline void ar_tpM3(float * x, float *y) { //float * 
	long N = 3;
	ar_tp(x, N, y);
	//float *y = ar_tp(x, N, y);
	//return y;
};

extern inline void ar_negM3(float *x, float *y) { //float * 
	long N = 3;
	ar_neg(x, N, y);
	//float *y = ar_neg(x, N, y);
	//return y;
};


extern inline void ar_sM3(float * x1, float * x2, float *y) { //float * 
	long N = 3;
	ar_s(x1, x2, N, y);
	//float *y = ar_s(x1, x2, N);
	//return y;
};


extern inline void ar_tp2dM3(float ** x, float **y) { //float ** 
	long M = 3;
	long N = 3;
	ar_tp2d(x, M, N, y);
	//float **y = ar_tp2d(x, M, N);
	//return y;
};

extern inline void ar_neg2dM3(float **x, float **y) { //float ** 
	long M = 3;
	long N = 3;
	ar_neg2d(x, M, N, y);
	//float **y = ar_neg2d(x, M, N);
	//return y;
};

extern inline void ar_m2dM3(float ** x1, float ** x2, float **y) { //float ** 
	long M1 = 3;
	long N1M2 = 3;
	long N2 = 3;
	ar_m2d(x1, x2, M1, N1M2, N2, y);
	//float **y = ar_m2d(x1, x2, M, N);
	//return y;
};

extern inline void ar_s2dM3(float ** x1, float ** x2, float **y) { //float ** 
	long M = 3;
	long N = 3;
	ar_s2d(x1, x2, M, N, y);
	//float **y = ar_s2d(x1, x2, M, N);
	//return y;
};

extern inline void ar_m2d1dM3(float ** x1, float * x2, float *y) { //float * 
	long M = 3;
	long N = 3;
	ar_m2d1d(x1, x2, M, N, y);
	//float *y = ar_m2d1d(x1, x2, M, N);
	//return y;
};

extern inline void ar_inv2dM3(float ** x, float **y) { //float **
	// determinant
	float determinant;
	determinant = x[0][0] * (x[1][1] * x[2][2] - x[2][1] * x[1][2]) -
	x[0][1] * (x[1][0] * x[2][2] - x[1][2] * x[2][0]) +
	x[0][2] * (x[1][0] * x[2][1] - x[1][1] * x[2][0]);

	// inverse
	//float **y = ar_declare2d(M, N);
	y[0][0] = (x[1][1] * x[2][2] - x[2][1] * x[1][2]) / determinant;
	y[0][1] = (x[0][2] * x[2][1] - x[0][1] * x[2][2]) / determinant;
	y[0][2] = (x[0][1] * x[1][2] - x[0][2] * x[1][1]) / determinant;
	y[1][0] = (x[1][2] * x[2][0] - x[1][0] * x[2][2]) / determinant;
	y[1][1] = (x[0][0] * x[2][2] - x[0][2] * x[2][0]) / determinant;
	y[1][2] = (x[1][0] * x[0][2] - x[0][0] * x[1][2]) / determinant;
	y[2][0] = (x[1][0] * x[2][1] - x[2][0] * x[1][1]) / determinant;
	y[2][1] = (x[2][0] * x[0][1] - x[0][0] * x[2][1]) / determinant;
	y[2][2] = (x[0][0] * x[1][1] - x[1][0] * x[0][1]) / determinant;

	//return y;
};

