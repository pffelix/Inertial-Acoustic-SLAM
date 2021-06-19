// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "array_memory_unsafe.h"
#if (active == 1)
	// array declarations (free() needed)
	/////////////////////////////////////////////////

	float* ar_linear(float start, double incr, long N) {
		// double* x_long = (double*)malloc(N * sizeof(double));
		float* x = ar_declare(N);
		// x_long[0] = start;
		x[0] = start;
		for (long n= 1; n < N; n++)
		{
			x[n] = start + incr*n;
			//x[n] = x_long[n];

		};
		//free(x_long);
		return x;
	};

	float* ar_linear_fast(float start, double incr, long N) {
		double* x_long = (double*)malloc(N * sizeof(double));
		float* x = ar_declare(N);
		if (x_long && x) {
			x_long[0] = start;
			x[0] = start;
			for (long n= 1; n < N; n++)
			{
				x_long[n] = x_long[n-1] + incr;
				x[n] = x_long[n];

			};
		}

		return x;
	};


	float * ar_declare(long N) {
		float *x = (float*)malloc(N * sizeof(float));
		return x;
	};

	float * ar_zeros(long N) {
		float *x = ar_declare(N);
		for (long n= 0; n < N; n++) {
			x[n] = 0;
		}
		return x;
	};

	float * ar_value(long N, float value) {
		float *x = ar_zeros(N);
		for (long n= 0; n < N; n++) {
			x[n] = value;
		}
		return x;
	};

	void ar_free(float * array, long M) {
		free(array);
	};

	float ** ar_declare2d(long M, long N) {
		float **x = (float **)malloc(M * sizeof(float*));
		if (x) {
			for (long m= 0; m < M; m++) {
			x[m] = (float*)malloc(N * sizeof(float));
			}
		}
		return x;
	};



	float ** ar_zeros2d(long M, long N) {
		float **x = ar_declare2d(M, N);
		for (long m= 0; m < M; m++) {
			for (long n= 0; n < N; n++) {
				x[m][n] = 0.0;
			}
		}
		return x;
	};

	float ** ar_valuediag2d(long M, float value) {
		float **x = ar_zeros2d(M, M);
		for (long m= 0; m < M; m++) {
			x[m][m] = value;
		}
		return x;
	};



	void ar_free2d(float ** x, long M) {
		for (long m= 0; m < M; m++) {
			free(x[m]);
		}
		free(x);
	};

	// array end results (no free() needed)
	/////////////////////////////////////////////////


	long ar_maxn(float *x, long N) {
		float maximum_x = x[0];
		long maximum_n = 0;
		for (long n= 1; n < N ; n++) {
			if (x[n] > maximum_x) {
				maximum_x = x[n];
				maximum_n = n;
			}
		}
		return maximum_n;
	};



	float ar_sum(float *x, long N) {
		float y = 0.0f;
		for (long n= 0; n < N; n++) {
			y += x[n];
		}
		return y;
	};

	float ar_sumsquared(float *x, long N) {
		float y = 0.0f;
		for (long n= 0; n < N; n++) {
			y += x[n]*x[n];
		}
		return y;
	};

	float ar_sumabs(float *x, long N) {
		float y = 0.0f;
		for (long n= 0; n < N; n++) {
			y += fabsf(x[n]);
		}
		return y;
	};

	// array reshapes (free() needed)
	/////////////////////////////////////////////////
	void ar_norm(float *x, long N, float *y) {
		float x_max = x[ar_maxn(x, N)];
		for (long n = 0; n < N; n++) {
			y[n] = x[n] / x_max;
		}
	};

	float * ar_tp(float *x, long N) {
		float *y = ar_declare(N);
		long N_mid = ((long)ceilf(((float)N) / 2));
		float x_tmp;
		for (long n= 0; n < N_mid; n++) {
			x_tmp = x[n];
			y[n] = x[N - 1 - n];
			y[N - 1 - n] = x_tmp;
		}
		return y;
	};

	float * ar_neg(float *x, long N) {
		float *y = ar_declare(N);
		for (long n= 0; n < N; n++) {
			y[n] = -x[n];
		}
		return y;
	};

	float * ar_s(float * x1, float * x2, long N) {
		float *y = ar_declare(N);
		for (long n= 0; n < N; n++) {
			y[n] = x1[n] + x2[n];
		}
		return y;
	};

	float ** ar_tp2d(float **x, long M, long N) {
		float **y = ar_zeros2d(M, N);
		for (long m= 0; m < M; m++) {
			for (long n= 0; n < N; n++) {
				y[m][n] = x[n][m];
			}
		}
		return y;
	};

	float ** ar_neg2d(float **x, long M, long N) {
		float **y = ar_zeros2d(M, N);
		for (long m= 0; m < M; m++) {
			for (long n= 0; n < N; n++) {
				y[m][n] = -x[m][n];
			}
		}
		return y;
	};

	float ** ar_m2d(float ** x1, float ** x2, long M, long N) {
		float **y = ar_zeros2d(M, N);
		for (long m= 0; m < M; m++) {
			for (long n= 0; n < N; n++) {
				for (long k = 0; k < M; k++) {
					y[m][n] += x1[m][k] * x2[k][n];
				}
			}
		}
		return y;
	};

	float ** ar_s2d(float ** x1, float ** x2, long M, long N) {
		float **y = ar_zeros2d(M, N);
		for (long m= 0; m < M; m++) {
			for (long n= 0; n < N; n++) {
					y[m][n] += x1[m][n] * x2[m][n];
			}
		}
		return y;
	};

	float * ar_m2d1d(float ** x1, float * x2, long M, long N) {
		float *y = ar_zeros(M);
		for (long m= 0; m < M; m++) {
			for (long k = 0; k < N; k++) {
				y[m] += x1[m][k] * x2[k];
			}
		}
		return y;
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

	/////////////////////////////////////////////////
	float * ar_tpM3(float * x) {
		long N = 3;
		float *y = ar_tp(x, N);
		return y;
	};

	float * ar_negM3(float *x) {
		long N = 3;
		float *y = ar_neg(x, N);
		return y;
	};


	float * ar_sM3(float * x1, float * x2) {
		long N = 3;
		float *y = ar_s(x1, x2, N);
		return y;
	};


	float ** ar_tp2dM3(float ** x) {
		long M = 3;
		long N = 3;
		float **y = ar_tp2d(x, M, N);
		return y;
	};

	float ** ar_neg2dM3(float **x) {
		long M = 3;
		long N = 3;
		float **y = ar_neg2d(x, M, N);
		return y;
	};

	float ** ar_m2dM3(float ** x1, float ** x2) {
		long M = 3;
		long N = 3;
		float **y = ar_m2d(x1, x2, M, N);
		return y;
	};

	float ** ar_s2dM3(float ** x1, float ** x2) {
		long M = 3;
		long N = 3;
		float **y = ar_s2d(x1, x2, M, N);
		return y;
	};

	float * ar_m2d1dM3(float ** x1, float * x2) {
		long M = 3;
		long N = 3;
		float *y = ar_m2d1d(x1, x2, M, N);
		return y;
	};

	float ** ar_inv2dM3(float ** x) {
		// determinant
		float determinant;
		determinant = x[0][0] * (x[1][1] * x[2][2] - x[2][1] * x[1][2]) -
		x[0][1] * (x[1][0] * x[2][2] - x[1][2] * x[2][0]) +
		x[0][2] * (x[1][0] * x[2][1] - x[1][1] * x[2][0]);

		// inverse
		float **y = ar_declare2d(3, 3);
		y[0][0] = (x[1][1] * x[2][2] - x[2][1] * x[1][2]) / determinant;
		y[0][1] = (x[0][2] * x[2][1] - x[0][1] * x[2][2]) / determinant;
		y[0][2] = (x[0][1] * x[1][2] - x[0][2] * x[1][1]) / determinant;
		y[1][0] = (x[1][2] * x[2][0] - x[1][0] * x[2][2]) / determinant;
		y[1][1] = (x[0][0] * x[2][2] - x[0][2] * x[2][0]) / determinant;
		y[1][2] = (x[1][0] * x[0][2] - x[0][0] * x[1][2]) / determinant;
		y[2][0] = (x[1][0] * x[2][1] - x[2][0] * x[1][1]) / determinant;
		y[2][1] = (x[2][0] * x[0][1] - x[0][0] * x[2][1]) / determinant;
		y[2][2] = (x[0][0] * x[1][1] - x[1][0] * x[0][1]) / determinant;

		return y;
	};

#endif