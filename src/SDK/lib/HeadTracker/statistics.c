// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "statistics.h"

// sample metrics
/////////////////////////////////////////

void sample_mean(float* x, long N, bool x_square, float *mean) {
	long n;
	float tmp;
	tmp = 0.0f;
	if (x_square) {
		for (n = 0; n < N; n++) {
			tmp += x[n] * x[n];
		}
	}
	else {
		for (n = 0; n < N; n++) {
			tmp += x[n];
		}
	}
	*mean = tmp / (float)N;
}

void sample_meani(int* x, int N, bool x_square, float *mean) {
	int n;
	float xf;
	float tmp;
	tmp = 0.0f;
	if (x_square) {
		for (n = 0; n < N; n++) {
			xf = (float)x[n];
			tmp += xf * xf;
		}
	}
	else {
		for (n = 0; n < N; n++) {
			xf = (float)x[n];
			tmp += xf;
		}
	}

	*mean = tmp / (float)N;
}

void sample_var(float* x, long N, float* mean, float* var) {

	float mean_square;
	sample_mean(x, N, true, &mean_square);
	*var = mean_square - *mean * *mean;

	// direct
	//float e;
	//long n;
	//*var = 0.0f;
 //   for (n = 0; n < N; n++)
 //   {
	//	e = x[n] - *mean;
 //       *var += e * e;
 //   }
 //   *var = *var / (float)N;


}

void sample_vari(int* x, int N, float* mean, float* var) {

	float mean_square;
	sample_meani(x, N, true, &mean_square);
	*var = mean_square - *mean * *mean;

	// direct
	//float e;
	//int n;
	//*var = 0.0f;
 //   for (n = 0; n < N; n++)
 //   {
	//	e = (float)x[n] - *mean;
 //       *var += e * e;
 //   }
 //   *var = *var / (float)N;
}

void sample_mean_var_circular(float* x, long N, float abs_ang_period, float *mean, float* var) {
	// background mean: https://en.wikipedia.org/wiki/Mean_of_circular_quantities
	// background var: http://www.fiserlab.org/manuals/procheck/manual/man_cv.html
	// both calculated together is faster
	long n;
	float x_tmp;
	float y_tmp;
	float tmp;
	float conv_bin_to_atan2;
	x_tmp = 0.0f;
	y_tmp = 0.0f;
	conv_bin_to_atan2 = PI2 / (abs_ang_period);

	// iteration
	for (n = 0; n < N; n++) {
		tmp = x[n] * conv_bin_to_atan2;
		x_tmp += sinf(tmp);
		y_tmp += cosf(tmp);
	}

	// mean
	tmp = atan2f(y_tmp / (float)N, x_tmp / (float)N); 
	tmp = tmp / conv_bin_to_atan2;
	modulusf(-tmp + abs_ang_period / 4.0f, (long)abs_ang_period, mean);

	// variance
	x_tmp = x_tmp * x_tmp;
	y_tmp = y_tmp * y_tmp;
	tmp = x_tmp + y_tmp;
	tmp = sqrtf(tmp);
	tmp /= (float)N;
	*var = 1 - tmp;
}


void sample_mean_var_circulari(int* x, int N, float abs_ang_period, float *mean, float *var) {
	// background mean: https://en.wikipedia.org/wiki/Mean_of_circular_quantities
	// background var: http://www.fiserlab.org/manuals/procheck/manual/man_cv.html
	// both calculated together is faster
	int n;
	float x_tmp;
	float y_tmp;
	float tmp;
	float conv_bin_to_atan2;
	x_tmp = 0.0f;
	y_tmp = 0.0f;
	conv_bin_to_atan2 = PI2 / (abs_ang_period);

	// iteration
	for (n = 0; n < N; n++) {
		tmp = (float)x[n] * conv_bin_to_atan2;
		x_tmp += sinf(tmp);
		y_tmp += cosf(tmp);
	}

	// mean
	tmp = atan2f(y_tmp, x_tmp); // scaling does not matter for atan2
	tmp = tmp / conv_bin_to_atan2;
	modulusf(-tmp + abs_ang_period / 4.0f, (long)abs_ang_period, mean);

	// variance
	x_tmp = x_tmp * x_tmp;
	y_tmp = y_tmp * y_tmp;
	tmp = x_tmp + y_tmp;
	tmp = sqrtf(tmp);
	*var = 1 - tmp / (float)N; // scaling says that maximal distance (radius) of walk (addition of angles) would be 1 if it ony occurs in one direction (one angle)
}


void sample_mean_var_moving_init(float x, float* x_buffer_p, long Nwindow) {
	ar_set(x_buffer_p, Nwindow, x); // set all past values to current value (constant observation x)
	x_buffer_p[Nwindow] = (float)(Nwindow - 1); // pointer to oldest value (start at last window point)
	x_buffer_p[Nwindow + 1] = x * Nwindow; // past sum of all values
	x_buffer_p[Nwindow + 2] = x * x * Nwindow; // past sum of all squared values
}

void sample_mean_var_moving(float x, float * x_buffer_p, long Nwindow, float* mean, float* var) {
	// background: https://www.dsprelated.com/showthread/comp.dsp/97276-1.php

	float x2;
	float x_p; 
	float x2_p;
	long Np; // oldest bin from buffer
	float x_sum;
	float x2_sum;
	float x_sum_p;
	float x2_sum_p;
	float Nwindowf;
	Nwindowf = (float)Nwindow;

	// load past values from buffer
	Np = (long)x_buffer_p[Nwindow]; // pointer to oldest value (start at 0)
	x_p = x_buffer_p[Np];
	x_sum_p = x_buffer_p[Nwindow + 1]; // past sum of all values
	x2_sum_p = x_buffer_p[Nwindow + 2]; // past sum of all squared values

	// init new sum element
	x2 = x * x;
	x2_p = x_p * x_p;

	// correct past sum
	x_sum = x_sum_p + x - x_p;
	x2_sum = x2_sum_p + x2 - x2_p;

	// update buffer values
	x_buffer_p[Np] = x;
	x_buffer_p[Nwindow + 1] = x_sum;
	x_buffer_p[Nwindow + 2] = x2_sum;

	// increment pointer to point to oldest x value in buffer
	Np = Np + 1;
	if (Np >= Nwindow) {
		Np = 0;
	}
	x_buffer_p[Nwindow] = (float)Np;

	// calculate (moving) mean
	*mean = x_sum / Nwindowf;

	// calculate (moving) variance
	// *var = (Nwindowf * x2_sum - (x_sum * x_sum)) / (Nwindowf * (Nwindowf - 1.0f));
	*var = x2_sum / Nwindowf - (x_sum / Nwindowf * x_sum / Nwindowf);

}

void sample_mean_var_moving_Npast(float x, float* x_buffer_p, long Nwindow,  long Npast, float* mean, float* var) {
	// init
	long n;
	float tmp;
	float tmp_square;
	long nwindow;
	long Np;
	tmp = 0.0f;
	tmp_square = 0.0f;
	Np = (long)x_buffer_p[Nwindow];
	nwindow = Np;

	// calculate mean and variance
	if (Npast != 0) {
		for (n = 0; n < Npast; n++) {
			nwindow -= 1;
			if (nwindow == -1) {
				nwindow = Nwindow - 1;
			}
			tmp += x_buffer_p[nwindow];
			tmp_square += tmp * tmp;

		}

		tmp /= (float)Npast;
		tmp_square /= (float)Npast;

		*mean = tmp;
		*var = tmp_square - tmp * tmp;
	}

	// update buffer values
	x_buffer_p[Np] = x;

	// increment pointer to point to oldest x value in buffer
	Np = Np + 1;
	if (Np >= Nwindow) {
		Np = 0;
	}
	x_buffer_p[Nwindow] = (float)Np;
}

void sample_mean_var_circular_moving_init(float ang, float* ang_buffer_p, long Nwindow, float abs_ang_period) {
	float conv_deg_to_rad;
	conv_deg_to_rad = PI2 / (abs_ang_period);
	ang = ang * conv_deg_to_rad;
	ar_set(ang_buffer_p, Nwindow, ang); // set all past values to current value (constant observation x)
	ang_buffer_p[Nwindow] = (float)(Nwindow - 1); // pointer to oldest value (start at last window point)
	ang_buffer_p[Nwindow + 1] = sinf(ang) * Nwindow; // past sum of all values
	ang_buffer_p[Nwindow + 2] = cosf(ang) * Nwindow; // past sum of all squared values
}

void sample_mean_var_circular_moving(float ang, float * ang_buffer_p, long Nwindow, float abs_ang_period, float* mean, float* var) {
	// background: https://www.dsprelated.com/showthread/comp.dsp/97276-1.php
	float ang_p;
	float x;
	float y;
	float x_p; 
	float y_p;
	long Np; // oldest bin from buffer
	float x_sum;
	float y_sum;
	float x_sum_p;
	float y_sum_p;
	float Nwindowf;
	float conv_deg_to_rad;
	float tmp;
	Nwindowf = (float)Nwindow;
	conv_deg_to_rad = PI2 / (abs_ang_period);

	// load past values from buffer
	Np = (long)ang_buffer_p[Nwindow]; // pointer to oldest value (start at 0)
	ang_p = ang_buffer_p[Np];
	x_sum_p = ang_buffer_p[Nwindow + 1]; // past sum of all values
	y_sum_p = ang_buffer_p[Nwindow + 2]; // past sum of all squared values


	// init new sum element
	ang = ang * conv_deg_to_rad;
	//ang_p = ang_p * conv_bin_to_atan2; // already included
	x = sinf(ang);
	y = cosf(ang);
	x_p = sinf(ang_p);
	y_p = cosf(ang_p);

	// correct past sum
	x_sum = x_sum_p + x - x_p;
	y_sum = y_sum_p + y - y_p;

	// update buffer values
	ang_buffer_p[Np] = ang;
	ang_buffer_p[Nwindow + 1] = x_sum;
	ang_buffer_p[Nwindow + 2] = y_sum;

	// increment pointer to point to oldest x value in buffer
	Np = Np + 1;
	if (Np >= Nwindow) {
		Np = 0;
	}
	ang_buffer_p[Nwindow] = (float)Np;

	// calculate (moving) circular mean
	tmp = atan2f(y_sum, x_sum); // p_norm scaling does not matter for atan2
	tmp = tmp / conv_deg_to_rad;
	modulusf(-tmp + abs_ang_period / 4.0f, (long)abs_ang_period, mean);


	// calculate (moving) circular variance
	x_sum = x_sum * x_sum;
	y_sum = y_sum * y_sum;
	tmp = x_sum + y_sum;
	tmp = sqrtf(tmp);
	*var = 1 - tmp / Nwindow;
}
	

void sample_sd_moving(float x, float* x_buffer_p, long Nwindow, float* sd) {
	float mean;
	sample_mean_var_moving(x, x_buffer_p, Nwindow, &mean, sd);
	*sd = sqrtf(*sd);
}

void sample_var_normalize(float* var, float x_range) {
	// background: https://stats.stackexchange.com/questions/350270/what-are-the-minimum-and-maximum-values-of-variance/350278
	// maximal value of variance is (x_up - x_low)^2 / 4
	*var = *var * 4.0f / (x_range * x_range); 
}


void sample_median(float *x, long x_N, long x_n_init, float* left, float* right, float* x_median) {
  // modified from: http://www.disnetwork.info/the-blog/median-value-selection-fixed
  const unsigned char HEAP_LEN = x_N / 2;
  float *p, median;
  unsigned char nLeft, nRight;

  // pick first value as median candidate
  p = x;
  median = *p++;
  nLeft = nRight = 0;

  // init headp buffer
  left[0] = x[x_n_init];
  right[0] = x[x_n_init];

  for (;;) {
    //dumpState(left, nLeft, median, right, nRight, p, 27 - (p-a));
    //assert(stateIsValid(left, nLeft, median, right, nRight));

    // get next value
    float val = *p++;

    // if value is smaller than median, append to left heap
    if (val <= median) {
      // move biggest value to the top of left heap
      unsigned char child = nLeft++, parent = (child - 1) / 2;
      while (child && val > left[parent]) {
        left[child] = left[parent];
        child = parent;
        parent = (parent - 1) / 2;
      }
      left[child] = val;

      // if left heap is full
      if (nLeft == HEAP_LEN) {
        //cout << "---" << endl;
        // for each remaining value
        for (unsigned char nVal = x_N-(p - x); nVal; --nVal) {
          //dumpState(left, nLeft, median, right, nRight, p, nVal);
          //assert(stateIsValid(left, nLeft, median, right, nRight));
          // get next value
          val = *p++;
          // discard values falling in other heap
          if (val >= median) {
            continue;
          }
          // if val is bigger than biggest in heap, val is new median
          if (val >= left[0]) {
            median = val;
            continue;
          }
          // biggest heap value becomes new median
          median = left[0];
          // insert val in heap
          parent = 0;
          child = 2;
          while (child < HEAP_LEN) {
            if (left[child-1] > left[child]) {
              child = child-1;
            }
            if (val >= left[child]) {
               break;
            }
            left[parent] = left[child];
            parent = child;
            child = (parent + 1) * 2;
          }
          left[parent] = val;
        }
		*x_median = median;
        return;
      }
    } else {
      // move smallest value to the top of right heap
      unsigned char child = nRight++, parent = (child - 1) / 2;
      while (child && val < right[parent]) {
        right[child] = right[parent];
        child = parent;
        parent = (parent - 1) / 2;
      }
      right[child] = val;

      // if right heap is full
      if (nRight == HEAP_LEN) {
        //cout << "---" << endl;
        // for each remaining value
        for (unsigned char nVal = x_N-(p - x); nVal; --nVal) {
          //dumpState(left, nLeft, median, right, nRight, p, nVal);
          //assert(stateIsValid(left, nLeft, median, right, nRight));
          // get next value
          val = *p++;
          // discard values falling in other heap
          if (val <= median) {
            continue;
          }
          // if val is smaller than smallest in heap, val is new median
          if (val <= right[0]) {
            median = val;
            continue;
          }
          // heap top value becomes new median
          median = right[0];
          // insert val in heap
          parent = 0;
          child = 2;
          while (child < HEAP_LEN) {
            if (right[child-1] < right[child]) {
              child = child-1;
            }
            if (val <= right[child]) {
              break;
            }
            right[parent] = right[child];
            parent = child;
            child = (parent + 1) * 2;
          }
          right[parent] = val;
        }
		*x_median = median;
		return;
      }
    }
  }
}

// expectation metrics
/////////////////////////////////////////


void get_probability_norm(float* p, float *p_norm, int N, bool drop_negativ_prob, bool normalize) {
	float sum;
	sum = 0.0f;
	if (drop_negativ_prob) {
		for (int n = 0; n < N; n++) {
			if (p[n] < 0.0f) {
				p[n] = 0.0f;
			}
			else {
				sum += p[n];
			}
		}
	}
	else {
		for (int n = 0; n < N; n++) {
			sum += p[n];
		}
	}
	if (normalize) {
		for (int n = 0; n < N; n++) {
			p[n] = p[n] / sum;
		}
	}
	*p_norm = sum;
}

void expectation_maximum(float* p, float p_norm, float* x, int N, float scale, float* max) {
	// declare
	UNUSED(p_norm);
	float max_amp;
	long max_n;

	// init

	//calculate
	ar_max(p, N, &max_n, &max_amp);
	if (x) {
		*max = x[max_n];
	}
	else {
		*max = max_n * scale / (float)N;
	}
}

void expectation_mean(float* p, float p_norm, float* x, int N, float scale, float* mean, bool square_x) {
	long n;
	float nf;
	float tmp;
	tmp = 0.0f;

	// if x value is known
	if (square_x) {
		if (x) {
			for (n = 0; n < N; n++) {
				if (p[n] > 0.0f) {
					tmp += p[n] * x[n] * x[n];
				}
			}
		}
		else {
			for (n = 0; n < N; n++) {
				if (p[n] > 0.0f) {
					nf = (float)n;
					tmp += p[n] * (nf * nf);
				}
			}
		}
	}
	else {
		if (x) {
			for (n = 0; n < N; n++) {
				if (p[n] > 0.0f) {
					tmp += p[n] * x[n];
				}
			}
		}
		else {
			for (n = 0; n < N; n++) {
				if (p[n] > 0.0f) {
					nf = (float)n;
					tmp += p[n] * nf;
				}
			}
		}
	}

	// normalize mean and scale
	if (p_norm != 1.0f && p_norm != 0.0f) {
		*mean = tmp / p_norm;
	}
	else {
		*mean = tmp;
	}
	if (!x && scale != 1.0f && square_x == false) {
		tmp = scale / (float)N;
		*mean = *mean * tmp;
	}
	if (!x && scale != 1.0f && square_x == true) {
		tmp = scale / (float)N;
		*mean = *mean * tmp * tmp;
	}
}



void expectation_var(float* p, float p_norm, float* x, int N, float scale, float* mean, float* var, bool var_normalize, bool standard_deviation) {
	float mean_sqare;
 	expectation_mean(p, p_norm, x, N, scale, &mean_sqare, true);
	*var = mean_sqare - *mean * *mean;
	if (var_normalize) {
		sample_var_normalize(var, scale);
	}
	if (standard_deviation) {
		*var = sqrtf(*var);
	}
}


void expectation_mean_var_circular(float* p, float p_norm, float* x, int N, float abs_ang_period, float scale, float* mean, float *var) {
	// background mean: https://en.wikipedia.org/wiki/Mean_of_circular_quantities
	// background var: http://www.fiserlab.org/manuals/procheck/manual/man_cv.html
	// both calculated together is faster
	UNUSED(scale);
	int n;
	float nf;
	int Nshift;
	float x_tmp;
	float y_tmp;
	float tmp;
	float pdf_norm;
	float conv_bin_to_atan2;
	x_tmp = 0.0f;
	y_tmp = 0.0f;
	pdf_norm = 0.0f;
	conv_bin_to_atan2 = PI2 / (abs_ang_period);

	// iteration
	// if x value is known
	if (x) {
		for (n = 0; n < N; n++) {
			if (p[n] > 0.0f) {
				tmp = x[n] * conv_bin_to_atan2;
				x_tmp += sinf(tmp) * p[n];
				y_tmp += cosf(tmp) * p[n];
			}
		}
	}
	else {
		// if unkown values can not be generated from dimensions linearly increasing [0...abs_ang_period[, create it instead from ]-abs_ang_period...0...abs_ang_period[ 
		if (!x && (int)abs_ang_period == ((N - 1) / 2)) {
			Nshift = (int)abs_ang_period;
			for (n = 0; n < N; n++) {
				if (p[n] > 0.0f) {
					nf = (float)(n - Nshift);
					tmp = nf * conv_bin_to_atan2;
					x_tmp += sinf(tmp) * p[n]; // mapping can be pre-initialized, except from p[n]
					y_tmp += cosf(tmp) * p[n]; // mapping can be pre-initialized, except from p[n]
				}
			}
		}
		else {
			for (n = 0; n < N; n++) {
				if (p[n] > 0.0f) {
					nf = (float)n;
					tmp = nf * conv_bin_to_atan2;
					x_tmp += sinf(tmp) * p[n]; // mapping can be pre-initialized, except from p[n]
					y_tmp += cosf(tmp) * p[n]; // mapping can be pre-initialized, except from p[n]
				}
			}
		}
	}

	// mean
	tmp = atan2f(y_tmp, x_tmp); // p_norm scaling does not matter for atan2
	tmp = tmp / conv_bin_to_atan2;
	modulusf(-tmp + abs_ang_period / 4.0f, (long)abs_ang_period, mean);

	// variance
	x_tmp = x_tmp * x_tmp;
	y_tmp = y_tmp * y_tmp;
	tmp = x_tmp + y_tmp;
	tmp = sqrtf(tmp);

	// normalize variance
	// *p_norm scaling says that maximal distance (radius) of walk (addition of angles) would be 1 if it ony occurs in one direction (one angle)
	if (p_norm != 1.0f && p_norm != 0.0f) {
		*var = 1 - tmp / p_norm;
	}
	else {
		*var = 1 - tmp;
	}
}


// shifted expectation metrics
/////////////////////////////////////////

void expectation_max_shift(float* p, float p_norm, float* x, int N, int Nshift, float scale, float* max) {
	expectation_maximum(p, p_norm, x, N, scale, max);
	if (Nshift != 0) {
		*max = *max - scale / (float)N * (float)Nshift;
	}
}

void expectation_mean_shift(float* p, float p_norm, float* x, int N, int Nshift, float scale, float* mean) {
	expectation_mean(p, p_norm, x, N, scale, mean, false);
	if (Nshift != 0) {
		*mean = *mean - scale / (float)N * (float)Nshift;
	}
}


void expectation_var_shift(float* p, float p_norm, float* x, int N, int Nshift, float scale, float* mean, float* var, bool var_normalize, bool standard_deviation) {
	float mean_backshifted;
	if (Nshift != 0) {
		mean_backshifted = *mean + scale / (float)N * (float)Nshift;
	}
	else {
		mean_backshifted = *mean;
	}
	expectation_var(p, p_norm, x, N, scale, &mean_backshifted, var, var_normalize, standard_deviation);
}





// probability distributions
/////////////////////////////////////////

void standard_distribution(int N, float mean, float sd, float* x)
{
	// modified from: https://rosettacode.org/wiki/Statistics/Normal_distribution#C.2B.2B
    int i;
    int M = N + N % 2;
 
    for (i = 0; i < M; i += 2)
    {
        float x_s,x_s2,rsq,f;
        do {
            x_s = 2.0f * (float)rand() / RAND_MAX - 1.0f;
            x_s2 = 2.0f * (float)rand() / RAND_MAX - 1.0f;
            rsq = x_s * x_s + x_s2 * x_s2;
        }while( rsq >= 1.0f || rsq == 0.0f );
        f = sqrtf( -2.0f * logf(rsq) / rsq );
        x[i]   = (x_s * f) * sd + mean;
		if (i < N - 1) {
			x[i + 1] = (x_s2 * f) * sd + mean;
		}
    }

}
 
void pdf_to_pdfcdf(float* pdf, int N_pdf) {
	int n;

	// cumulative sum
	for (n = 1; n < N_pdf; n++) {
		pdf[n] += pdf[n - 1];
	}

	// norm cumulative sum
	if (pdf[N_pdf - 1] != 1.0f) {
		for (n = 0; n < N_pdf; n++) {
			pdf[n] /= pdf[N_pdf - 1];
		}
	}
}

 
void pdf_to_cdf(float* pdf, int N_pdf, float* cdf) {
	int n;
	float e_high = 1.001f; // maximal allowed error in sum of cumulative probability distribution
	float e_low = 0.999f; // maximal allowed error in sum of cumulative probability distribution

	// cumulative sum
	cdf[0] = pdf[0];
	for (n = 1; n < N_pdf; n++) {
		cdf[n] = cdf[n - 1] + pdf[n];
	}

	// norm cumulative sum
	if (cdf[N_pdf - 1] > e_high || cdf[N_pdf - 1] < e_low) {
		for (n = 0; n < N_pdf; n++) {
			cdf[n] /= cdf[N_pdf - 1];
		}
	}
}

void sample_index_from_cdf(float* cdf, int N_cdf, int* sample_index) {
	float p;
	int n;

	// draw random uniform probability between 0 and 1
	p = (float)rand() / (float)(RAND_MAX);
	*sample_index = 0;
	// find sample corresponding to drawn probability
	for (n = 0; n < N_cdf; n++) {
		if (p <= cdf[n]) {
			*sample_index = n;
			break;
		}
	}
}


void pdf_to_cdf_struct(struct pdfcdf* x, int N_pdfcdf, bool sort) {
	int n;
	float e_high = 1.001f; // maximal allowed error in sum of cumulative probability distribution
	float e_low = 0.999f; // maximal allowed error in sum of cumulative probability distribution

	//sort pdf bins according to their values using quicksort
	if (sort) {
		qsort(x, 3, sizeof(x[0]), compare_cdfpdf_struct);
	}

	// cumulative sum
	for (n = 1; n < N_pdfcdf; n++) {
		x[n].weight = x[n - 1].weight;
	}

	// norm cumulative sum
	if (x[N_pdfcdf - 1].weight > e_high || x[N_pdfcdf - 1].weight < e_low) {
		for (n = 0; n < N_pdfcdf; n++) {
			x[n].weight /= x[N_pdfcdf - 1].weight;
		}
	}
}

int compare_cdfpdf_struct(const void *a, const void *b)
{
    struct pdfcdf *a1 = (struct pdfcdf *)a;
    struct pdfcdf *a2 = (struct pdfcdf *)b;
    if ((*a1).value > (*a2).value)
        return -1;
    else if ((*a1).value < (*a2).value)
        return 1;
    else
        return 0;
}


void sample_from_cdf_struct(struct pdfcdf* x, int N_cdf, float* sample) {
	float p;
	int n;

	// draw random uniform probability between 0 and 1
	p = (float)rand() / (float)(RAND_MAX);

	// find sample corresponding to drawn probability
	for (n = 0; n < N_cdf; n++) {
		if (p <= x[n].weight) {
			*sample = x[n].value;
			break;
		}
	}
}


