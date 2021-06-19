// Copyright 2020, Felix Pfreundtner, All rights reserved.
// statistics.h: Statistics functions.


#pragma once
#include "params.h"

// sample metrics
/////////////////////////////////////////

// mean of 1d liner quantity in float array
// if x_square: square x
void sample_mean(float *x, long N, bool x_square, float *mean);

// fluctuation around mean of 1d liner quantity in float array
void sample_var(float* x, long N, float *mean, float* var);

// mean of 1d liner quantity in integer array
// if x_square: square x
void sample_meani(int *x, int N, bool x_square, float *mean);

// fluctuation around mean of 1d liner quantity in integer array
void sample_vari(int* x, int N, float *mean, float* var);

// mean and fluctuation around mean of 1d circular quantity in float array
// angles in x must be given in degrees or radian with interval limit [0...abs_ang_period[ or ]-abs_ang_period...0...abs_ang_period[
void sample_mean_var_circular(float* x, long N, float abs_ang_period, float* mean, float* var);

// mean and fluctuation around mean of 1d circular quantity in integer array
// angles in x must be given in degrees or radian with interval limit [0...abs_ang_period[ or ]-abs_ang_period...0...abs_ang_period[
void sample_mean_var_circulari(int* x, int N, float abs_ang_period,  float* mean, float* var);

// init moving variance or standard deviation calculation over a running window with first observation x to avoid jump in variance
// size of buffer array must be Nwindow + 3 to store additional temporary values  
void sample_mean_var_moving_init(float x, float* x_buffer_p, long Nwindow);

// calculate moving variance over a running window with size Nwindow, new observation x and past Nwindow observations stored in x_buffer_p 
// size of buffer array must be Nwindow + 3 to store additional temporary values  
void sample_mean_var_moving(float x, float* x_buffer_p, long Nwindow, float *mean, float *var);

// moving mean and variance buffer update using Npast number of observations x (slower as not running calculation)
void sample_mean_var_moving_Npast(float x, float* x_buffer_p, long Nwindow, long Npast, float* mean, float* var);

// init circular moving variance or standard deviation calculation over a running window with first observation ang (degree) to avoid jump in variance
// size of buffer array must be Nwindow + 3 to store additional temporary values (ang_buffer_p in radians)
void sample_mean_var_circular_moving_init(float ang, float* ang_buffer_p, long Nwindow, float abs_ang_period);

// calculate circular moving variance over a running window with size Nwindow, new observation ang (degree) and past Nwindow observations stored in ang_buffer_p 
// size of buffer array must be Nwindow + 3 to store additional temporary values (ang_buffer_p in radians) 
void sample_mean_var_circular_moving(float ang, float* ang_buffer_p, long Nwindow, float abs_ang_period, float* mean, float* var);

// calculate moving standard deviation over a running window with size Nwindow, new observation x and past Nwindow observations stored in x_buffer_p 
// size of buffer array must be Nwindow + 3 to store additional temporary values 
void sample_sd_moving(float x, float* x_buffer_p, long Nwindow, float *sd);

// normalize variance var for elements x with interval range x_range
void sample_var_normalize(float* var, float x_range);

// mean of 1d liner quantity in float array (buffer left and right, both ceil(N/2) values, initially median value is guessable by x_n_init position)
void sample_median(float* x, long x_N, long x_n_init, float* left, float* right, float* x_median);

// expectation metrics (of circular quantity x with probability p)
/////////////////////////////////////////

// get norm of probability array p and drop negative probabilities if wished
void get_probability_norm(float* p, float* p_norm, int N, bool drop_negativ_prob, bool normalize);

// peak value (maximum)
void expectation_maximum(float* p, float p_norm, float* x, int N, float scale, float* max);

// expectation value (mean)
// if x == NULL: bin number is considered as value
void expectation_mean(float* p, float p_norm, float* x, int N, float scale, float* mean, bool square_x);


// fluctuation around expectation value (variance)
// if x == NULL: bin number is considered as value
void expectation_var(float* p, float p_norm, float* x, int N, float scale, float* mean, float* var, bool var_normalize, bool standard_deviation);

// expectation value (mean) and fluctuation around expectation value (variance) of circular quantity
// 1d float array of probabilities pdf p with attached value x
// if pdf_normalize: pdf p is assumed as not integrating to 1, normalize pdf instead 
// angles in x must be given in degrees or radian with interval limit [ang_min...abs_ang_period]
// if x == NULL: bin number is considered as value
// If abs_ang_period == (N - 1 / 2) ->  x = ]-abs_ang_period...0...abs_ang_period[,  else: x = [0...N[  
void expectation_mean_var_circular(float* p, float p_norm, float* x, int N, float abs_ang_period, float scale, float* mean, float* var);


// shifted expectation metrics (of circular quantity x with 0 point shifted by Nshift and probability p)
/////////////////////////////////////////

// shifted peak value (maximum)
void expectation_max_shift(float* p, float p_norm, float* x, int N, int Nshift, float scale, float* max);

// shifted expectation value (mean)
void expectation_mean_shift(float* p, float p_norm, float* x, int N, int Nshift, float scale, float* mean);

// shifted fluctuation around expectation value (variance) of linear quantity
void expectation_var_shift(float* p, float p_norm, float* x, int N, int Nshift, float scale, float* mean, float* var, bool var_normalize, bool standard_deviation);



// probability distributions
/////////////////////////////////////////

// normal random numbers generator using Marsaglia algorithm
void standard_distribution(int N, float mean, float sd, float* x);

// convert probability density function weight to cumulative density function weight in same array
void pdf_to_pdfcdf(float* pdf, int N_pdf);

// convert probability density function weight to cumulative density function weight in other array
void pdf_to_cdf(float* pdf, int N_pdf, float* cdf);

// sample random index from given cumulative density function struct array
void sample_index_from_cdf(float* cdf, int N_cdf, int* sample_index);

// convert probability density function  struct array to cumulative density function struct array, if values are unsorted use quicksort
void pdf_to_cdf_struct(struct pdfcdf* x, int N_pdfcdf, bool sort);

// qsort() compare function for sorting probability or cumulative density function struct array
int compare_cdfpdf_struct(const void* a, const void* b);

// sample random number from given cumulative density function struct array
void sample_from_cdf_struct(struct pdfcdf* x, int N_cdf, float* sample);
