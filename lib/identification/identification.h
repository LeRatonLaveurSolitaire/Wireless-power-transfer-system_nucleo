#ifdef __cplusplus
extern "C" {
#endif

#ifndef IDENTIFICATION
#define IDENTIFICATION

#include "kiss_fft.h"
#include <math.h>
#include <stdint.h>

// Strucutre of the data before analysis

typedef struct {
  const float *data;
  size_t num_rows;
  size_t num_columns;
} MatFileData;


typedef struct {
  kiss_fft_scalar *clean_current;
  kiss_fft_scalar *noisy_current;
  kiss_fft_scalar *noisy_voltage;
  uint32_t data_len;
} System_Data;

typedef struct {
  kiss_fft_cpx *cc; // Clean current FFT
  kiss_fft_cpx *nc; // Noisy current FFT
  kiss_fft_cpx *nv; // Noisy voltage FFT
  float_t *freqs_list;
  uint32_t data_len;
} Fft_Data;

typedef struct {
  float_t amplitude;
  float_t angle;
} Impedance_data;

typedef struct {
  Impedance_data *data; // Clean current FFT
  float_t *freqs_list;
  uint32_t data_len;
} Impedance;

// define basic complex arithmetics

kiss_fft_cpx add_cpx(kiss_fft_cpx, kiss_fft_cpx);
kiss_fft_cpx sub_cpx(kiss_fft_cpx, kiss_fft_cpx);
kiss_fft_cpx mul_cpx(kiss_fft_cpx, kiss_fft_cpx);
kiss_fft_cpx div_cpx(kiss_fft_cpx, kiss_fft_cpx);
float_t mag_cpx(kiss_fft_cpx);
float_t ang_cpx(kiss_fft_cpx);

// Data parser function

System_Data data_parser(const MatFileData);

Fft_Data compute_fft(const System_Data, const float);

Impedance compute_impedance(const Fft_Data);

Impedance smoothing_filter(const Impedance,
                           const float_t);

void create_input_tensor(const Impedance, const float, const float, const float, float*);

#endif

#ifdef __cplusplus
}
#endif