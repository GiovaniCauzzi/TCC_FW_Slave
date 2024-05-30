#ifndef INCLUDES_H
#define INCLUDES_H

#include "codec.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define dPI 3.14159265f

#define dBUFFER_I2S_SIZE 16
#define dBUFFER_ADC_SIZE 20//16
// #define INT16_TO_FLOAT 1.0f / 32768.0f
// #define FLOAT_TO_INT16 32768.0F
#define INT16_TO_FLOAT 1.0f / 1000.0f
#define FLOAT_TO_INT16 1000.0F

#define dFS 24000
#define dTS 0.000042408F



int GL_timer_100us = 0;
int GL_timer_1ms = 0;
int GL_timer_10ms = 0;
int GL_timer_100ms = 0;
int GL_timer_1s = 0;
uint8_t GL_timer_48khz = 0;







#endif /* INCLUDES_H */
