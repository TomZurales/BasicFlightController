/*
 * utils.c
 *
 *  Created on: Jan 4, 2020
 *      Author: tom
 */

#include "utils.h"
#include <stdint.h>

int16_t trim(int16_t in, int16_t min, int16_t max){
  if(in < min){
    return min;
  }
  if(in > max){
    return max;
  }
  return in;
}

int16_t map(int16_t in, int16_t from_min, int16_t from_max, int16_t to_min, int16_t to_max){
  return (in - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}
