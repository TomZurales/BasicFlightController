/*
 * utils.c
 *
 *  Created on: Jan 4, 2020
 *      Author: tom
 */

#include "utils.h"
#include <stdint.h>

float trim(float in, float min, float max){
  if(in < min){
    return min;
  }
  if(in > max){
    return max;
  }
  return in;
}

float map(float in, float from_min, float from_max, float to_min, float to_max){
  return (in - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}
