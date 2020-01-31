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

float shift(float* arr, int arrLen){
    float ret = arr[arrLen - 1];
    for(int i = arrLen - 2; i >= 0; i--){
        arr[i + 1] = arr[i];
    }
    return ret;
}

float average(float* arr, uint8_t arrLen){
    float sum = 0;
    for(int i = 0; i < arrLen; i++){
        sum += arr[i];
    }
    return sum / arrLen;
}
