/*
 * utils.h
 *
 *  Created on: Jan 26, 2020
 *      Author: root
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stdint.h"

float trim(float in, float min, float max);

float map(float in, float from_min, float from_max, float to_min, float to_max);

#endif /* UTILS_H_ */
