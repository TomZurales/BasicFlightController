/*
 * utils.h
 *
 *  Created on: Jan 26, 2020
 *      Author: root
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stdint.h"

int16_t trim(int16_t in, int16_t min, int16_t max);

int16_t map(int16_t in, int16_t from_min, int16_t from_max, int16_t to_min, int16_t to_max);

#endif /* UTILS_H_ */
