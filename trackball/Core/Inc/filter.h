/*
 * filter.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Alvin
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

typedef struct{

	float alpha;

	float out;
}lpfilter;

void filter_init(lpfilter * filter, float alpha);
void filter_set_alpha(lpfilter *filter, float alpha);
float filter_update(lpfilter *filter, float input);

#endif /* INC_FILTER_H_ */
