/*
 * filter.c
 *
 *  Created on: Nov 23, 2024
 *      Author: Alvin
 */
#include "filter.h"

void filter_init(lpfilter * filter, float alpha){
	filter_set_alpha(filter, alpha);

	filter->out = 0.0f;
}
void filter_set_alpha(lpfilter *filter, float alpha){
	// clamp the value of alpha between 0 and 1

	if (alpha > 1.0f){
		filter->alpha = 1.0f;
	}
	else if(alpha < 0.0f){
		filter->alpha = 0.0f;
	}
	else{
		filter->alpha = alpha;
	}
}
float filter_update(lpfilter *filter, float input){
	filter->out = filter->alpha * input + (1 - filter->alpha) * filter->out;

	return filter->out;
}


