/**
  ******************************************************************************
  * @file    moving_average.h
  * @author  Mohammad Hussein Tavakoli Bina, Sepehr Hashtroudi.
  * @brief   This file contains function prototype of moving average filter.
  ******************************************************************************
  *MIT License
  *
  *Copyright (c) 2018 Mohammad Hussein Tavakoli Bina
  *
  *Permission is hereby granted, free of charge, to any person obtaining a copy
  *of this software and associated documentation files (the "Software"), to deal
  *in the Software without restriction, including without limitation the rights
  *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  *copies of the Software, and to permit persons to whom the Software is
  *furnished to do so, subject to the following conditions:
  *
  *The above copyright notice and this permission notice shall be included in all
  *copies or substantial portions of the Software.
  *
  *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  *SOFTWARE.
  */

#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Definitions ---------------------------------------------------------------*/
#define WindowLength_0 30
#define WindowLength_1 5
#define WindowLength_2 5

/* TypeDefs ------------------------------------------------------------------*/

typedef struct{

	uint32_t Sum;	/* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
	uint32_t History[WindowLength_0]; /*Array to store values of filter window*/
}FilterTypeDef_0;

typedef struct{

	uint32_t Sum;	/* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
	uint32_t History[WindowLength_1]; /*Array to store values of filter window*/
}FilterTypeDef_1;

typedef struct{

	uint32_t Sum;	/* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
	uint32_t History[WindowLength_2]; /*Array to store values of filter window*/
}FilterTypeDef_2;


/* Function prototypes -------------------------------------------------------*/
void Moving_Average_Init_0(FilterTypeDef_0* filter_struct);
uint32_t Moving_Average_Compute_0(uint32_t raw_data, FilterTypeDef_0* filter_struct);

void Moving_Average_Init_1(FilterTypeDef_1* filter_struct);
uint32_t Moving_Average_Compute_1(uint32_t raw_data, FilterTypeDef_1* filter_struct);

void Moving_Average_Init_2(FilterTypeDef_2* filter_struct);
uint32_t Moving_Average_Compute_2(uint32_t raw_data, FilterTypeDef_2* filter_struct);


#endif
