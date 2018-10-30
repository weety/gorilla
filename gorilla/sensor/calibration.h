/*
 * File      : calibration.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-30     weety    first version.
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "light_matrix.h"

typedef struct
{
	double V[9];
	double D[9];
	double P[9][9];
	double R;
	
	double OFS[3];
	double GAIN[3];
	Mat    EigVec;
	Mat    RotM;
} Cali_Obj;

#endif

