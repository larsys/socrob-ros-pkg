/***********************************************************************
 *                                                                     *
 * This file contains the main entry point of the software for the     *
 * 5DOF test board v3.                                                 *
 *                                                                     *
 ***********************************************************************
 *                                                                     * 
 *    Author:         (c) Tom Pycke <tom@pycke.be>                     *
 *    Filename:       main.c                                           *
 *    Date:           13/10/2007                                       *
 *    File Version:   1.00                                             *
 *    Other Files Required: microcontroller.h                          *
 *                          uart1.h                                    *
 *                          gyro.h                                     *
 *                          accelero.h                                 *
 *                          timer.h                                    *
 *                          ars.h                                      *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 * Other Comments:                                                     *
 *                                                                     *
 ***********************************************************************/


#include <stdio.h>

#include "HAL/microcontroller.h"
#include "HAL/adc.h"
#include "HAL/uart1.h"
#include "HAL/gyro.h"
#include "HAL/accelero.h"
#include "HAL/timer.h"
#include "ars.h"

#include <math.h>


void PrintInteger(unsigned int x, char *buffer);
void PrintSignedInteger(int x, void (*printer)(char[]));

/*
 *   Used for connecting to my stabilization module using UART
 */
void PrintForStabilizer(float roll)
{
	int i;
	uart1_puts("b");
	for (i = 0; i < 60; i+=2) i--;
	PrintSignedInteger((int)roll, &uart1_puts);	
	for (i = 0; i < 60; i+=2) i--;
	uart1_puts("e");
}

/*
 *   Used for displaying an artificial horizon on PC (VB app)
 */
void PrintForVbApp(float roll, float acc_roll)
{
	PrintSignedInteger((int)roll, &uart1_puts); // Send result to uart
	uart1_puts(";");
	PrintSignedInteger((int)(acc_roll/3.14*180.0), &uart1_puts);  // Send accelerometer angle to uart
	uart1_puts("\n\r");
}

/*
 *   For debugging
 */
void PrintAll(float a_x, float a_y, float a_z, int p, int q, int roll)
{
	printf(" %5f %5f %5f %5d %5d %5d\n\r", a_x, a_y, a_z, p, q, roll);
}



float PredictAccG_roll(float a_z, float a_y, float a_x)
{
	return -(atan2(-a_z, a_y)-(3.14159/2.0));
}


int main()
{
	char buffer[20];
	int i;
	float tmp, roll_angle;
	float acc_gyro = 0, dt;
	struct Gyro1DKalman filter_roll;
	struct Gyro1DKalman filter_pitch;
	
	init_microcontroller();

	adc_init();
	adc_start();

	timer_init_ms();

	uart1_open();
	uart1_puts("Device initialized\n\r");
		
	uart1_puts("Entering main loop\n\r");
	
	init_Gyro1DKalman(&filter_roll, 0.0001, 0.0003, 0.69);
	
	while(1)
	{
		dt = timer_dt();   // time passed in s since last call

		// execute kalman filter
		tmp = PredictAccG_roll(accelero_z(), accelero_y(), accelero_x());
		ars_predict(&filter_roll, gyro_roll_rad(), dt);    // Kalman predict
	    roll_angle = ars_update(&filter_roll, tmp);        // Kalman update + result (angle)
	    
		//PrintAll(accelero_x(), accelero_y(), accelero_z(), gyro_roll_rad(), gyro_pitch_rad(), r);
	    PrintForVbApp(roll_angle / 3.14*180.0, tmp); 
		//PrintForStabilizer(r);
	}
	
	uart1_close();
		
	return 0;
}



void PrintInteger(unsigned int x, char *buffer)
{
	static char emptyChar = ' ', tmpChar;
	unsigned int radix,
	    i;
	emptyChar = ' ';
	
	// auto-align on 16bit size (655.. = 5 positions)
	for (radix=10000, i=0; i < 5; radix/=10, i++)
	{
		if (x > radix-1) {
			tmpChar =  '0' + (x/radix);
			buffer[i] = tmpChar;
			x = x % radix;
			emptyChar = '0';
		} else
			buffer[i] = emptyChar;	
	}
	if (emptyChar == ' ')
		buffer[4] = '0';
	buffer[5] = '\0';
}


void PrintSignedInteger(int x, void (*printer)(char[]))
{
	static char tmpChar[2] = "0";
	unsigned int radix,
                 i, converted;
	char emptyChar = ' ';
	
	if (x < 0)
	{
		printer("-");
		x*=-1;
	}
	else if (x == 0)
	{
		printer("0");
		return;	
	}
	converted = (unsigned int) x;
	// auto-align on 16bit size (655.. = 5 positions)
	for (radix=10000, i=0; i < 5; radix/=10, i++)
	{
		if (converted > radix-1) {
			tmpChar[0] =  '0' + (converted/radix);
			printer(tmpChar);
			converted = converted % radix;
			emptyChar = '0';
		} else if (emptyChar == '0') {
			printer("0");
		}
	}
}

