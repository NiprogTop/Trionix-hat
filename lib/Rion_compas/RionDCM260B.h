/*
 * RionDCM260B.h
 *
 *  Created on: Jun 7, 2024
 *      Author: max
 */

#ifndef RIONDCM260B_H_
#define RIONDCM260B_H_

#include <stdbool.h>

#define DEVICE_INDENTIFICATION 0x68


enum RionDCM260B_Command
{
	READ_DATA = 0x04,
	INIT_DEVICE = 0x07,
	START_CALIBRATON = 0x08,
	STOP_CALIBRATION  = 0x0A

};

enum RionDCM260B_ErrCode
{
	INIT_OK = 0x87,
	CALIBRATION_OK = 0x00,
	READ_DATA_OK = 0x84
};

enum RionDCM260B_State
{
	WORKING,
	CALIBRATION
};

typedef struct
{
	float heading;
	float roll;
	float pitch;
} RionDCM260BData;


void initCompass(void);
bool readCompassData(RionDCM260BData *data);
void startCalibration(void);
void stopCalibration(void);
#endif /* RIONDCM260B_H_ */
