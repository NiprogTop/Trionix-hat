#include "RionDCM260B.h"


#include <stdio.h>
#include <stdbool.h>

#define BUFFER_MAX_SIZE 14
#define UART_TIMEOUT 100

#define LENGTHOF(array) (sizeof(array) / sizeof(*array))

static uint8_t buffer[BUFFER_MAX_SIZE];

static uint8_t CalculateCheckSum(uint8_t *data, size_t length)
{
	uint8_t result = 0;

	while(length--)
		result += *data++;

	return result;
}

static float ConvertFloatValue(uint8_t *buffer)
{
	float wholePart      = (buffer[0] & 0x0F) * 100 + (buffer[1] >> 4) * 10 + (buffer[1] & 0x0F) * 1,
		  fractionalPart = (buffer[2] >> 4) * 10 + (buffer[2] & 0x0F) * 1;

	float result = wholePart + fractionalPart / 100;

	return (buffer[0] >> 4) ? -result : result;
}


// static void UartReceive(uint8_t *buffer, size_t size)
// {
// 	HAL_UART_Receive(DEVICE_PORT, buffer, size, UART_TIMEOUT);
// }

// static void UartTransmit(const uint8_t *buffer, size_t size)
// {
// 	HAL_UART_Transmit(DEVICE_PORT, buffer, size,UART_TIMEOUT);
// }

void initCompass(void)
{
	HAL_Delay(50);

	buffer[0] = DEVICE_INDENTIFICATION;
	buffer[1] = 0x04;
	buffer[2] = 0x00;
	buffer[3] = INIT_DEVICE;
	buffer[4] = CalculateCheckSum(&buffer[1], 3);

	UartTransmit(buffer, 5);
	UartReceive(buffer, 7);
}

bool readCompassData(RionDCM260BData *data)
{
	buffer[0] = DEVICE_INDENTIFICATION;
	buffer[1] = 0x04;
	buffer[2] = 0x00;
	buffer[3] = READ_DATA;
	buffer[4] = CalculateCheckSum(&buffer[1], 3);
	HAL_Delay(10);
	UartTransmit(buffer, 5);
	UartReceive(buffer, LENGTHOF(buffer));
	if( buffer[0] == 0x68 && (buffer[13] == CalculateCheckSum(&buffer[1], 12)) )
	{
		*data = (RionDCM260BData)
		{
		   .heading = ConvertFloatValue(&buffer[10]),
		   .roll    = ConvertFloatValue(&buffer[7]),
		   .pitch   = ConvertFloatValue(&buffer[4])
		};
		return true;
	}else{
		return false;
	}


}

void startCalibration(void)
{
	buffer[0] = DEVICE_INDENTIFICATION;
	buffer[1] = 0x04;
	buffer[2] = 0x00;
	buffer[3] = START_CALIBRATON;
	buffer[4] = CalculateCheckSum(&buffer[1], 3);
	UartTransmit(buffer, 5);

}
void stopCalibration(void)
{
	buffer[0] = DEVICE_INDENTIFICATION;
	buffer[1] = 0x04;
	buffer[2] = 0x00;
	buffer[3] = STOP_CALIBRATION;
	buffer[4] = CalculateCheckSum(&buffer[1], 3);
	UartTransmit(buffer, 5);
}

