#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stddef.h> // For size_t
#include "i2c.h"

/**************************************************
 *               I2C Addressing
 **************************************************/
#define LINEAR_ENCODER_1_ADDRESS  		0x8
#define LINEAR_ENCODER_2_ADDRESS  		0x9
#define LINEAR_ENCODER_3_ADDRESS  		0xC
#define LINEAR_ENCODER_4_ADDRESS  		0xD

/**************************************************
 *               Indicator Colors
 **************************************************/
#define RED    'D'
#define GREEN  'G'
#define BLUE   'B'
#define OFF    'O'
#define RESET_ENCODER 'R'

#define ENABLED 1
#define DISABLE 0

#define TOTAL_ENCODERS 4

typedef struct {
	uint8_t Enable;
	uint8_t ID;
	uint8_t Address;
	uint8_t Transmit;
	uint8_t Recieve[4];
	uint8_t R_Length;
	uint8_t Fail;
	float  Distance;
	uint8_t Color;
	uint8_t Reset;

} ENCODER;

extern ENCODER encoders[TOTAL_ENCODERS];

HAL_StatusTypeDef i2c_transmit(uint8_t _address, uint8_t _data[], uint8_t _data_length);
HAL_StatusTypeDef i2c_receive(uint8_t _address, uint8_t _data[], uint8_t _data_length);
void encoder_sync(ENCODER* encoders, size_t count);
void encoder_transmit(ENCODER* encoders, size_t count);
void encoder_receive(ENCODER* encoders, size_t count);

#endif /* INC_ENCODER_H_ */
