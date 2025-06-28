#include "encoder.h"

ENCODER encoders[] = {
    {
        .Enable = ENABLED,
        .ID = 0,
        .Address = LINEAR_ENCODER_1_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    },
    {
        .Enable = ENABLED,
        .ID = 1,
        .Address = LINEAR_ENCODER_2_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    },
    {
        .Enable = ENABLED,
        .ID = 2,
        .Address = LINEAR_ENCODER_3_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    },
    {
        .Enable = ENABLED,
        .ID = 3,
        .Address = LINEAR_ENCODER_4_ADDRESS,
        .Transmit = 0,
        .Recieve = {0, 0, 0, 0},
		.R_Length = 2,
		.Fail = 0,
        .Distance = 0.0,         // Initialize Distance
        .Color = 0,
        .Reset = 0
    }
};

void I2C_Error_Handler(void) {
    // Disable the I2C peripheral
    __HAL_I2C_DISABLE(&hi2c1);

    // Reset the I2C peripheral by setting the corresponding bit in the RCC_APB1RSTR register
    __HAL_RCC_I2C1_FORCE_RESET();

    // Add a small delay to allow the reset process to complete
    HAL_Delay(1);

    // Clear the reset bit
    __HAL_RCC_I2C1_RELEASE_RESET();

    // Re-initialize the I2C peripheral using HAL functions
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);

    // Re-enable the I2C peripheral
    __HAL_I2C_ENABLE(&hi2c1);
}

HAL_StatusTypeDef i2c_transmit(uint8_t _address, uint8_t _data[], uint8_t _data_length)
{
	HAL_StatusTypeDef _status=0;
    uint8_t retry_count_tx = 0;
    uint8_t max_retries = 3;

    // Keep trying to transmit data until successful or max retries reached
    while ( (_status = HAL_I2C_Master_Transmit(&hi2c1, _address << 1 , _data, _data_length, 100)) != HAL_OK) {
        // Transmission error
        I2C_Error_Handler();
        retry_count_tx++;
        //osDelay(1);
        // Check if maximum retries reached
        if (retry_count_tx >= max_retries) {
            // Give up and break the loop
        	_status = HAL_ERROR;
            break;
        }
    }
    	return _status;

}


HAL_StatusTypeDef i2c_receive(uint8_t _address, uint8_t _data[], uint8_t _data_length)
{
	HAL_StatusTypeDef _status=0;
    uint8_t retry_count_rx = 0;
    uint8_t max_retries = 3;

    // Keep trying to receive data until successful or max retries reached
    while ( (_status = HAL_I2C_Master_Receive(&hi2c1, _address << 1 , _data, _data_length, 100)) != HAL_OK) {
        // Reception error
        I2C_Error_Handler();
        retry_count_rx++;
        //osDelay(1);
        // Check if maximum retries reached
        if (retry_count_rx >= max_retries) {
            // Give up and break the loop
        	_status = HAL_ERROR;
            break;
        }
    }
    	return _status;
}




void encoder_sync(ENCODER* encoders, size_t count)
{
	encoder_transmit(encoders,TOTAL_ENCODERS);
	encoder_receive(encoders,TOTAL_ENCODERS);

}

void encoder_transmit(ENCODER* encoders, size_t count)
{

    for (size_t i = 0; i < count; i++) {
    	//encoders[i].Fail=0;
        HAL_StatusTypeDef _status=0;
        // Handle Reset. Reset bit should be cleared after reset is sent
        if (encoders[i].Enable == ENABLED && encoders[i].Reset ==1)
        {
        	// Copy Reset Byte to Transmit
        	//encoders[i].Transmit = encoders[i].Reset;
        	encoders[i].Transmit = RESET_ENCODER;
            _status = i2c_transmit(encoders[i].Address,&encoders[i].Transmit,1);

            if (_status != HAL_OK )
            {
            	encoders[i].Fail =1;
            }
            else
            {
            	encoders[i].Fail =0;
            	// Clear Reset
            	encoders[i].Reset=0;
            }
        }
        // To handle color setting
        else if (encoders[i].Enable == ENABLED)
		{
        	// Copy Color Byte to Transmit
        	encoders[i].Transmit = encoders[i].Color;
            _status = i2c_transmit(encoders[i].Address,&encoders[i].Transmit,1);

            if (_status != HAL_OK )
            {
            	encoders[i].Fail =1;
            }
            else
            {
            	encoders[i].Fail =0;
            }
		}
        else
        {
        	encoders[i].Fail=0;
        }
    }

}

void encoder_receive(ENCODER* encoders, size_t count)
{
    for (size_t i = 0; i < count; i++) {

        HAL_StatusTypeDef _status=0;
        if (encoders[i].Enable == ENABLED)
        {
            _status = i2c_receive(encoders[i].Address,encoders[i].Recieve,encoders[i].R_Length);

            if (_status != HAL_OK )
            {
            	encoders[i].Fail=1;
            	encoders[i].Distance=0;
            }
            else
            {
            	encoders[i].Fail=0;

            	// Parse Received Data
            	int16_t encoder_distance = (encoders[i].Recieve[0] << 8) | encoders[i].Recieve[1];
            	encoders[i].Distance = (float)encoder_distance/100;

            }
        }
        else
        {
        	encoders[i].Fail=0;
        }
    }
}
