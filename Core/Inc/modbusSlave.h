#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f1xx_hal.h"

extern uint8_t SLAVE_ID;

#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03

extern uint16_t Holding_Registers_Database[8];
extern uint16_t Input_Registers_Database[4];
extern uint8_t Coils_Database[2];
extern const uint8_t Inputs_Database[1];

uint8_t readHoldingRegs (void);
uint8_t readInputRegs (void);
uint8_t readCoils (void);
uint8_t readInputs (void);

uint8_t writeSingleReg (void);
uint8_t writeHoldingRegs (void);
uint8_t writeSingleCoil (void);
uint8_t writeMultiCoils (void);

void modbusException (uint8_t exceptioncode);

#endif /* INC_MODBUSSLAVE_H_ */
