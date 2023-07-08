/**
 *
 * @brief 	Library for Analog Devices DDS AD9834
 *
 */

#include "stm32l4xx_hal.h"

#ifndef AD9834_LIB
#define AD9834_LIB

#define SPI_TIMEOUT 	8000
#define FREQ_OFFSET		5.36871		// = 2^28 / F_MCKL = 2^28 / 50Meg
#define PHASE_OFFSET	11.37778	// = 2^12 / 360
#define MAX_FREQUENCY	17000000
#define FREQ_REG0		0
#define FREQ_REG1		1
#define PHASE_REG0		0
#define PHASE_REG1		1


/*
 * @brief: Structure to hold connection information for interfacing with AD9834
 *
 * 		   	Control Register Bits Description
 * 	MSB	   	DB15	Control Bit
 * 		 	DB14	Control Bit
 * 		 	DB13	Two write operation enable, DB13=1 allows a complete word to be loaded freq registers
 * 		 	DB12	Sets MSB or LSB for one work register transfer
 * 		 	DB11	Select frequency register
 * 		 	DB10	Select phase register
 * 		 	DB9		Selects PIN or SW for freq/phase registers, reset and sleep
 * 		 	DB8		Reset
 * 		 	DB7		Sleep1
 * 		 	DB6		Sleep12
 * 		 	DB5		Enable/Disable SIGN BIT OUT pine
 * 		 	DB4		SIGN/PIB
 * 		 	DB3		DIV2
 * 		 	DB2		Reserved. ALways 0
 * 		 	DB1		Set DAC mode. MODE=1 sets output to triangle and MODE=0 sets output to sine
 * 	LSB	 	DB0		Reserved. ALways 0
 *
 *
 * */
typedef struct  {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* fsync_gpio;
	GPIO_TypeDef* sleep_gpio;
	GPIO_TypeDef* reset_gpio;
	GPIO_TypeDef* phase_select_gpio;
	GPIO_TypeDef* freq_select_gpio;
	uint16_t fsync_pin;
	uint16_t sleep_pin;
	uint16_t reset_pin;
	uint16_t phase_select_pin;
	uint16_t freq_select_pin;
	uint16_t control_reg;
} AD9834_Connection;

void AD9834_set_square_output(AD9834_Connection* connection);
void AD9834_set_sinusoidal_output(AD9834_Connection* connection);
void AD9834_set_triangle_output(AD9834_Connection* connection);
void AD9834_set_phase(AD9834_Connection* connection, uint16_t phase, uint8_t reg);
void AD9834_set_frequency(AD9834_Connection* connection, uint32_t frequency, uint8_t reg);
void AD9834_set_frequency_lsb(AD9834_Connection* connection, uint32_t frequency, uint8_t reg);
void AD9834_set_frequency_msb(AD9834_Connection* connection, uint32_t frequency, uint8_t reg);
void AD9834_set_control_reg(AD9834_Connection* connection);
void AD9834_reset_pin_disable(AD9834_Connection* connection);
void AD9834_reset_pin_enable(AD9834_Connection* connection);
void AD9834_reset_bit_disable(AD9834_Connection* connection);
void AD9834_reset_bit_enable(AD9834_Connection* connection);
void AD9834_sleep_pin_disable(AD9834_Connection* connection);
void AD9834_sleep_pin_enable(AD9834_Connection* connection);
void AD9834_frequency_register0_pin(AD9834_Connection* connection);
void AD9834_frequency_register1_pin(AD9834_Connection* connection);
void AD9834_phase_register0_pin(AD9834_Connection* connection);
void AD9834_phase_register1_pin(AD9834_Connection* connection);
void AD9834_init(AD9834_Connection* connection);

#endif // AD9834_LIB
