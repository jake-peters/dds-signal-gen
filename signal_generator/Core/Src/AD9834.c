/**
 *
 * @brief 	Library for Analog Devices DDS AD9834
 *
 */

#include "AD9834.h"



/**
  * @brief  Set square wave output from SIGN BIT OUT.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_set_square_output(AD9834_Connection* connection) {
	connection->control_reg = connection->control_reg & 0x3FF8; // Set MODE = 0
	connection->control_reg = connection->control_reg | 0x0038; // Set OBPITEN = 1, SIGN/PIB = 1, DIV2 = 1
	uint8_t control_register[2];
	control_register[1] = (uint8_t)(connection->control_reg >> 8);
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set sinusoidal wave output.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_set_sinusoidal_output(AD9834_Connection* connection) {
	connection->control_reg = connection->control_reg & 0x3FD8; // Set MODE = 0 so SIN ROM is used, Set OPBITEN = 0 to set SIGN BIT OUT high impedance
	uint8_t control_register[2];
	control_register[1] = (uint8_t)(connection->control_reg >> 8);
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set triangular wave output.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_set_triangle_output(AD9834_Connection* connection) {
	connection->control_reg = connection->control_reg & 0x3FDA; // Set OBPITEN = 0 to set SIGN BIT OUT high impedance
	connection->control_reg = connection->control_reg | 0x0002; // Set MODE = 1 so SIN ROM is bypassed
	uint8_t control_register[2];
	control_register[1] = (uint8_t)(connection->control_reg >> 8);
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set AD9834 phase.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  * @param	phase		Desired phase in degrees, 0 <= phase <= 360
  * @param	reg			Desired phase register to write to, 0=FREQ0 and 1=FREQ1
  *
  * @return void
  */
void AD9834_set_phase(AD9834_Connection* connection, uint16_t phase, uint8_t reg) {
	if (phase > 360) {
		phase = 360;
	} else if (phase < 0) {
		phase = 0;
	}
	uint16_t phase_reg = (uint16_t)(((float)phase)*PHASE_OFFSET);

	uint8_t phase_register[2];
	phase_register[0] = phase_reg & 0xFF;
	if (reg) {
		phase_register[1] = (uint8_t)(((phase_reg >> 8) & 0x0F) | 0xE0); // Select frequency register 1
	} else {
		phase_register[1] = (uint8_t)(((phase_reg >> 8) & 0x0F) | 0xC0); // Select frequency register 0
	}

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)phase_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set AD9834 frequency using all 28 bits of frequency register.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  * @param	frequency	Desired frequency in Hz
  * @param	reg			Desired frequency register to write to, 0=FREQ0 and 1=FREQ1
  *
  * @return void
  */
void AD9834_set_frequency(AD9834_Connection* connection, uint32_t frequency, uint8_t reg) {
	if (frequency > MAX_FREQUENCY) {
		frequency = MAX_FREQUENCY;
	}
	uint32_t freq_reg_whole = (uint32_t)(((float)frequency)*FREQ_OFFSET);
	uint16_t freq_reg_msb = (int16_t)((freq_reg_whole & 0x0FFFC000) >> 14);
	uint16_t freq_reg_lsb = (int16_t)(freq_reg_whole & 0x00003FFF);

	uint8_t freq_register[4];
	uint8_t control_register[2];
	freq_register[0] = freq_reg_lsb & 0xFF;
	freq_register[2] = freq_reg_msb & 0xFF;

	if (reg) {
		freq_register[1] = (uint8_t)((freq_reg_lsb >> 8) | 0x80); // Select frequency register 1
		freq_register[3] = (uint8_t)((freq_reg_msb >> 8) | 0x80);
	} else {
		freq_register[1] = (uint8_t)((freq_reg_lsb >> 8) | 0x40); // Select frequency register 0
		freq_register[3] = (uint8_t)((freq_reg_msb >> 8) | 0x40);
	}
	control_register[1] = (uint8_t)((connection->control_reg >> 8) | 0x20); // Two consecutive writes to frequency register
	control_register[0] = (uint8_t)(connection->control_reg & 0xFF);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)freq_register, 2, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set AD9834 frequency using 14 LSBs of frequency register.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  * @param	frequency	Desired frequency in Hz
  * @param	reg			Desired frequency register to write to, 0=FREQ0 and 1=FREQ1
  *
  * @return void
  */
void AD9834_set_frequency_lsb(AD9834_Connection* connection, uint32_t frequency, uint8_t reg) {
	if (frequency > MAX_FREQUENCY) {
		frequency = MAX_FREQUENCY;
	}
	uint32_t freq_reg_whole = (uint32_t)(((float)frequency)*FREQ_OFFSET);
	uint16_t freq_reg_lsb = (int16_t)(freq_reg_whole & 0x00003FFF);

	uint8_t freq_register[2];
	uint8_t control_register[2];
	freq_register[0] = freq_reg_lsb & 0xFF;
	if (reg) {
		freq_register[1] = (uint8_t)((freq_reg_lsb >> 8) | 0x80); // Select frequency register 1
	} else {
		freq_register[1] = (uint8_t)((freq_reg_lsb >> 8) | 0x40); // Select frequency register 0
	}
	control_register[1] = (uint8_t)((connection->control_reg >> 8) & 0x0F); // Write to 14 LSBs of frequency register
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)freq_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set AD9834 frequency using 14 MSBs of frequency register.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  * @param	frequency	Desired frequency in Hz
  * @param	reg			Desired frequency register to write to, 0=FREQ0 and 1=FREQ1
  *
  * @return void
  */
void AD9834_set_frequency_msb(AD9834_Connection* connection, uint32_t frequency, uint8_t reg) {
	if (frequency > MAX_FREQUENCY) {
		frequency = MAX_FREQUENCY;
	}
	uint32_t freq_reg_whole = (uint32_t)(((float)frequency)*FREQ_OFFSET);
	uint16_t freq_reg_msb = (int16_t)((freq_reg_whole & 0x0FFFC000) >> 14);

	uint8_t freq_register[2];
	uint8_t control_register[2];
	freq_register[0] = freq_reg_msb & 0xFF;
	if (reg) {
		freq_register[1] = (uint8_t)((freq_reg_msb >> 8) | 0x80); // Select frequency register 1
	} else {
		freq_register[1] = (uint8_t)((freq_reg_msb >> 8) | 0x40); // Select frequency register 0
	}
	control_register[1] = (uint8_t)((connection->control_reg >> 8) & 0x1F); // Write to 14 MSBs of frequency register
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)freq_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Transmit control register to AD9834
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_set_control_reg(AD9834_Connection* connection) {
	uint8_t control_register[2];
	control_register[1] = (uint8_t)(connection->control_reg >> 8);
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set reset line low on AD9834. This enable the output on the AD9834.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_reset_pin_disable(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->reset_gpio, connection->reset_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set reset line high on AD9834. This disable the output on the AD9834.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_reset_pin_enable(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->reset_gpio, connection->reset_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set reset bit low in AD9834 Control Register. This enable the output on the AD9834.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_reset_bit_disable(AD9834_Connection* connection) {
	uint8_t control_register[2];
	control_register[1] = (uint8_t)((connection->control_reg >> 8) & 0xFE);
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set reset bit high in AD9834 Control Register. This disable the output on the AD9834.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_reset_bit_enable(AD9834_Connection* connection) {
	uint8_t control_register[2];
	control_register[1] = (uint8_t)((connection->control_reg >> 8) | 0x01);
	control_register[0] = (uint8_t)(connection->control_reg  & 0xFF);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(connection->hspi, (uint8_t*)control_register, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
}

/**
  * @brief  Disable sleep mode.
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_sleep_pin_disable(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->sleep_gpio, connection->sleep_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Enable sleep mode.
  *
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_sleep_pin_enable(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->sleep_gpio, connection->sleep_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set AD9834 to use frequency register 0
  *
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_frequency_register0_pin(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->freq_select_gpio, connection->freq_select_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set AD9834 to use frequency register 0
  *
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_frequency_register1_pin(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->freq_select_gpio, connection->freq_select_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set AD9834 to use frequency register 0
  *
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_phase_register0_pin(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->phase_select_gpio, connection->phase_select_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set AD9834 to use frequency register 0
  *
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_phase_register1_pin(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->phase_select_gpio, connection->phase_select_pin, GPIO_PIN_SET);
}

/**
  * @brief  Initialize AD9834 with the following:
  * 			- Set reset pin high to disable output
  * 			- Use external pins instead of control bits to control freq/phase registers, reset and sleep
  * 			- SIGN BIT OUT output disable
  * 			- Sinusoidal signal output
  * 			- Select frequency and phase register 0
  *
  * @param 	connection 	Struct containing connection information to AD9834
  *
  * @return void
  */
void AD9834_init(AD9834_Connection* connection) {
	HAL_GPIO_WritePin(connection->fsync_gpio, connection->fsync_pin, GPIO_PIN_SET);
	AD9834_reset_pin_enable(connection);
	AD9834_sleep_pin_disable(connection);
	AD9834_frequency_register0_pin(connection);
	AD9834_phase_register0_pin(connection);

	connection->control_reg = 0x0200; // binary is 0000001000000000
	AD9834_set_control_reg(connection);
	AD9834_set_control_reg(connection); // STM32 SPI HAL Driver doesn't start high so need to do twice if not using pull up resistor
}
