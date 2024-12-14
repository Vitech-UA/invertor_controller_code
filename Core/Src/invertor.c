/*
 * invertor.c
 *
 *  Created on: Nov 17, 2024
 *      Author: Vitech-UA
 */

#ifndef INC_INVERTOR_C_
#define INC_INVERTOR_C_
#include "invertor.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
MCP23017_HandleTypeDef hmcp;

void connect_charger(bool state)
{

}
void set_12V(bool state)
{
	mcp23013_set_pin_state(&hmcp, CEN12V_PORT, CEN12V_PIN, state);
}

void set_bridge_power(bool state)
{
	mcp23013_set_pin_state(&hmcp, CEN_BRIDGE_POWER_PORT, CEN_BRIDGE_POWER_PIN, state);
}

void set_buzzer(bool state)
{
HAL_GPIO_WritePin(cEN_BUZZER_GPIO_Port, cEN_BUZZER_Pin, state);
}

void init_gpio_expander(void)
{
	mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_20); // init IC
	mcp23013_set_pin_dir(&hmcp, CEN12V_PORT, CEN12V_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_dir(&hmcp, CEN_BRIDGE_POWER_PORT, CEN_BRIDGE_POWER_PIN, GPIO_OUTPUT);

}

#endif /* INC_INVERTOR_C_ */
