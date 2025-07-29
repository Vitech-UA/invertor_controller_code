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
#include "stdio.h"
extern I2C_HandleTypeDef hi2c1;
MCP23017_HandleTypeDef hmcp;

void config_eg_dead_time(EG_DEAD_TIME_t dead_time) {
	bool dt0_state = dead_time & 0x01;
	bool dt1_state = (dead_time >> 1) & 0x01;
	mcp23013_set_pin_state(&hmcp, DT0_PORT, DT0_PIN, dt0_state);
	mcp23013_set_pin_state(&hmcp, DT1_PORT, DT1_PIN, dt1_state);
}

void set_12V(bool state) {
	mcp23013_set_pin_state(&hmcp, CEN12V_PORT, CEN12V_PIN, state);
}

void set_bridge_power(bool state) {
	mcp23013_set_pin_state(&hmcp, CEN_BRIDGE_POWER_PORT, CEN_BRIDGE_POWER_PIN,
			state);
}

void set_eg_pwm(bool state) {
	mcp23013_set_pin_state(&hmcp, CEN_PWM_PORT, CEN_PWM_PIN, state);
}

void set_buzzer(bool state) {
//	HAL_GPIO_WritePin(cEN_BUZZER_GPIO_Port, cEN_BUZZER_Pin, state);
}

void set_charger(bool state) {
	mcp23013_set_pin_state(&hmcp, CCONNECT_CHARGER_PORT, CCONNECT_CHARGER_PIN,
			state);
}

void set_invertor_freq(EG_INVERTOR_FREQ_t freq) {
	mcp23013_set_pin_state(&hmcp, FQSEL_PORT, FQSEL_PIN, freq);
}

void set_invertor_softstart(bool softstart) {
	mcp23013_set_pin_state(&hmcp, SST_PORT, SST_PIN, !softstart);
}

void init_gpio_expander(void) {
	mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_20); // init IC

	mcp23013_set_pin_dir(&hmcp, CEN12V_PORT, CEN12V_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, CEN12V_PORT, CEN12V_PIN, false);

	mcp23013_set_pin_dir(&hmcp, CCONNECT_CHARGER_PORT, CCONNECT_CHARGER_PIN,
			GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, CCONNECT_CHARGER_PORT, CCONNECT_CHARGER_PIN,
	false);

	mcp23013_set_pin_dir(&hmcp, CEN_BRIDGE_POWER_PORT, CEN_BRIDGE_POWER_PIN,
			GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, CEN_BRIDGE_POWER_PORT, CEN_BRIDGE_POWER_PIN,
	true);

	mcp23013_set_pin_dir(&hmcp, DT0_PORT, DT0_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, DT0_PORT, DT0_PIN, false);

	mcp23013_set_pin_dir(&hmcp, DT1_PORT, DT1_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, DT1_PORT, DT1_PIN, false);

	mcp23013_set_pin_dir(&hmcp, CEN_PWM_PORT, CEN_PWM_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, CEN_PWM_PORT, CEN_PWM_PIN, false);

	mcp23013_set_pin_dir(&hmcp, FQSEL_PORT, FQSEL_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, FQSEL_PORT, FQSEL_PIN, false);

	mcp23013_set_pin_dir(&hmcp, SST_PORT, SST_PIN, GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, SST_PORT, SST_PIN, false);

	mcp23013_set_pin_dir(&hmcp, CEN_220V_OUT_PORT, CEN_220V_OUT_PIN,
			GPIO_OUTPUT);
	mcp23013_set_pin_state(&hmcp, CEN_220V_OUT_PORT, CEN_220V_OUT_PIN, false);

}

void set_220v_out(bool state) {
	mcp23013_set_pin_state(&hmcp, CEN_220V_OUT_PORT, CEN_220V_OUT_PIN, state);
}

void print_ibat(float i_bat_value) {
	char LCD_BUFFER[20] = { 0 };
	sprintf(LCD_BUFFER, "IBAT: %4.1f", i_bat_value);
	ST7789_WriteString(0, 30, LCD_BUFFER, Font_16x26, WHITE, BLACK);
}

void print_vbat(uint16_t raw_adc_vbat_value) {
	float v_bat_volt = 0;
	v_bat_volt = calculate_vbat(raw_adc_vbat_value);
	char LCD_BUFFER[20] = { };
	sprintf(LCD_BUFFER, "VBAT: %.1f V", v_bat_volt);
	ST7789_WriteString(0, 0, LCD_BUFFER, Font_16x26, WHITE, BLACK);
}

void print_ac_vout(float ac_voltage_value) {
	char LCD_BUFFER[20] = { };
	sprintf(LCD_BUFFER, "Vout: %f V", ac_voltage_value);
	ST7789_WriteString(0, 60, LCD_BUFFER, Font_16x26, WHITE, BLACK);
}

void print_ac_power(float ac_power_value) {
	char LCD_BUFFER[10] = { };
	 int value = (int)ac_power_value;
	    if (value > 999) {
	        value = 999;
	    }
	sprintf(LCD_BUFFER, "Pout: %03d W", value);
	ST7789_WriteString(0, 90, LCD_BUFFER, Font_16x26, WHITE, BLACK);
}

float calculate_vbat(uint16_t adc_value) {
	float vref = 3.3f;
	float r_top = 10000.0f;
	float r_bottom = 2200.0f;
	float u_adc = ((float) adc_value / 4095.0f) * vref;
	float vbat = u_adc * (r_top + r_bottom) / r_bottom;
	return vbat;
}
#endif /* INC_INVERTOR_C_ */
