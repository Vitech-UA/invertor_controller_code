/*
 * energy_monitor.c
 *
 *  Created on: Dec 5, 2024
 *      Author: Vitech-UA
 */

#include "energy_monitor.h"

void set_energy_monitor_pwr(bool state)
{
 HAL_GPIO_WritePin(cEN_ENERGY_MONITOR_GPIO_Port, cEN_ENERGY_MONITOR_Pin, state);
}
