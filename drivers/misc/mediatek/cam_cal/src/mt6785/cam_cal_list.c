/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define IMX586_MAX_EEPROM_SIZE 0x24D0
#define OV8856_MAX_EEPROM_SIZE 0x8000
#define S5K4H7_MAX_EEPROM_SIZE 0x8000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{S5K2LQSX_SENSOR_ID, 0xA0, Common_read_region},
	{S5KHM2SP_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, IMX586_MAX_EEPROM_SIZE},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{OV8856_SENSOR_ID, 0x6C, Common_read_region, OV8856_MAX_EEPROM_SIZE},
	{S5K4H7_SENSOR_ID, 0x20, Common_read_region, S5K4H7_MAX_EEPROM_SIZE},
	//prize add by zhuzhengjiang 20200327 for k6505 OTP start
	{IMX582_SENSOR_ID, 0xB0, Common_read_region},
	{OV32A1Q_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4H7YX_SENSOR_ID, 0xA2, Common_read_region},
	{S5KGM1ST_SENSOR_ID, 0xB0, Common_read_region},
	{OV16A1Q_SENSOR_ID, 0xA0, Common_read_region},
	{OV64B40_SENSOR_ID, 0xA2, Common_read_region},
//prize add by lipengpeng 20210317 start 
	{S5KGM1SP_SENSOR_ID, 0xA0, Common_read_region},
	//{S5KGM1SPGMS_SENSOR_ID, 0xB0, Common_read_region},
//prize add by lipengpeng 20210317 end 
	//prize add by zhuzhengjiang 20200327 for k6505 OTP end
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


