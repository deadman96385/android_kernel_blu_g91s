/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include "gpio.h"

struct GPIO_PINCTRL gpio_pinctrl_list_cam[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
};

#ifdef MIPI_SWITCH
struct GPIO_PINCTRL gpio_pinctrl_list_switch[
			GPIO_CTRL_STATE_MAX_NUM_SWITCH] = {
	{"cam_mipi_switch_en_1"},
	{"cam_mipi_switch_en_0"},
	{"cam_mipi_switch_sel_1"},
	{"cam_mipi_switch_sel_0"}
};
#endif

static struct GPIO gpio_instance;
// prize by zhuzhengjaing for avdd2 1.8v 20200113 start
struct pinctrl		 *ppinctrl_cam;
struct pinctrl_state *ppinctrl_maincam_avdd2_h;
struct pinctrl_state *ppinctrl_maincam_avdd2_l;

struct pinctrl_state *ppinctrl_maincam_af_h;
struct pinctrl_state *ppinctrl_maincam_af_l;
// prize by zhuzhengjaing for avdd2 1.8v 20200113 start
static enum IMGSENSOR_RETURN gpio_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	int    i, j;
	struct GPIO            *pgpio            = (struct GPIO *)pinstance;
	enum   IMGSENSOR_RETURN ret              = IMGSENSOR_RETURN_SUCCESS;
	char str_pinctrl_name[LENGTH_FOR_SNPRINTF];
	char *lookup_names = NULL;


	pgpio->pgpio_mutex = &pcommon->pinctrl_mutex;

	pgpio->ppinctrl = devm_pinctrl_get(&pcommon->pplatform_device->dev);
	if (IS_ERR(pgpio->ppinctrl)) {
		PK_DBG("%s : Cannot find camera pinctrl!", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}
	// prize by zhuzhengjaing for avdd2 1.8v 20200113 start
	ppinctrl_cam = pgpio->ppinctrl;
	ppinctrl_maincam_avdd2_h = pinctrl_lookup_state(pgpio->ppinctrl,"maincam_avdd2_1");
	ppinctrl_maincam_avdd2_l = pinctrl_lookup_state(pgpio->ppinctrl,"maincam_avdd2_0");
	
	ppinctrl_maincam_af_h = pinctrl_lookup_state(pgpio->ppinctrl,"maincam_vcamaf_1");
	ppinctrl_maincam_af_l = pinctrl_lookup_state(pgpio->ppinctrl,"maincam_vcamaf_0");
	// prize by zhuzhengjaing for avdd2 1.8v 20200113 end
	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {
		for (i = 0 ; i < GPIO_CTRL_STATE_MAX_NUM_CAM; i++) {
			lookup_names =
			gpio_pinctrl_list_cam[i].ppinctrl_lookup_names;

			if (lookup_names) {
				snprintf(str_pinctrl_name,
				sizeof(str_pinctrl_name),
				"cam%d_%s",
				j,
				lookup_names);
				pgpio->ppinctrl_state_cam[j][i] =
					pinctrl_lookup_state(
						pgpio->ppinctrl,
						str_pinctrl_name);
			}

			if (pgpio->ppinctrl_state_cam[j][i] == NULL ||
				IS_ERR(pgpio->ppinctrl_state_cam[j][i])) {
				PK_DBG(
					"%s : pinctrl err, %s\n",
					__func__,
					str_pinctrl_name);
				ret = IMGSENSOR_RETURN_ERROR;
			}
		}
	}
#ifdef MIPI_SWITCH
	for (i = 0; i < GPIO_CTRL_STATE_MAX_NUM_SWITCH; i++) {
		printk("zzj add for mipi switch \n");
		if (gpio_pinctrl_list_switch[i].ppinctrl_lookup_names) {
			pgpio->ppinctrl_state_switch[i] =
				pinctrl_lookup_state(
					pgpio->ppinctrl,
			gpio_pinctrl_list_switch[i].ppinctrl_lookup_names);
		}

		if (pgpio->ppinctrl_state_switch[i] == NULL ||
			IS_ERR(pgpio->ppinctrl_state_switch[i])) {
			pr_info(
				"%s : pinctrl err, %s\n",
				__func__,
			gpio_pinctrl_list_switch[i].ppinctrl_lookup_names);
			ret = IMGSENSOR_RETURN_ERROR;
		}
	}
#endif

	return ret;
}

static enum IMGSENSOR_RETURN gpio_release(void *pinstance)
{
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN gpio_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct pinctrl_state  *ppinctrl_state;
	struct GPIO           *pgpio = (struct GPIO *)pinstance;
	enum   GPIO_STATE      gpio_state;

	if (pin < IMGSENSOR_HW_PIN_PDN ||
#ifdef MIPI_SWITCH
	    pin > IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL ||
#else
	   pin > IMGSENSOR_HW_PIN_DOVDD ||
#endif
	   pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
	   pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
		return IMGSENSOR_RETURN_ERROR;

	gpio_state =
		(pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_0) ?
		GPIO_STATE_H : GPIO_STATE_L;
	
	   // prize by zhuzhengjaing for avdd2 1.8v 20200113 start
	   if(sensor_idx == 0 && pin == IMGSENSOR_HW_PIN_AVDD) {
		printk("set gpio avdd 2 pin_state=%d\n",pin_state);	

		if (!IS_ERR(ppinctrl_maincam_avdd2_h) && (gpio_state == GPIO_STATE_H))//prize wyq 20200225 current leakage
		{
		    printk("set gpio avdd 2 to high");
		    pinctrl_select_state(ppinctrl_cam, ppinctrl_maincam_avdd2_h);
		}
		else if (!IS_ERR(ppinctrl_maincam_avdd2_l) && (gpio_state == GPIO_STATE_L))
		{
		    printk("set gpio avdd 2 to low");
		    pinctrl_select_state(ppinctrl_cam, ppinctrl_maincam_avdd2_l);
		}
		else
		    pinctrl_select_state(ppinctrl_cam, ppinctrl_maincam_avdd2_l);
	   }

	   // af
	   if((sensor_idx == 0 ||sensor_idx == 3) && pin == IMGSENSOR_HW_PIN_AVDD) {
	       if (!IS_ERR(ppinctrl_maincam_af_h) && (gpio_state == GPIO_STATE_H))
	       {
		        printk("set gpio vcamaf to high");
		        pinctrl_select_state(ppinctrl_cam, ppinctrl_maincam_af_h);
	       }
	       else if (!IS_ERR(ppinctrl_maincam_af_l) && (gpio_state == GPIO_STATE_L))
	       {
	            printk("set gpio vcamaf to low");
	            pinctrl_select_state(ppinctrl_cam, ppinctrl_maincam_af_l);
	       }
	       else
	            pinctrl_select_state(ppinctrl_cam, ppinctrl_maincam_af_l);
	   }
	   // prize by zhuzhengjaing  
       printk("zzj add gpio_set entry sensor_idx=%d pin=%d pin_state=%d gpio_state=%d",sensor_idx,pin,pin_state,gpio_state);
#ifdef MIPI_SWITCH
	if (pin == IMGSENSOR_HW_PIN_MIPI_SWITCH_EN)
		ppinctrl_state = pgpio->ppinctrl_state_switch[
			GPIO_CTRL_STATE_MIPI_SWITCH_EN_H + gpio_state];
	else if (pin == IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL)
		ppinctrl_state = pgpio->ppinctrl_state_switch[
			GPIO_CTRL_STATE_MIPI_SWITCH_SEL_H + gpio_state];
	else
#endif
	{
		ppinctrl_state =
			pgpio->ppinctrl_state_cam[(unsigned int)sensor_idx][
			((pin - IMGSENSOR_HW_PIN_PDN) << 1) + gpio_state];
	}

	mutex_lock(pgpio->pgpio_mutex);

	if (ppinctrl_state != NULL && !IS_ERR(ppinctrl_state))
		pinctrl_select_state(pgpio->ppinctrl, ppinctrl_state);
	else
		PK_DBG("%s : pinctrl err, PinIdx %d, Val %d\n",
			__func__, pin, pin_state);

	mutex_unlock(pgpio->pgpio_mutex);

	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_GPIO,
	.pinstance = (void *)&gpio_instance,
	.init      = gpio_init,
	.set       = gpio_set,
	.release   = gpio_release
};

enum IMGSENSOR_RETURN imgsensor_hw_gpio_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

