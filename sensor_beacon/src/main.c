/*
 * Copyright (c) 2019 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <soc.h>
#include <device.h>
#include <gpio.h>
#include <sensor.h>
#include <misc/printk.h>
#include <stdio.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

/*
 * Beacon device manufacturer's company identifier code.
 * Please set it to your MFG ID.
 */
#define BEACON_MFG_ID			0xffff


#define LOW_POWER_STATE_TIME		200

#define BEACON_ADVERTISING_TIME		200

#define PROXIMITY_THOLD			200

#define MAX_NUMOF_BELOW_THOLD		100

K_SEM_DEFINE(bt_ready_sem, 0, 1);
static struct k_delayed_work adv_timer;
static struct device *apds9960_dev;
static struct device *hdc1010_dev;

static float numof_proxy_events = 0;
static float numof_below_thold = 0;

#define BT_LE_ADV_PRJ BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
				      BT_GAP_ADV_FAST_INT_MIN_2, \
				      BT_GAP_ADV_FAST_INT_MAX_2)

enum sensor_type_id {
	SENSOR_ID_NONE = 0,
	SENSOR_ID_TEMPERATURE,
	SENSOR_ID_HUMIDITY,
	SENSOR_ID_PROX,
	SENSOR_ID_NUMOF_EVENTS,
};

struct beacon_sensor_data {
	u8_t type[4];
	union {
		u8_t values[16];
		float fvalues[4];
	};
} __packed;

struct altbeacon_proto_fields {
	u16_t mfg_id;
	u16_t beacon_code;
	/* beacon code 20 octets */
	struct beacon_sensor_data data;
	/* reference RSSI */
	s8_t rssi;
	/* reserved octet */
	u8_t reserved;
} __packed;

static struct altbeacon_proto_fields mfg_data = {
	.mfg_id = BEACON_MFG_ID,
	.beacon_code = 0xacbe,
	.data = {
		.type[0] = SENSOR_ID_NONE,
		.values = {0},
	},
	.rssi = -40,
	.reserved = 0x00,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_LIMITED),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &mfg_data, sizeof(mfg_data)),
};

static int get_apds9960_val(struct sensor_value *val)
{
	if (sensor_sample_fetch(apds9960_dev)) {
		printk("Failed to fetch sample for device %s\n",
		       DT_AVAGO_APDS9960_0_LABEL);
		return -1;
	}

	if (sensor_channel_get(apds9960_dev, SENSOR_CHAN_PROX, &val[0])) {
		return -1;
	}

	return 0;
}

static int get_hdc1010_val(struct sensor_value *val)
{
	if (sensor_sample_fetch(hdc1010_dev)) {
		printk("Failed to fetch sample for device %s\n",
		       DT_TI_HDC1010_0_LABEL);
		return -1;
	}

	if (sensor_channel_get(hdc1010_dev,
			       SENSOR_CHAN_AMBIENT_TEMP,
			       &val[0])) {
		return -1;
	}

	if (sensor_channel_get(hdc1010_dev,
			       SENSOR_CHAN_HUMIDITY,
			       &val[1])) {
		return -1;
	}

	return 0;
}

static void bt_ready(int err)
{
	k_sem_give(&bt_ready_sem);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	return;
}

static void do_adv_stop(struct k_work *work)
{
	printk("adv stop\n");
	int err = bt_le_adv_stop();
	if (err) {
		printk("Advertising failed to stop (err %d)\n", err);
	}
}

static void measure_and_update_adv(void)
{
	int err;
	struct sensor_value hdc1010_val[2];
	struct sensor_value apds9960_val[1];

	/*
	 * size_t bt_addr_count = ARRAY_SIZE(bt_addr);
	 * bt_addr_le_t bt_addr[1];
	 * bt_id_get(bt_addr, &bt_addr_count);
	 * if (bt_addr_count == 0) {
	 * 	printk("Failed to fetch bluetooth id\n");
	 * 	return;
	 * }
	 * printk("addr type %d\n", bt_addr[0].type);
	 * printk("addr 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x\n",
	 *        bt_addr[0].a.val[5],
	 *        bt_addr[0].a.val[4],
	 *        bt_addr[0].a.val[3],
	 *        bt_addr[0].a.val[2],
	 *        bt_addr[0].a.val[1],
	 *        bt_addr[0].a.val[0]);
	 */

	/* Check I2C functionality */
	if (get_hdc1010_val(hdc1010_val)) {
		printk("Failed to fetch T|RH value\n");
		return;
	}

	do {
		k_sleep(K_MSEC(LOW_POWER_STATE_TIME));
		/* Reset all types and values */
		memset(&mfg_data.data, 0, sizeof(mfg_data.data));

		if (!get_apds9960_val(apds9960_val)) {
			printk("Proximity:%d\n", apds9960_val[0].val1);
			/* Check if value is over threshold */
			if (apds9960_val[0].val1 < PROXIMITY_THOLD) {
				numof_below_thold ++;
				if (numof_below_thold > MAX_NUMOF_BELOW_THOLD) {
					numof_below_thold = 0;
					printk("Timeout\n");
				} else {
					continue;
				}
			} else {
				mfg_data.data.type[2] = SENSOR_ID_PROX;
				mfg_data.data.fvalues[2] = apds9960_val[0].val1;
				numof_below_thold = 0;
				/* Increment number of events */
				numof_proxy_events ++;
				mfg_data.data.type[3] = SENSOR_ID_NUMOF_EVENTS;
				mfg_data.data.fvalues[3] = numof_proxy_events;
			}
		} else {
			printk("Failed to fetch proximity value\n");
		}

		if (!get_hdc1010_val(hdc1010_val)) {
			mfg_data.data.type[0] = SENSOR_ID_TEMPERATURE;
			mfg_data.data.fvalues[0] = hdc1010_val[0].val1 +
					(float)hdc1010_val[0].val2 / 1000000.0;

			mfg_data.data.type[1] = SENSOR_ID_HUMIDITY;
			mfg_data.data.fvalues[1] = hdc1010_val[1].val1 +
					(float)hdc1010_val[1].val2 / 1000000.0;

			printk("T: %d.%d C | RH: %d%%\n",
			       hdc1010_val[0].val1,
			       hdc1010_val[0].val2/100000,
			       hdc1010_val[1].val1);
		} else {
			printk("Failed to fetch T|RH values\n");
		}

		err = bt_le_adv_start(BT_LE_ADV_PRJ, ad, ARRAY_SIZE(ad),
				      NULL, 0);
		if (err) {
			/* This will also happens if the frequency of the
			 * event is to high and the controller still
			 * advertises. */
			printk("Adv failed to start (err %d)\n", err);
		}

		k_delayed_work_submit(&adv_timer,
				      K_MSEC(BEACON_ADVERTISING_TIME));

	} while (1);
}


#define PERIPH_PON_PIN		0

static void force_dcdc_low_power_mode(void)
{
	volatile NRF_GPIO_Type *gpio = NRF_P1;

	/*
	 * Workaround to force low power mode of power regulator.
	 */
	gpio->PIN_CNF[PERIPH_PON_PIN] =
		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
		(GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	gpio->OUTCLR = BIT(PERIPH_PON_PIN);
}

void main(void)
{
	int err;

	force_dcdc_low_power_mode();
	k_delayed_work_init(&adv_timer, do_adv_stop);

	hdc1010_dev = device_get_binding(DT_TI_HDC1010_0_LABEL);
	if (hdc1010_dev == NULL) {
		printk("Failed to get %s device\n", DT_TI_HDC1010_0_LABEL);
		return;
	}

	apds9960_dev = device_get_binding(DT_AVAGO_APDS9960_0_LABEL);
	if (apds9960_dev == NULL) {
		printk("Failed to get %s device\n", DT_AVAGO_APDS9960_0_LABEL);
		return;
	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	k_sem_take(&bt_ready_sem, K_FOREVER);
	printk("\n*** Sensor Beacon Demo on %s ***\n", CONFIG_BOARD);

	measure_and_update_adv();
}
