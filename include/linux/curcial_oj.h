#ifndef _CURCIAL_OJ_H
#define _CURCIAL_OJ_H

#define CURCIAL_OJ_NAME "curcial_oj"

struct curcial_oj_platform_data {
	struct input_dev *input_dev;
	struct work_struct work;
	bool click;
	uint8_t key;
	uint32_t last_key_time;
	bool ap_code;
	uint8_t degree;
	uint8_t  debugflag;
	uint32_t last_click_time;
	uint16_t interval;
	uint8_t mdelay_time;
	int8_t normal_th;
	int8_t xy_ratio;
	void (*oj_shutdown)(int);
	int (*oj_poweron)(int);
	void(*oj_adjust_xy)(uint8_t *, int16_t *, int16_t *);
	void (*oj_reset)(int);
	int microp_version;
	bool share_power;
	bool reset_pin;
	bool swap;
	int x;
	int y;
	uint8_t Xsteps[30];
	uint8_t Ysteps[30];
	uint16_t sht_tbl[10];
	uint8_t pxsum_tbl[10];
	int irq;

	unsigned irq_gpio;
	unsigned rst_gpio;
	uint8_t  ledval;
	int device_id;
};
void curcial_oj_send_key(unsigned int code, int value);

#endif
