
#ifndef YOC_DEVICELIST_H
#define YOC_DEVICELIST_H

#include <stdint.h>
#include <yoc/uservice.h>
#include <devices/esp8266.h>
#include <devices/sim800.h>
#include <devices/ethernet.h>
#include <devices/hal/battery_impl.h>


typedef struct rtl8723ds_gpio_pin_t {
    int wl_en;
    int power;
} rtl8723ds_gpio_pin;


extern void uart_csky_register(int idx);
extern void flash_csky_register(int idx);
extern void spiflash_csky_register(int idx);

extern void sensor_dht11_register(void *config, int idx);
extern void sensor_sht20_register(void *config, int idx);
extern void sensor_dht22_register(void *config, int idx);
extern void sensor_simulate_register(void *config, int idx);
extern void sensor_mpu6050_register(void);
extern void sensor_light_csky_register(void *config, int idx);
extern void led_rgb_register(void *config, int idx);

extern void battery_simulate_register(battery_pin_config_t *config, int idx);
extern void eth_enc26j60_register(eth_config_t *eth_config);
extern void wifi_esp8266_register(utask_t *task, esp_wifi_param_t *param);
extern void gprs_sim800_register(utask_t *task, sim_gprs_param_t *param);
extern void wifi_m88wi6700s_register(void);
extern void wifi_rtl8723ds_register(rtl8723ds_gpio_pin* config);

extern void snd_card_register(int vol_range);

extern int vfs_fatfs_register(void);
#endif
