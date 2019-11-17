/**
 *Name        : cst0xx_common.h
 *Author      : gary
 *Version     : V1.0
 *Create      : 2018-1-23
 *Copyright   : zxzz
 */


#ifndef CST0XX_COMMON_H__
#define CST0XX_COMMON_H__


#include "cst0xx_config.h"

/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define HYN_DRIVER_VERSION                  "Hynitron V0.1"

#define FLAGBIT(x)              (0x00000001 << (x))
#define FLAGBITS(x, y)          ((0xFFFFFFFF >> (32 - (y) - 1)) << (x))

#define I2C_BUFFER_LENGTH_MAXINUM           256
#define FILE_NAME_LENGTH                    128
#define ENABLE                              1
#define DISABLE                             0
/*register address*/
#define HYN_REG_INT_CNT                     0x8F
#define HYN_REG_FLOW_WORK_CNT               0x91
#define HYN_REG_WORKMODE                    0x00
#define HYN_REG_WORKMODE_FACTORY_VALUE      0x40
#define HYN_REG_WORKMODE_WORK_VALUE         0x00
#define HYN_REG_CHIP_ID                     0xA3
#define HYN_REG_CHIP_ID2                    0x9F
#define HYN_REG_POWER_MODE                  0xA5
#define HYN_REG_POWER_MODE_SLEEP_VALUE      0x03
#define HYN_REG_FW_VER                      0xA6
#define HYN_REG_VENDOR_ID                   0xA8
#define HYN_REG_LCD_BUSY_NUM                0xAB
#define HYN_REG_FACE_DEC_MODE_EN            0xB0
#define HYN_REG_GLOVE_MODE_EN               0xC0
#define HYN_REG_COVER_MODE_EN               0xC1
#define HYN_REG_CHARGER_MODE_EN             0x8B
#define HYN_REG_GESTURE_EN                  0xD0
#define HYN_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define HYN_REG_ESD_SATURATE                0xED


/* i2c communication*/
int hyn_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
int hyn_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
int hyn_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
int hyn_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
int hyn_i2c_init(void);
int hyn_i2c_exit(void);

/* Apk and functions */
#if HYN_APK_NODE_EN
int hyn_create_apk_debug_channel(struct i2c_client * client);
void hyn_release_apk_debug_channel(void);
#endif

/* ADB functions */
#if HYN_SYSFS_NODE_EN
int hyn_create_sysfs(struct i2c_client *client);
int hyn_remove_sysfs(struct i2c_client *client);
#endif

/* Other */
extern int g_show_log;
int hyn_reset_proc(int hdelayms);

void hyn_irq_disable(void);
void hyn_irq_enable(void);

#if HYN_DEBUG_EN
#define HYN_DEBUG_LEVEL     1

#if (HYN_DEBUG_LEVEL == 2)
#define HYN_DEBUG(fmt, args...) printk(KERN_ERR "[HYNITRON][%s]"fmt"\n", __func__, ##args)
#else
#define HYN_DEBUG(fmt, args...) printk(KERN_ERR "[HYNITRON]"fmt"\n", ##args)
#endif

#define HYN_FUNC_ENTER() printk(KERN_ERR "[HYNITRON] %s: Enter\n", __func__)
#define HYN_FUNC_EXIT()  printk(KERN_ERR "[HYNITRON] %s: Exit(%d)\n", __func__, __LINE__)
#else
#define HYN_DEBUG(fmt, args...)
#define HYN_FUNC_ENTER()
#define HYN_FUNC_EXIT()
#endif

#define HYN_INFO(fmt, args...) do { \
            if (g_show_log) {printk(KERN_ERR "[HYN][Info]"fmt"\n", ##args);} \
        }  while (0)

#define HYN_ERROR(fmt, args...)  do { \
             if (g_show_log) {printk(KERN_ERR "[HYN][Error]"fmt"\n", ##args);} \
        }  while (0)


#endif /*CST0XX_COMMON_H__*/

