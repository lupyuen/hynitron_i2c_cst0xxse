/**
 *Name        : cst0xx_core.h
 *Author      : gary
 *Version     : V1.0
 *Create      : 2018-1-23
 *Copyright   : zxzz
 */


#ifndef CST0XX_CORE_H__
#define CST0XX_CORE_H__


#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
//#include <linux/sched/prio.h>
#include <linux/rtpm_prio.h>
#include <linux/fs.h>
#include <linux/device.h>
#include "tpd.h"

#include <linux/wait.h>
#include <linux/time.h>
#include <linux/dma-mapping.h>
//#include <linux/of.h>
//#include <linux/of_address.h>
//#include <linux/of_device.h>
//#include <linux/of_gpio.h>
//#include <linux/of_irq.h>
#include "cust_gpio_usage.h"
#include <pmic_drv.h>
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/irqs.h>

#include "cst0xx_common.h"

#define TPD_HAVE_BUTTON                         1
#define TPD_KEY_COUNT                           1
#define TPD_BUTTON_HEIGH                    (60)      //100
#define TPD_KEYS                            {KEY_BACK}        // {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
//#define TPD_KEYS_DIM                      {{100,1000,20,TPD_BUTTON_HEIGH}, {200,1000,20,TPD_BUTTON_HEIGH}, {300,1000,20,TPD_BUTTON_HEIGH}}
#define TPD_KEYS_DIM                        {{160,350,200,TPD_BUTTON_HEIGH} }

#define TPD_POWER_SOURCE_CUSTOM            PMIC_APP_CAP_TOUCH_VDD
#define CFG_MAX_TOUCH_POINTS                               5

#define HYN_MAX_POINTS                          10
#define HYN_MAX_ID                              0x0F
#define HYN_TOUCH_STEP                          6
#define HYN_FACE_DETECT_POS                     1
#define HYN_TOUCH_X_H_POS                       3
#define HYN_TOUCH_X_L_POS                       4
#define HYN_TOUCH_Y_H_POS                       5
#define HYN_TOUCH_Y_L_POS                       6
#define HYN_TOUCH_EVENT_POS                     3
#define HYN_TOUCH_ID_POS                        5
#define FT_TOUCH_POINT_NUM                      2
#define HYN_TOUCH_XY_POS                        7
#define HYN_TOUCH_MISC                          8
#define POINT_READ_BUF                          (3 + HYN_TOUCH_STEP * HYN_MAX_POINTS)
#define TPD_DELAY                               (2*HZ/100)
#define HYN_SYSFS_ECHO_ON(buf)      ((strnicmp(buf, "1", 1)  == 0) || \
                                        (strnicmp(buf, "on", 2) == 0))
#define HYN_SYSFS_ECHO_OFF(buf)     ((strnicmp(buf, "0", 1)  == 0) || \
                                        (strnicmp(buf, "off", 3) == 0))



/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct touch_info
{
    int y[HYN_MAX_POINTS];
    int x[HYN_MAX_POINTS];
    int p[HYN_MAX_POINTS];
    int id[HYN_MAX_POINTS];
    int count;
};


/*touch event info*/
struct ts_event
{
    u16 au16_x[HYN_MAX_POINTS];               /* x coordinate */
    u16 au16_y[HYN_MAX_POINTS];               /* y coordinate */
    u8 au8_touch_event[HYN_MAX_POINTS];       /* touch event: 0 -- down; 1-- up; 2 -- contact */
    u8 au8_finger_id[HYN_MAX_POINTS];         /* touch ID */
    u16 pressure[HYN_MAX_POINTS];
    u16 area[HYN_MAX_POINTS];
    u8 touch_point;
    int touchs;
    u8 touch_point_num;
};

extern struct input_dev *hyn_input_dev;
extern struct tpd_device *tpd;
extern unsigned int tpd_rst_gpio_number;
extern struct i2c_client *hyn_i2c_client;

#endif /*CST0XX_CORE_H__*/



