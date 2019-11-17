
/*****************************************************************************
*
* File Name: cst0xx_core.c
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

//#include <linux/sched/prio.h>
#include <linux/rtpm_prio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/wakelock.h>
#include "cst0xx_core.h"
#include "cst0xx_ex_fun.h"
//#include "tpd.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define HYN_DRIVER_NAME                     "hyn_ts"
#define HYN_I2C_SLAVE_ADDR                  0x15//0x2A 0x70 0x15 0x38
#define I2C_EXTENSION_FOCAL                 1
#define CTP_NAME                            "hyn_ts"
/*****************************************************************************
* Static variables
*****************************************************************************/
struct i2c_client *hyn_i2c_client;
struct input_dev *hyn_input_dev;
struct task_struct *thread_tpd;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag;
#if HYN_DEBUG_EN
int g_show_log = 1;
#else
int g_show_log = 0;
#endif
#if 0
unsigned int tpd_rst_gpio_number = 0;
static unsigned int tpd_int_gpio_number = 1;
static unsigned int hyn_touch_irq = 0;
static unsigned int f_hyn_irq_disable = 0;
static spinlock_t irq_lock;
#endif
//taikoto add 20180129
u8 *g_dma_buff_va = NULL;
dma_addr_t g_dma_buff_pa = 0;
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;

static void msg_dma_alloct(void) {
    if (NULL == g_dma_buff_va) {
        tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
    }

    if(!g_dma_buff_va) {
        HYN_DEBUG("[HYNITRON][DMA] Allocate DMA I2C Buffer failed!\n");
    }
}

static void msg_dma_release(void) {
    if(g_dma_buff_va) {
        dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
        g_dma_buff_va = NULL;
        g_dma_buff_pa = 0;
        HYN_DEBUG("[HYNITRON][DMA] Allocate DMA I2C Buffer release!\n");
    }
}

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);
//taikoto add end

#if TPD_HAVE_BUTTON   
        static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
        static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

static int tpd_remove(struct i2c_client *client);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static void hyn_release_all_finger(void);

/*****************************************************************************
* Focaltech ts i2c driver configuration
*****************************************************************************/
static const struct i2c_device_id hyn_tpd_id[] = {{HYN_DRIVER_NAME, 0}, {} };
static struct i2c_board_info __initdata hyn_i2c_tpd={ I2C_BOARD_INFO(HYN_DRIVER_NAME, (0x2A>>1))};
#if 0
static const struct of_device_id hyn_dt_match[] =
{
    {.compatible = "mediatek,cap_touch"},
    {},
};
MODULE_DEVICE_TABLE(of, hyn_dt_match);
#endif

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = HYN_DRIVER_NAME,
        //.of_match_table = of_match_ptr(hyn_dt_match),
    },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = hyn_tpd_id,
    .driver.name = HYN_DRIVER_NAME,
    .detect = tpd_i2c_detect,
};
#if 0
static int of_get_hyn_platform_data(struct device *dev)
{
    const struct of_device_id *match;
    HYN_FUNC_ENTER();
    if (dev->of_node) {
        match = of_match_device(of_match_ptr(hyn_dt_match), dev);
        if (!match) {
            HYN_ERROR("[HYNITRON]: No device match found\n");
            
	    return -ENODEV;
	}
    }
    HYN_FUNC_EXIT();

    return 0;
}
#endif
#if 0
/*****************************************************************************
*  Name: hyn_reset_proc
*  Brief: Execute reset operation
*  Input: hdelayms - delay time unit:ms
*  Output:
*  Return: 0 - Get correct Device ID
*****************************************************************************/
int hyn_reset_proc(int hdelayms)
{   
    tpd_gpio_output(tpd_rst_gpio_number, 0);
    msleep(20);
    tpd_gpio_output(tpd_rst_gpio_number, 1);
    if (hdelayms){
        msleep(hdelayms);
    }

    return 0;
}
#endif 
int hyn_reset_proc(int hdelayms)
{   
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);    

    msleep(20);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

    if (hdelayms){
        msleep(hdelayms);
    }

    return 0;
}

#if 0
/*****************************************************************************
*  Name: hyn_irq_disable
*  Brief: disable irq
*  Input:
*   sync: 1 - call disable_irq(); Can't used in interrup handler
*         0 - call disable_irq_nosync(); Usually used in interrup handler
*  Output:
*  Return:
*****************************************************************************/
void hyn_irq_disable(void)
{
    unsigned long irqflags;	
    
    spin_lock_irqsave(&irq_lock, irqflags);

    if (!f_hyn_irq_disable)
    {
        disable_irq(hyn_touch_irq);
        f_hyn_irq_disable = 1;
    }

    spin_unlock_irqrestore(&irq_lock, irqflags);
}

/*****************************************************************************
*  Name: hyn_irq_enable
*  Brief: enable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void hyn_irq_enable(void)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&irq_lock, irqflags);

    if (f_hyn_irq_disable)
    {
        enable_irq(hyn_touch_irq);
        f_hyn_irq_disable = 0;
    }

    spin_unlock_irqrestore(&irq_lock, irqflags);
}
#endif
#if 1
/*****************************************************************************
*  Reprot related
*****************************************************************************/
/*****************************************************************************
*  Name: hyn_release_all_finger
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void hyn_release_all_finger(void)
{
#if HYN_MT_PROTOCOL_B_EN
    unsigned int finger_count=0;
#endif

    HYN_FUNC_ENTER();
#if (!HYN_MT_PROTOCOL_B_EN)
    input_mt_sync ( tpd->dev );
#else
    for (finger_count = 0; finger_count < CFG_MAX_TOUCH_POINTS; finger_count++)
    {
        input_mt_slot( tpd->dev, finger_count);
        input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
    }
    input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif
    input_sync ( tpd->dev );
    HYN_FUNC_EXIT();
}
#endif

#if (HYN_DEBUG_EN && (HYN_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = {0};

#define HYN_ONE_TCH_LEN     HYN_TOUCH_STEP
static void hyn_show_touch_buffer(u8 *buf, int point_num)
{
    int len = point_num * HYN_ONE_TCH_LEN;
    int count = 0;
    int i;

    memset(g_sz_debug, 0, 1024);
    if (len > (POINT_READ_BUF-3))
    {
        len = POINT_READ_BUF-3;
    }
    else if (len == 0)
    {
        len += HYN_ONE_TCH_LEN;
    }
    count += sprintf(g_sz_debug, "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
    for (i = 0; i < len; i++)
    {
        count += sprintf(g_sz_debug+count, ",%02X", buf[i+3]);
    }
    HYN_DEBUG("buffer: %s", g_sz_debug);
}
#endif

#if (!HYN_MT_PROTOCOL_B_EN)
static void tpd_down(int x, int y, int p, int id)
{
    if ((id < 0) || (id > 9))
        return;
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    input_report_key(tpd->dev, BTN_TOUCH, 1);

#if (HYN_REPORT_PRESSURE_EN)
    p = 0x3f;
    input_report_abs(tpd->dev, ABS_MT_PRESSURE, p);
#endif

    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);

#if HYN_REPORT_PRESSURE_EN
    HYN_DEBUG("[HYNITRON]: P%d(%4d,%4d)[p:%d] DOWN!", id, x, y, p);
#else
    HYN_DEBUG("[HYNITRON]: P%d(%4d,%4d) DOWN!", id, x, y );
#endif
}

static void tpd_up(int x, int y)
{
    HYN_DEBUG("[HYNITRON]: All Up!");
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i = 0;
    int ret;
    u8 data[POINT_READ_BUF] = { 0 };
    u16 high_byte, low_byte;
    u16 reg;
    char writebuf[10]= { 0 };

    ret = hyn_i2c_read(hyn_i2c_client, writebuf, 1, data, POINT_READ_BUF);
    if (ret < 0)
    {
        HYN_ERROR("[HYNITRON]: Read touchdata failed, ret: %d", ret);
        HYN_FUNC_EXIT();
        return ret;
    }

    if ((data[0] & 0x70) != 0)
        return false;
#if 0
    memcpy(pinfo, cinfo, sizeof(struct touch_info));
    memset(cinfo, 0, sizeof(struct touch_info));
    for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
        cinfo->p[i] = 1;    /* Put up */
#endif
#if 1 
    mutex_lock(&i2c_access);
    reg = 0x00;
    hyn_i2c_read(hyn_i2c_client, &reg, 1, data, 64);
    mutex_unlock(&i2c_access);    
#endif
    /*get the number of the touch points*/
    cinfo->count = data[2] & 0x0f;
    HYN_DEBUG("[HYNITRON]: Number of touch points = %d", cinfo->count);

#if (HYN_DEBUG_EN && (HYN_DEBUG_LEVEL == 2))
    hyn_show_touch_buffer(data, cinfo->count);
#endif
    for (i = 0; i < cinfo->count; i++)
    {
        cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
        cinfo->id[i] = data[3+6*i+2]>>4;                        // touch id

        /*get the X coordinate, 2 bytes*/
        high_byte = data[3 + 6 * i];
        high_byte <<= 8;
        high_byte &= 0x0F00;

        low_byte = data[3 + 6 * i + 1];
        low_byte &= 0x00FF;
        cinfo->x[i] = high_byte | low_byte;

        /*get the Y coordinate, 2 bytes*/
        high_byte = data[3 + 6 * i + 2];
        high_byte <<= 8;
        high_byte &= 0x0F00;

        low_byte = data[3 + 6 * i + 3];
        low_byte &= 0x00FF;
        cinfo->y[i] = high_byte | low_byte;

        HYN_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d", i,
                  cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
    }

    return true;
};
#else
/************************************************************************
* Name: hyn_read_touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static int hyn_read_touchdata(struct ts_event *data)
{
    u8 buf[POINT_READ_BUF] = { 0 };
    int ret = -1;
    int i = 0;
    u8 pointid = HYN_MAX_ID;

    HYN_FUNC_ENTER();
    ret = hyn_i2c_read(hyn_i2c_client, buf, 1, buf, POINT_READ_BUF);
    if (ret < 0)
    {
        HYN_ERROR("[HYNITRON]: Read touchdata failed, ret: %d", ret);
        HYN_FUNC_EXIT();
        return ret;
    }
    memset(data, 0, sizeof(struct ts_event));
    data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;
    data->touch_point = 0;


#if (HYN_DEBUG_EN && (HYN_DEBUG_LEVEL == 2))
    hyn_show_touch_buffer(buf, data->touch_point_num);
#endif

    for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
    {
        pointid = (buf[HYN_TOUCH_ID_POS + HYN_TOUCH_STEP * i]) >> 4;
        if (pointid >= HYN_MAX_ID)
            break;
        else
            data->touch_point++;
        data->au16_x[i] =
            (s16) (buf[HYN_TOUCH_X_H_POS + HYN_TOUCH_STEP * i] & 0x0F) <<
            8 | (s16) buf[HYN_TOUCH_X_L_POS + HYN_TOUCH_STEP * i];
        data->au16_y[i] =
            (s16) (buf[HYN_TOUCH_Y_H_POS + HYN_TOUCH_STEP * i] & 0x0F) <<
            8 | (s16) buf[HYN_TOUCH_Y_L_POS + HYN_TOUCH_STEP * i];
        data->au8_touch_event[i] =
            buf[HYN_TOUCH_EVENT_POS + HYN_TOUCH_STEP * i] >> 6;
        data->au8_finger_id[i] =
            (buf[HYN_TOUCH_ID_POS + HYN_TOUCH_STEP * i]) >> 4;

        data->pressure[i] =
            (buf[HYN_TOUCH_XY_POS + HYN_TOUCH_STEP * i]);//cannot constant value
        data->area[i] =
            (buf[HYN_TOUCH_MISC + HYN_TOUCH_STEP * i]) >> 4;
        if ((data->au8_touch_event[i]==0 || data->au8_touch_event[i]==2)&&(data->touch_point_num==0))
            break;

    }
    HYN_FUNC_EXIT();
    return 0;
}
/************************************************************************
* Name: hyn_report_key
* Brief: report key event
* Input: event info
* Output: no
* Return: 0: is key event, -1: isn't key event
***********************************************************************/
static int hyn_report_key(struct ts_event *data)
{
    int i = 0;

    if (1 != data->touch_point)
        return -1;

    for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
    {
        if (data->au16_y[i] <= TPD_RES_Y)
        {
            return -1;
        }
    }
    if (data->au8_touch_event[i]== 0 ||
        data->au8_touch_event[i] == 2)
    {
        tpd_button(data->au16_x[0], data->au16_y[0], 1);
        HYN_DEBUG("[HYNITRON]: Key(%d, %d) DOWN!", data->au16_x[0], data->au16_y[0]);
    }
    else
    {
        tpd_button(data->au16_x[0], data->au16_y[0], 0);
        HYN_DEBUG("[HYNITRON]: Key(%d, %d) UP!", data->au16_x[0], data->au16_y[0]);
    }

    input_sync(tpd->dev);

    return 0;
}

/************************************************************************
* Name: hyn_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int hyn_report_value(struct ts_event *data)
{
    int i = 0;
    int up_point = 0;
    int touchs = 0;

    for (i = 0; i < data->touch_point; i++)
    {
        input_mt_slot(tpd->dev, data->au8_finger_id[i]);

        if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
        {
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
#if HYN_REPORT_PRESSURE_EN
            data->pressure[i] = 0x3f;
            input_report_abs(tpd->dev, ABS_MT_PRESSURE, data->pressure[i]);
#endif
            if (data->area[i] <= 0)
            {
                HYN_ERROR("[HYNITRON]Illegal touch-major: %d", data->area[i]);
                data->area[i] = 1;
            }
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, data->area[i]);

            input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);
            touchs |= BIT(data->au8_finger_id[i]);
            data->touchs |= BIT(data->au8_finger_id[i]);

#if HYN_REPORT_PRESSURE_EN
            HYN_DEBUG("[HYNITRON]P%d(%4d,%4d)[p:%d,tm:%d] DOWN!", data->au8_finger_id[i], data->au16_x[i], data->au16_y[i],
                      data->pressure[i], data->area[i]);
#else
            HYN_DEBUG("[HYNITRON]P%d(%4d,%4d)[tm:%d] DOWN!", data->au8_finger_id[i], data->au16_x[i], data->au16_y[i],
                      data->area[i]);
#endif
        }
        else
        {
            up_point++;
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
#if HYN_REPORT_PRESSURE_EN
            input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
#endif
            data->touchs &= ~BIT(data->au8_finger_id[i]);
            HYN_DEBUG("[HYNITRON]P%d UP!", data->au8_finger_id[i]);
        }

    }
    for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
    {
        if (BIT(i) & (data->touchs ^ touchs))
        {
            HYN_DEBUG("[HYNITRON]P%d UP!", i);
            data->touchs &= ~BIT(i);
            input_mt_slot(tpd->dev, i);
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
#if HYN_REPORT_PRESSURE_EN
            input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
#endif
        }
    }
    data->touchs = touchs;

    if ((data->touch_point == up_point) || !data->touch_point_num)
    {
        HYN_DEBUG("[HYNITRON]Points All UP!");
        input_report_key(tpd->dev, BTN_TOUCH, 0);
    }
	else
    {
        input_report_key(tpd->dev, BTN_TOUCH, 1);
    }

    input_sync(tpd->dev);
    return 0;
}
#endif


/*****************************************************************************
*  Name: tpd_eint_interrupt_handler
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

/*****************************************************************************
*  Name: tpd_irq_registration
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
#if 0
static int tpd_irq_registration(struct i2c_client *client)
{
    struct device_node *node = NULL;
    int ret = 0;
    u32 ints[2] = {0,0};

    HYN_FUNC_ENTER();
    node = of_find_matching_node(node, touch_of_match);
    if (node)
    {
        of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);

        hyn_touch_irq = irq_of_parse_and_map(node, 0);
        ret = request_irq(hyn_touch_irq, tpd_eint_interrupt_handler,
                          IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
        if (ret == 0)
        {
            HYN_INFO("[HYNITRON]: IRQ request succussfully, irq=%d", hyn_touch_irq);
            client->irq = hyn_touch_irq;
        }
        else
            HYN_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
            
    }
    else
    {
        HYN_ERROR("Can not find touch eint device node!");
    }
    HYN_FUNC_EXIT();
    return 0;
}
#endif 
/*****************************************************************************
*  Name: touch_event_handler
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int touch_event_handler(void *unused)
{
    int i = 0;
    int ret;

#if HYN_MT_PROTOCOL_B_EN
    struct ts_event pevent;
#else
    struct touch_info  cinfo, pinfo;
#endif

    struct touch_info  finfo;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

    if (TPD_HAVE_BUTTON)
    {
        memset(&finfo, 0, sizeof(struct touch_info));
        for (i = 0; i < HYN_MAX_POINTS; i++)
            finfo.p[i] = 1;
    }
#if !HYN_MT_PROTOCOL_B_EN
    memset(&cinfo, 0, sizeof(struct touch_info));
    memset(&pinfo, 0, sizeof(struct touch_info));
#endif
    sched_setscheduler(current, SCHED_RR, &param);

    do
    {   mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);

        tpd_flag = 0;

        set_current_state(TASK_RUNNING);


        HYN_DEBUG("[HYNITRON]: touch_event_handler start");
#if HYN_MT_PROTOCOL_B_EN
        {
            ret = hyn_read_touchdata(&pevent);
            if (ret == 0)
            {
                if (TPD_HAVE_BUTTON)
                {
                    ret = !hyn_report_key(&pevent);
                }
                if (ret == 0)
                {
                    hyn_report_value(&pevent);
                }
            }
        }
#else //HYN_MT_PROTOCOL_A_EN
        {
            ret = tpd_touchinfo(&cinfo, &pinfo);
            if (ret)
            {
                if (TPD_HAVE_BUTTON)
                {
                    if (cinfo.p[0] == 0)
                        memcpy(&finfo, &cinfo, sizeof(struct touch_info));
                }

                if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
                    && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
                {
                    HYN_DEBUG("All up");
                    tpd_up(pinfo.x[0], pinfo.y[0]);
                    input_sync(tpd->dev);
                    continue;
                }

                if (TPD_HAVE_BUTTON)
                {
                    if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
                        && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
                    {
                        HYN_DEBUG("All key up");
                        tpd_button(pinfo.x[0], pinfo.y[0], 0);
                        input_sync(tpd->dev);
                        continue;
                    }

                    if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y))
                    {
                        if (finfo.y[0] > TPD_RES_Y)
                        {
                            if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2))
                            {
                                HYN_DEBUG("Key(%d,%d) Down", pinfo.x[0], pinfo.y[0]);
                                tpd_button(pinfo.x[0], pinfo.y[0], 1);
                            }
                            else if ((cinfo.p[0] == 1) &&
                                     ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
                            {
                                HYN_DEBUG("Key(%d,%d) Up!", pinfo.x[0], pinfo.y[0]);
                                tpd_button(pinfo.x[0], pinfo.y[0], 0);
                            }
                            input_sync(tpd->dev);
                        }
                        continue;
                    }
                }
                if (cinfo.count > 0)
                {
                    for (i = 0; i < cinfo.count; i++)
                        tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
                }
                else
                {
                    tpd_up(cinfo.x[0], cinfo.y[0]);
                }
                input_sync(tpd->dev);

            }
        }
#endif
    }
    while (!kthread_should_stop());

    return 0;
}

/************************************************************************
* Name: tpd_i2c_detect
* Brief:
* Input:
* Output:
* Return:
***********************************************************************/
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);

    return 0;
}


#if I2C_EXTENSION_FOCAL

int hyn_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=0;

	// for DMA I2c transfer

	mutex_lock(&i2c_rw_access);

	if((NULL!=client) && (writelen>0) && (writelen<=128))
	{
		// DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
			//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
			HYN_ERROR("i2c write failed\n");
		client->addr = (client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}

	// DMA Read

	if((NULL!=client) && (readlen>0) && (readlen<=128))

	{
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);

		memcpy(readbuf, g_dma_buff_va, readlen);

		client->addr = (client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}
int hyn_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int i = 0;
	int ret = 0;

	if (writelen <= 8) {
	    client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		return i2c_master_send(client, writebuf, writelen);
	}
	else if((writelen > 8)&&(NULL != tpd_i2c_dma_va))
	{
		for (i = 0; i < writelen; i++)
			tpd_i2c_dma_va[i] = writebuf[i];

		client->addr = (client->addr & I2C_MASK_FLAG )| I2C_DMA_FLAG;
	    //client->ext_flag = client->ext_flag | I2C_DMA_FLAG;
	    ret = i2c_master_send(client, (unsigned char *)tpd_i2c_dma_pa, writelen);
	    client->addr = client->addr & I2C_MASK_FLAG & ~I2C_DMA_FLAG;
		//ret = i2c_master_send(client, (u8 *)(uintptr_t)tpd_i2c_dma_pa, writelen);
	    //client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		return ret;
	}
	return 1;
}

#else

int hyn_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
    int ret = 0;
    int i = 0, readlenth = 0;

    if (client == NULL)
    {
        HYN_ERROR("[IIC][%s]i2c_client==NULL!", __func__);
        return -1;
    }
#if 1
    mutex_lock(&i2c_rw_access);
    if (readlen > 0)
    {
        if (writelen > 0)
        {
            struct i2c_msg msgs[] =
            {
                {
                    .addr = client->addr,
                    .flags = 0,
                    .len = writelen,
                    .buf = writebuf,
                },
                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = readlen,
                    .buf = readbuf,
                },
            };

            ret = i2c_transfer(client->adapter, msgs, 2);
            HYN_INFO("[HYNITRON]: %s",__func__);
            HYN_INFO("[IIC]: writelen is %d\n", writelen);
            HYN_INFO("[IIC]: writelen is %d\n", readlen);
            readlenth = readlen;
            for (i = 0; i < readlenth; i++)
                HYN_INFO("[IIC]: readbuf is %c", readbuf[i]);
            HYN_INFO("[IIC]: client->addr read is 0x%02x", client->addr);
            if (ret < 0)
            {   
                HYN_ERROR("[IIC]: i2c_transfer(read) error, ret=%d!!", ret);
            }
        }
        else
        {
            struct i2c_msg msgs[] =
            {
                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = readlen,
                    .buf = readbuf,
                },
            };
            ret = i2c_transfer(client->adapter, msgs, 1);
            HYN_INFO("[HYNITRON]: %s",__func__);
            HYN_INFO("[IIC]: client->addr read is 0x%02x", client->addr);
            if (ret < 0)
            {
                HYN_ERROR("[IIC]: [else] i2c_transfer(read) error, ret=%d!!", ret);
            }
        }
    }

    mutex_unlock(&i2c_rw_access);
#endif
#if 0    
    readbuf[0] = client->addr;
    client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(client, &readbuf[0], (readlen << 8 | 1));
    HYN_INFO("[HYNITRON]: %s",__func__);
    HYN_INFO("[IIC]: client->addr read is 0x%02x, 0x%02x", client->addr,readbuf[0]);
    HYN_INFO("[IIC]: i2c_transfer ret=%d\n", ret);
#endif
    return ret;
}

/*
 *
 */
int hyn_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
    int ret = 0;

    if (client == NULL)
    {
        HYN_ERROR("[IIC][%s]i2c_client==NULL!", __func__);
        return -1;
    }

    mutex_lock(&i2c_rw_access);
    if (writelen > 0)
    {
        struct i2c_msg msgs[] =
        {
            {
                .addr = client->addr,
                .flags = 0,
                .len = writelen,
                .buf = writebuf,
            },
        };
        ret = i2c_transfer(client->adapter, msgs, 1);
        HYN_INFO("[HYNITRON]: %s",__func__);
        HYN_INFO("[IIC]: client->addr write is 0x%02x", client->addr);
        if (ret < 0)
        {
            HYN_ERROR("[IIC]: i2c_transfer(write) error, ret=%d!!", ret);
        }
    }
    mutex_unlock(&i2c_rw_access);

    return ret;
}
#endif
/*
 *
 */
int hyn_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
    u8 buf[2] = {0};

    buf[0] = regaddr;
    buf[1] = regvalue;
    return hyn_i2c_write(client, buf, sizeof(buf));
}
/*
 *
 */
int hyn_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
    return hyn_i2c_read(client, &regaddr, 1, regvalue, 1);
}

/*
 *
 */
int hyn_i2c_init(void)
{
    HYN_FUNC_ENTER();

    HYN_FUNC_EXIT();
    return 0;
}
/*
 *
 */
int hyn_i2c_exit(void)
{
    HYN_FUNC_ENTER();
    HYN_FUNC_EXIT();
    return 0;
}




/************************************************************************
* Name: hyn_probe
* Brief:
* Input:
* Output:
* Return:
***********************************************************************/
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = 0;
    int slave_addr = 0;
    int reset_count = 0;
    char data;
    int err = 0;
    unsigned char uc_reg_value;
    unsigned char uc_reg_addr;
    HYN_FUNC_ENTER();

    //hyn_i2c_client = client;
    //hyn_input_dev = tpd->dev;
#if 0
    if (hyn_i2c_client->addr != HYN_I2C_SLAVE_ADDR)
    {   
        hyn_i2c_init();   
        slave_addr = HYN_I2C_SLAVE_ADDR;
        HYN_INFO("[HYNITRON]Change i2c addr 0x%02x to 0x%02x", hyn_i2c_client->addr,slave_addr);
        hyn_i2c_client->addr = HYN_I2C_SLAVE_ADDR;
        HYN_DEBUG("[HYNITRON]hyn_i2c_client->addr=0x%x\n", hyn_i2c_client->addr);
    }

    of_get_hyn_platform_data(&client->dev);
#endif
    //spin_lock_init(&irq_lock);
    //mutex_init(&report_mutex);
    /* Configure gpio to irq and request irq */

#if HYN_MT_PROTOCOL_B_EN
    input_mt_init_slots(tpd->dev, CFG_MAX_TOUCH_POINTS, INPUT_MT_DIRECT);
#endif
#if 0
    retval = regulator_enable(tpd->reg);
    if (retval != 0) {
	HYN_DEBUG("[HYNITRON]: Failed to enable reg-vgp6: %d\n", retval);
    }
#endif
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#if 0
    tpd_gpio_as_int(tpd_int_gpio_number);
#endif
    msg_dma_alloct();
reset_proc:
	/* Reset CTP */
#if 0
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
#endif
        hyn_i2c_client = client;
        hyn_input_dev = tpd->dev;
        //HYN_DEBUG("[HYNITRON]: hyn_i2c_client: %s, hyn_input_dev: %s\n", hyn_i2c_client, hyn_input_dev);
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);  
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);  
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);   
        msleep(20);  
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);  
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);  
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);     
        msleep(400);

        err = hyn_i2c_read_reg(client, 0x00, &data);
        HYN_ERROR("[HYNITRON]: i2c err=%d, data=%d!", err,data);
	if(err< 0 || data!=0)// reg0 data running state is 0; other state is not 0
	{
                HYN_ERROR("[HYNITRON]: I2C transfer error, func: %s, line: %d\n",__func__,__LINE__);
		if ( ++reset_count < 3)
		{
			goto reset_proc;
		}
#if 0
                retval	= regulator_disable(tpd->reg); //disable regulator
		if(retval)
		{
			HYN_ERROR("[HYNITRON]: tpd_probe regulator_disable() failed!\n");
		}

		regulator_put(tpd->reg);
#endif
                hwPowerDown(TPD_POWER_SOURCE_CUSTOM,"TP");
                msg_dma_release();
                
		return -1; 
    }
    
    uc_reg_addr = HYN_REG_CHIP_ID;
    hyn_i2c_write(hyn_i2c_client, &uc_reg_addr, 1);
    retval = hyn_i2c_read(hyn_i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
    HYN_INFO("[HYNITRON]: chip id is %d.\n",uc_reg_value);
    if(retval < 0)
    {
       	HYN_ERROR("[HYNITRON]: Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
	return -1;
    }

    tpd_load_status = 1;
#if 0
    tpd_gpio_as_int(tpd_int_gpio_number);
#endif
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN,GPIO_CTP_EINT_PIN_M_EINT);  //设置引脚中断  
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN,GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN,GPIO_PULL_ENABLE); //打开中断使能  
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN,GPIO_PULL_UP);  //设定中断触发方式 
    msleep(100);
#if 0
    tpd_irq_registration(client);
#endif 
    mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM,EINTF_TRIGGER_RISING,tpd_eint_interrupt_handler, 0);
#if 0
    hyn_irq_enable();
#endif
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
#if HYN_SYSFS_NODE_EN
    hyn_create_sysfs(client);
#endif


#if HYN_APK_NODE_EN
    hyn_create_apk_debug_channel(client);
#endif

    thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread_tpd))
    {
        retval = PTR_ERR(thread_tpd);
        HYN_ERROR("[HYNITRON]: Failed to create kernel thread_tpd,ret=%d!", retval);
    }

    HYN_DEBUG("[HYNITRON]: Touch Panel Device Probe %s!", (retval < 0) ? "FAIL" : "PASS");

#if HYN_EN_AUTO_UPDATE
    ctp_hynitron_update(client);
#endif
#if 0
    hyn_irq_enable(); /* need execute before upgrade */
#endif
    //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    HYN_INFO("[HYNITRON]: %s",__func__);
    HYN_FUNC_EXIT();

    return 0;
}

/*****************************************************************************
*  Name: tpd_remove
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int tpd_remove(struct i2c_client *client)
{
    HYN_FUNC_ENTER();

    msg_dma_release();

#if HYN_SYSFS_NODE_EN
    hyn_remove_sysfs(client);
#endif


#if HYN_APK_NODE_EN
    hyn_release_apk_debug_channel();
#endif

    hyn_i2c_exit();
    HYN_INFO("[HYNITRON]: %s",__func__);
    HYN_FUNC_EXIT();

    return 0;
}

/*****************************************************************************
*  Name: tpd_local_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int tpd_local_init(void)
{
    //int ret;

    HYN_FUNC_ENTER();
#if 0
    tpd->reg=regulator_get(tpd->tpd_dev, "vtouch"); // get pointer to regulator structure
		if (IS_ERR(tpd->reg)) {
			HYN_ERROR("[HYNITRON]: regulator_get() failed!\n");
		}

		ret=regulator_set_voltage(tpd->reg, 2800000, 2800000);	// set 2.8v
		if (ret)
			HYN_ERROR("[HYNITRON]: regulator_set_voltage() failed!\n");
		ret=regulator_enable(tpd->reg);  //enable regulator
		if (ret)
			HYN_ERROR("[HYNITRON]: regulator_enable() failed!\n");
#endif
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
    if(tpd_load_status==1)
    {
	return -1;
    }
                
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        HYN_ERROR("[HYNITRON]: Unable to add hyn i2c driver!!");
        HYN_FUNC_EXIT();
        return -1;
    }
    if(tpd_load_status == 0) 
    {
       	HYN_ERROR("[HYNITRON]: add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }

    if (TPD_HAVE_BUTTON)
    {
#if 0
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
                           tpd_dts_data.tpd_key_dim_local);
#endif
        tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
    }

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

    memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
#endif

    tpd_type_cap = 1;
    HYN_INFO("[HYNITRON]: %s",__func__);
    HYN_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: tpd_suspend
*  Brief: When suspend, will call this function
*           1. Set gesture if EN
*           2. Disable ESD if EN
*           3. Process MTK sensor hub if configure, default n, if n, execute 4/5/6
*           4. disable irq
*           5. Set TP to sleep mode
*           6. Disable power(regulator) if EN
*           7. hyn_release_all_finger
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void tpd_suspend(struct device *h)
{
    int retval = 0;

    HYN_FUNC_ENTER();


    hyn_release_all_finger();


    //disable_irq(hyn_touch_irq);
#if 0
    hyn_irq_disable();
#endif
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    /* TP enter sleep mode */
    retval = hyn_i2c_write_reg(hyn_i2c_client, HYN_REG_POWER_MODE, HYN_REG_POWER_MODE_SLEEP_VALUE);
    if (retval < 0)
    {
        HYN_ERROR("[HYNITRON]: Set TP to sleep mode fail, ret=%d\n", retval);
    }
#if 0
    retval = regulator_disable(tpd->reg);
    if (retval != 0)
        HYN_ERROR("[HYNITRON]: Failed to disable regulator when suspend ret=%d\n", retval);
#endif
    hwPowerDown(TPD_POWER_SOURCE_CUSTOM,"TP");
    HYN_INFO("[HYNITRON]: %s",__func__);
    HYN_FUNC_EXIT();
}


/*****************************************************************************
*  Name: tpd_resume
*  Brief: When suspend, will call this function
*           1. Clear gesture if EN
*           2. Enable power(regulator) if EN
*           3. Execute reset if no IDC to wake up
*           4. Confirm TP in valid app by read chip ip register:0xA3
*           5. hyn_release_all_finger
*           6. Enable ESD if EN
*           7. tpd_usb_plugin if EN
*           8. Process MTK sensor hub if configure, default n, if n, execute 9
*           9. disable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void tpd_resume(struct device *h)
{
    //int retval = 0;

    HYN_FUNC_ENTER();
#if 0
    retval = regulator_enable(tpd->reg);
    if (retval != 0)
       HYN_ERROR("[HYNITRON]: Failed to disable regulator when resume ret=%d\n", retval);
#endif
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");

    hyn_reset_proc(200);
#if 0 
    hyn_irq_enable();
#endif
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}

/*****************************************************************************
*  TPD Device Driver
*****************************************************************************/

struct device_attribute *hyn_attrs[] =
{
};

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = HYN_DRIVER_NAME,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
    .attrs = {
        .attr = hyn_attrs,
        .num  = ARRAY_SIZE(hyn_attrs),
    },
};

/*****************************************************************************
*  Name: tpd_driver_init
*  Brief: 1. Get dts information
*         2. call tpd_driver_add to add tpd_device_driver
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int __init tpd_driver_init(void)
{
    HYN_FUNC_ENTER();
    HYN_INFO("[HYNITRON]: Driver version: %s", HYN_DRIVER_VERSION);
#if 0
    //tpd_get_dts_info();
#endif
    i2c_register_board_info(1, &hyn_i2c_tpd, 1);

    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        HYN_ERROR("[HYNITRON]: Add HYN Touch driver failed!!");
    }

    HYN_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: tpd_driver_exit
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void __exit tpd_driver_exit(void)
{
    HYN_FUNC_ENTER();
    tpd_driver_remove(&tpd_device_driver);
    HYN_FUNC_EXIT();
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


MODULE_AUTHOR("Hynitron Driver Team");
MODULE_DESCRIPTION("Hynitron Touchscreen Driver for Mediatek");
MODULE_LICENSE("GPL v2");

