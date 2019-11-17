#ifndef __CST0XX_EX_FUN_H__
#define __CST0XX_EX_FUN_H__

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>


/*
 *
 */
//int cst0xx_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
/*
 *
 */
//int cst0xx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);
/*
 *
 */
int ctp_hynitron_update(struct i2c_client *mclient);

#endif

