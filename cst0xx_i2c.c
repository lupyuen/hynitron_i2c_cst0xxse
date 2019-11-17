/**
 *Name        : cst0xx_i2c.c
 *Author      : gary
 *Version     : V1.0
 *Create      : 2018-1-23
 *Copyright   : zxzz
 */


#include "cst0xx_core.h"

/*****************************************************************************/
#define CONFIG_MTK_I2C_EXTENSION_FOCAL

u8 *g_dma_buff_va = NULL;
dma_addr_t g_dma_buff_pa = 0;

static DEFINE_MUTEX(i2c_rw_access);

/*
 *
 */
#ifdef CONFIG_MTK_I2C_EXTENSION_FOCAL

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
                HYN_INFO("[IIC]: readbuf is %s", readbuf[i]);
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

