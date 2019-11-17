/**
 *Name        : cst0xx_ex_fun.c
 *Author      : gary
 *Version     : V1.0
 *Create      : 2018-1-23
 *Copyright   : zxzz
 */

#include <linux/netdevice.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

//#include "cst0xx.h"
#include "cst0xx_ex_fun.h"
#include "cst0xx_config.h"
//#include "cst0xx_ctl.h"

extern int hyn_reset_proc(int hdelayms);
extern int hyn_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int hyn_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
#define cst0xx_i2c_Read     hyn_i2c_read
#define cst0xx_i2c_Write    hyn_i2c_write
#define cst0xx_reset        hyn_reset_proc

#if HYN_EN_AUTO_UPDATE
static unsigned char dev_addr;
static struct i2c_client *client_up;
/*****************************************************************/
/*
 *
 */
int hctp_write_bytes(unsigned short reg,unsigned char *buf,unsigned short len,unsigned char reg_len){
	int ret;
    unsigned char mbuf[600];
    if (reg_len == 1){
        mbuf[0] = reg;
        memcpy(mbuf+1,buf,len);
    }else{
        mbuf[0] = reg>>8;
        mbuf[1] = reg;
        memcpy(mbuf+2,buf,len);    
    }

    ret = cst0xx_i2c_Write(client_up,mbuf,len+reg_len);
	if (ret < 0){
		dev_err(&client_up->dev, "%s i2c write error.\n", __func__);
	}
    return ret;
}
/*
 *
 */
int hctp_read_bytes(unsigned short reg,unsigned char* buf,unsigned short len,unsigned char reg_len){
	int ret;
    unsigned char reg_buf[2];
    if (reg_len == 1){
        reg_buf[0] = reg;
    }else{
        reg_buf[0] = reg>>8;
        reg_buf[1] = reg;
    }
    ret = cst0xx_i2c_Read(client_up,reg_buf,reg_len,buf,len);
	if (ret < 0){
		dev_err(&client_up->dev, "f%s: i2c read error.\n",__func__);
	}
    return ret;
}
#if HYN_EN_AUTO_UPDATE_CST0xxSE
/*****************************************************************/
// For CST0xxSE update

#define CST0xxSE_REG_LEN    2
 /*
  *
  */
static int cst0xxse_enter_bootmode(void){
     char retryCnt = 10;

     cst0xx_reset();
     mdelay(5);
     while(retryCnt--){
         unsigned char cmd[3];
         cmd[0] = 0xAA;
         if (-1 == hctp_write_bytes(0xA001,cmd,1,CST0xxSE_REG_LEN)){  // enter program mode
             mdelay(2); // 4ms
             continue;                   
         }
         if (-1 == hctp_read_bytes(0xA003,cmd,1,CST0xxSE_REG_LEN)) { // read flag
             mdelay(2); // 4ms
             continue;                           
         }else{
             if (cmd[0] != 0x55){
                 msleep(2); // 4ms
                 continue;
             }else{
                 return 0;
             }
         }
     }
     return -1;
 }
 /*
  *
  */
static void cst0xxse_update(unsigned short startAddr,unsigned short len,unsigned char* src){
     unsigned short sum_len;
     unsigned char cmd[10];
 
     sum_len = 0;
 
#define PER_LEN	512
     do{
         if (sum_len >= len){
             return;
         }
         
         // send address
         cmd[0] = startAddr&0xFF;
         cmd[1] = startAddr>>8;
         hctp_write_bytes(0xA014,cmd,2,CST0xxSE_REG_LEN);
         
         hctp_write_bytes(0xA018,src,PER_LEN,CST0xxSE_REG_LEN);
 
         cmd[0] = 0xEE;
         hctp_write_bytes(0xA004,cmd,1,CST0xxSE_REG_LEN);
         msleep(500);
 
         {
             unsigned char retrycnt = 50;
             while(retrycnt--){
                 cmd[0] = 0;
                 hctp_read_bytes(0xA005,cmd,1,CST0xxSE_REG_LEN);
                 if (cmd[0] == 0x55){
                     // success
                     break;
                 }
                 msleep(10);
             }
         }
         startAddr += PER_LEN;
         src += PER_LEN;
         sum_len += PER_LEN;
     }while(len);
     
     // exit program mode
     cmd[0] = 0x00;
     hctp_write_bytes(0xA003,cmd,1,CST0xxSE_REG_LEN);
 }
 /*
  *
  */
static unsigned long cst0xxse_read_checksum(unsigned short startAddr,unsigned short len){
     union{
         unsigned long sum;
         unsigned char buf[4];
     }checksum;
     char readback[4] = {0};
 
     if (-1 == hctp_read_bytes(0xA000,readback,1,CST0xxSE_REG_LEN)){
         return -1;   
     }
     if (readback[0] != 1){
         return -1;
     }
     if (-1 == hctp_read_bytes(0xA008,checksum.buf,4,CST0xxSE_REG_LEN)){
         return -1;
     }
     return checksum.sum;
 }
 #endif
 
 /*****************************************************************/
 // For CST0xx update
#if HYN_EN_AUTO_UPDATE_CST0xx
#define CST0xx_REG_LEN  1
 /*
 ** Parameters: void
 ** Return Values: 0: enter cst0xx boot mode success -1:enter cst0xx boot mode fail
 */
static int cst0xx_enter_bootmode(void){
     char retryCnt;
     
     cst0xx_reset(0);
     for(retryCnt = 10; retryCnt > 0; retryCnt--){
         char enterBootData[] = {'0', '1', '2', '3'};
         char readBackData[4] = {0, 0, 0, 0};
         
         mdelay(2);
         hctp_write_bytes(0x00, enterBootData, sizeof(enterBootData),CST0xx_REG_LEN);
         mdelay(1);
         hctp_read_bytes(0x00, readBackData, 4,CST0xx_REG_LEN);
         
         if(memcmp(readBackData, "3210", 4) == 0){
             return 0;
         }
     }
     return -1;
 }
 /*
  *
  */
static void cst0xx_update(unsigned short startAddr, unsigned short length, unsigned char *data){
     const unsigned char perLen = 128;
     unsigned char curLen;
     
     unsigned char cmdReg;
     unsigned char cmdData[8];
     
     do{
         cmdReg = 0xF0;
         cmdData[0] = 0x96;
         cmdData[1] = startAddr/128;
         hctp_write_bytes(cmdReg, cmdData, 2,CST0xx_REG_LEN);
         msleep(30);
         hctp_read_bytes(cmdReg, cmdData, 2,CST0xx_REG_LEN);
         
         curLen = perLen < length ? perLen : length;
         hctp_write_bytes(0x00, data, curLen,CST0xx_REG_LEN);
         
         cmdReg = 0xF0;
         cmdData[0] = 0xE1;
         cmdData[1] = 0x80|(curLen-1);
         cmdData[2] = startAddr;
         cmdData[3] = startAddr >> 8;
         cmdData[4] = 0;
         cmdData[5] = 0;
         hctp_write_bytes(cmdReg, cmdData, 6,CST0xx_REG_LEN);
         msleep(30);
         hctp_read_bytes(cmdReg, cmdData, 2,CST0xx_REG_LEN);
         
         startAddr += curLen;
         length -= curLen;
         data += curLen;
         
     }while(length > 0);
 }
 /*
  *
  */
static unsigned long cst0xx_read_checksum(unsigned short startAddr,unsigned short length)
 {
     unsigned short checksum;
     unsigned char cmdReg;
     unsigned char cmdData[8];
     
     cmdReg = 0xF0;
     cmdData[0] = 0xD2;
     cmdData[1] = length/128-1;
     cmdData[2] = startAddr;
     cmdData[3] = startAddr >> 8;
     hctp_write_bytes(cmdReg, cmdData, 4,CST0xx_REG_LEN);
     msleep(200);
     hctp_read_bytes(cmdReg, cmdData, 4,CST0xx_REG_LEN);
     
     checksum = cmdData[3];
     checksum <<= 8;
     checksum |= cmdData[2];
     return checksum;
 }
 #endif
/*****************************************************************/
// common

 /*
  *
  */
 int ctp_hynitron_update(struct i2c_client *mclient){
     
     client_up = mclient;
     dev_addr = client_up->addr;
     
#if HYN_EN_AUTO_UPDATE_CST0xxSE
     client_up->addr = 0x5A;

     if (cst0xxse_enter_bootmode() == 0){ //CST0xxSE
#include "capacitive_hynitron_cst0xxse_update.h"
         if(sizeof(app_bin) > 10){
             unsigned short startAddr = app_bin[1];
             unsigned short len = app_bin[3];
             unsigned long fileCheck = app_bin[7];
             startAddr = ((startAddr<<8)|app_bin[0]);
             len = (len<<8)|app_bin[2];
             fileCheck = (fileCheck<<8)|app_bin[6];
             fileCheck = (fileCheck<<8)|app_bin[5];
             fileCheck = (fileCheck<<8)|app_bin[4];
             if (fileCheck != cst0xxse_read_checksum(startAddr,len))
             {
                 cst0xxse_update(startAddr,len,app_bin+8);
                 cst0xxse_read_checksum(startAddr,len);
             }
         }
     }
#else
     if (0){
     }
#endif
     else
     { //CST0xx
#if HYN_EN_AUTO_UPDATE_CST0xx
#include "capacitive_hynitron_cst0xx_update.h"
         client_up->addr = 0x15;
         if(sizeof(app_bin) > 10){
             unsigned short startAddr = app_bin[1];
             unsigned short length = app_bin[3];
             unsigned short checksum = app_bin[5];
             startAddr <<= 8; startAddr |= app_bin[0];
             startAddr -= 0x1000;
             length <<= 8; length |= app_bin[2];
             checksum <<= 8; checksum |= app_bin[4];
	      printk("\r\n CTP File, start-0x%04x len-0x%04x fileCheck-0x%04x\r\n",startAddr,length,checksum);
             if (0 == cst0xx_enter_bootmode()){
                 printk("\r\nCTP enter boot mode success...");
             }else{
                 printk("\r\nCTP enter boot mode fail...");
	      }
             if(cst0xx_read_checksum(startAddr, length) != checksum){
                 cst0xx_update(startAddr, length, app_bin+6);
                 length = cst0xx_read_checksum(startAddr, length);
	          printk("\r\nCTP update %s, checksum-0x%04x",((length==checksum) ? "success" : "fail"),length);
             }else{
	          printk("\r\nCTP check pass...");
             	}
         }
#endif         
     }
     client_up->addr = dev_addr;
     cst0xx_reset(0);
     return 0;
 }

#endif  //CTP_HYNITRON_EXT==1

