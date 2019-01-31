#ifndef _E2PROM_H_
#define _E2PROM_H_

#include "freertos/FreeRTOS.h"

  /*
  EEPROM PAGE0
    0x00 DeviceId       (16byte)  -------注册获得，鉴权使用
    0x10 SerialNum       (16byte)   --------烧录获得，鉴权使用
    0x20 RegisterCode    (16byte)  -------烧录获得
    0x30 ProductId      (32byte)   ---------烧录获得，鉴权使用
  EEPROM PAGE1
    0X00 none
  */
#define DEVICEID_ADDR         0x00
#define SERIALNUM_ADDR        0x10
#define REGISTERCODE_ADDR     0x20
#define PRODUCTID_ADDR        0x30


#define DEVICEID_LEN          16
#define SERIALNUM_LEN         16
#define REGISTERCODE_LEN      16
#define PRODUCTID_LEN         32


char DeviceId[DEVICEID_LEN+1];
char SerialNum[SERIALNUM_LEN+1];
char RegisterCode[REGISTERCODE_LEN+1];
char ProductId[PRODUCTID_LEN+1];



extern void E2prom_Init(void);
extern int E2prom_Write(uint8_t addr,uint8_t*data_write,int len);
extern int E2prom_Read(uint8_t addr,uint8_t*data_read,int len);
extern int E2prom_BluWrite(uint8_t addr,uint8_t*data_write,int len);
extern int E2prom_BluRead(uint8_t addr,uint8_t*data_read,int len);
#endif

