#include <stdio.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "Smartconfig.h"
#include "Http.h"
#include "Nvs.h"
#include "Mqtt.h"
#include "Json_parse.h"
#include "Uart0.h"

#include "Led.h"
#include "E2prom.h"
#include "RtcUsr.h"

#include "Key.h"
#include "Beep.h"
#include "sht31.h"
#include "libGSM.h"
#include "gnss.h"


extern const int PPP_CONNECTED_BIT;

static int active_status=0;

void timer_periodic_cb(void *arg); 
esp_timer_handle_t timer_periodic_handle = 0; //定时器句柄


esp_timer_create_args_t timer_periodic_arg = {
    .callback =
        &timer_periodic_cb, 
    .arg = NULL,            
    .name = "PeriodicTimer" 
};

void timer_periodic_cb(void *arg) //1ms中断一次
{
  static int64_t timer_count = 0;
  timer_count++;
  if (timer_count >= 3000) //1s
  {
    timer_count = 0;
    printf("[APP] Free memory: %d bytes\n", esp_get_free_heap_size());

  }
}

static void Uart0_Task(void* arg)
{
    while(1)
    {
        Uart0_read();
        vTaskDelay(10 / portTICK_RATE_MS);
    }  
}

static void http_activate_Task(void* arg)
{
    if(http_activate()==0)//注册失败
    {
      active_status=0;
    }
    else
    {
      //激活成功
      E2prom_Read(DEVICEID_ADDR,(uint8_t *)DeviceId,DEVICEID_LEN);
      printf("DeviceId=%s\n", DeviceId);
      active_status=1;
    }
    vTaskDelete(NULL);
}



void app_main(void)
{
  
  //ESP_ERROR_CHECK( nvs_flash_init() );
  ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
  Led_Init();
  i2c_init();
  Uart0_Init();
  key_Init();
  Beep_Init();
  ppposInit(); 
  GNSS_init();

  xTaskCreate(Uart0_Task, "Uart0_Task", 4096, NULL, 10, NULL);

  /*step1 判断是否有Serial_No/product id/RegisterCode****/

  E2prom_Read(SERIALNUM_ADDR,(uint8_t *)SerialNum,SERIALNUM_LEN);
  printf("SerialNum=%s\n", SerialNum);

  E2prom_Read(PRODUCTID_ADDR,(uint8_t *)ProductId,PRODUCTID_LEN);
  printf("ProductId=%s\n", ProductId);

  E2prom_Read(REGISTERCODE_ADDR,(uint8_t *)RegisterCode,REGISTERCODE_LEN);
  printf("RegisterCode=%s\n", RegisterCode); 

  if((strlen(SerialNum)==0)||(strlen(ProductId)==0)||(strlen(RegisterCode)==0)) //未获取到序列号或productid，未烧写序列号
  {
    printf("no SerialNum or productid or RegisterCode!\n");
    while(1)
    {
      //故障灯闪烁
      vTaskDelay(500 / portTICK_RATE_MS);
    }
  }

  //阻塞等待ppp连接
  xEventGroupWaitBits(ppp_event_group, PPP_CONNECTED_BIT , false, true, portMAX_DELAY); 


  //模拟清空DeviceId，激活后获取
  //uint8_t data_write_0[DEVICEID_LEN]="\0";
  //E2prom_Write(DEVICEID_ADDR, data_write_0, DEVICEID_LEN);

  /*step3 判断是否有DeviceId****/
  
  E2prom_Read(DEVICEID_ADDR,(uint8_t *)DeviceId,DEVICEID_LEN);
  printf("DeviceId=%s\n", DeviceId);

  if(strlen(DeviceId)==0)//未获取到DeviceId进行激活流程
  {
    printf("no DeviceId!\n");

    xTaskCreate(http_activate_Task, "http_activate_Task", 4096, NULL, 10, NULL);
    while(active_status==0)
    {
      vTaskDelay(10000 / portTICK_RATE_MS);
    }
  } 

  /*******************************timer 1s init**********************************************/
  esp_err_t err = esp_timer_create(&timer_periodic_arg, &timer_periodic_handle);
  err = esp_timer_start_periodic(timer_periodic_handle, 1000); //创建定时器，单位us，定时1ms
  if (err != ESP_OK)
  {
    printf("timer periodic create err code:%d\n", err);
  }

  initialise_mqtt();

}
