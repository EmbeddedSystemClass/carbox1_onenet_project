#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "minmea.h"
#include "gnss.h"
#include "Led.h"

#define UART2_TXD  (UART_PIN_NO_CHANGE)
#define UART2_RXD  (GPIO_NUM_27)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE    1024

static char tag[] = "GNSS";


void GNSS_READ_task(void* arg)
{
	char data_u2[BUF_SIZE];
	char rmc[256];
	int rmc_len=0;
	while(1) 
	{
		
		int u2_len = uart_read_bytes(UART_NUM_2, (uint8_t *)data_u2, BUF_SIZE, 80 / portTICK_RATE_MS);
		if(u2_len>0)
		{
			u2_len=0;
			//ESP_LOGI(tag, "u2=%s", data_u2);
			char* rmc_start=strstr(data_u2,"$GNRMC");
			char* rmc_end=strchr(data_u2,'\n');
			if((rmc_start!=NULL)&&(rmc_end!=NULL))
			{
				rmc_len=rmc_end-rmc_start;
				strncpy(rmc,rmc_start,rmc_len-1);
			}
			//printf("rmc_len=%d\n",rmc_len);			
			//printf("rmc=%s\n",rmc);
			switch(minmea_sentence_id(rmc, false)) 
			{
				case MINMEA_SENTENCE_RMC:
					//ESP_LOGI(tag, "Sentence - MINMEA_SENTENCE_RMC");
					;
					struct minmea_sentence_rmc frame;
					if (minmea_parse_rmc(&frame, data_u2)) 
					{
						/*ESP_LOGI(tag, "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d",
										frame.latitude.value, frame.latitude.scale,
										frame.longitude.value, frame.longitude.scale,
										frame.speed.value, frame.speed.scale);
						ESP_LOGI(tag, "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d",
										minmea_rescale(&frame.latitude, 1000),
										minmea_rescale(&frame.longitude, 1000),
										minmea_rescale(&frame.speed, 1000));*/
						latitude=minmea_tocoord(&frame.latitude);
						longitude=minmea_tocoord(&frame.longitude);
						speed=minmea_tofloat(&frame.speed);
						speed=speed*1.852;
						if(latitude>0)
						{
							Led_R_On();
							vTaskDelay(200 / portTICK_RATE_MS);
							Led_R_Off();
						}

						
						//ESP_LOGI(tag, "latitude=%f,longitude=%f,speed=%f",latitude,longitude,speed);
					}
					else 
					{
						ESP_LOGI(tag, "$xxRMC sentence is not parsed\n");
					}
				break;

				case MINMEA_SENTENCE_GGA:
					ESP_LOGI(tag, "Sentence - MINMEA_SENTENCE_GGA");
				break;

				case MINMEA_SENTENCE_GSV:
					ESP_LOGI(tag, "Sentence - MINMEA_SENTENCE_GSV");
				break;

				default:
					ESP_LOGI(tag, "Sentence - other");
				break;
			}
			bzero(data_u2,sizeof(data_u2));   
			bzero(rmc,sizeof(rmc));   
		}
		vTaskDelay(5 / portTICK_RATE_MS);
	}
} 


void GNSS_init(void)
{
     //配置GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1 << UART2_RXD;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    
    uart_config_t uart_config = 
	{
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
	xTaskCreate(&GNSS_READ_task, "GNSS_READ_task", 8192, NULL, 10, NULL);

}