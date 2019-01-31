#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cJSON.h>
#include "esp_system.h"
#include "Json_parse.h"
#include "Nvs.h"
#include "ServerTimer.h"
#include "Http.h"

#include "esp_wifi.h"
#include "Smartconfig.h"
#include "E2prom.h"
#include "Http.h"
#include "Beep.h"
#include "sht31.h"
#include "gnss.h"




//解析激活返回数据
esp_err_t parse_objects_http_active(char *http_json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_value = NULL;
    cJSON *json_data_parse_data = NULL;

    cJSON *json_data_parse_data_deviceid = NULL;

    //char *json_print;

    printf("start_parse_active_http_json\r\n");
    
    if(http_json_data[0]!='{')
    {
        printf("http_json_data Json Formatting error\n");

        return 0;       
    }

    json_data_parse = cJSON_Parse(http_json_data); 
    if (json_data_parse == NULL)                  
    {

        printf("Json Formatting error3\n");

        cJSON_Delete(json_data_parse);
        return 0;
    }
    else
    {

        json_data_parse_value = cJSON_GetObjectItem(json_data_parse, "error"); 
        if (!(strcmp(json_data_parse_value->valuestring, "succ")))
        {
            printf("active:success\r\n");
        }
        else
        {
            printf("active:error\r\n");
            cJSON_Delete(json_data_parse);
            return 0;
        }

        if (cJSON_GetObjectItem(json_data_parse, "data") != NULL)
        {
            json_data_parse_data = cJSON_GetObjectItem(json_data_parse, "data");
            json_data_parse_data_deviceid = cJSON_GetObjectItem(json_data_parse_data, "device_id");
            printf("DeviceId=%s\r\n", json_data_parse_data_deviceid->valuestring);

            //写入DeviceId
            sprintf(DeviceId,"%s%c",json_data_parse_data_deviceid->valuestring,'\0');           
            E2prom_Write(DEVICEID_ADDR,(uint8_t *)DeviceId, DEVICEID_LEN);
        }

    }
    cJSON_Delete(json_data_parse);
    return 1;
}



esp_err_t parse_Uart0(char *json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_ProductID = NULL;
    cJSON *json_data_parse_SeriesNumber = NULL;
    cJSON *json_data_parse_RegisterCode = NULL;

    //if(strstr(json_data,"{")==NULL)
    if(json_data[0]!='{')
    {
        printf("uart0 Json Formatting error1\n");
        return 0;
    }

    json_data_parse = cJSON_Parse(json_data);
    if (json_data_parse == NULL) //如果数据包不为JSON则退出
    {
        printf("uart0 Json Formatting error\n");
        cJSON_Delete(json_data_parse);

        return 0;
    }
    else
    {
        /*
        {
            "Command": "SetupProduct",
            "ProductID": "198412",
            "SeriesNumber": "AAA0001",
            "RegisterCode": "7CyBf3I2YgMmzZAg"
        }
        */
        
        
        json_data_parse_ProductID = cJSON_GetObjectItem(json_data_parse, "ProductID"); 
        printf("ProductID= %s\n", json_data_parse_ProductID->valuestring);

        json_data_parse_SeriesNumber = cJSON_GetObjectItem(json_data_parse, "SeriesNumber"); 
        printf("SeriesNumber= %s\n", json_data_parse_SeriesNumber->valuestring);

        json_data_parse_RegisterCode = cJSON_GetObjectItem(json_data_parse, "RegisterCode"); 
        printf("RegisterCode= %s\n", json_data_parse_RegisterCode->valuestring);

        char zero_data[256];
        bzero(zero_data,sizeof(zero_data));
        E2prom_Write(0x00, (uint8_t *)zero_data, sizeof(zero_data)); 
    
        sprintf(ProductId,"%s%c",json_data_parse_ProductID->valuestring,'\0');
        E2prom_Write(PRODUCTID_ADDR, (uint8_t *)ProductId, strlen(ProductId));
        
        sprintf(SerialNum,"%s%c",json_data_parse_SeriesNumber->valuestring,'\0');
        E2prom_Write(SERIALNUM_ADDR, (uint8_t *)SerialNum, strlen(SerialNum)); 

        sprintf(RegisterCode,"%s%c",json_data_parse_RegisterCode->valuestring,'\0');
        E2prom_Write(REGISTERCODE_ADDR, (uint8_t *)RegisterCode, strlen(RegisterCode)); 

        //清空DeviceId存储，激活后获取
        uint8_t data_write_0[DEVICEID_LEN]="\0";
        E2prom_Write(DEVICEID_ADDR, data_write_0, DEVICEID_LEN);


        //E2prom_Read(0x30,(uint8_t *)SerialNum,16);
        //printf("read SerialNum=%s\n", SerialNum);

        //E2prom_Read(0x40,(uint8_t *)ProductId,32);
        //printf("read ProductId=%s\n", ProductId);

        printf("{\"status\":\"success\",\"err_code\": 0}");
        cJSON_Delete(json_data_parse);
        fflush(stdout);//使stdout清空，就会立刻输出所有在缓冲区的内容。
        esp_restart();//芯片复位 函数位于esp_system.h
        return 1;

    }
}






/*esp_err_t parse_objects_mqtt(char *json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_value = NULL;

    json_data_parse = cJSON_Parse(json_data);

    if(json_data[0]!='{')
    {
        printf("mqtt Json Formatting error\n");

        return 0;       
    }

    if (json_data_parse == NULL) //如果数据包不为JSON则退出
    {

        printf("Json Formatting error4\n");

        cJSON_Delete(json_data_parse);
        return 0;
    }
    else
    {
        json_data_parse_value = cJSON_GetObjectItem(json_data_parse, "switch"); 
        printf("switch= %s\n", json_data_parse_value->valuestring);
        if(strcmp(json_data_parse_value->valuestring,"on")==0)
        {
            printf("switch on\n");
            Beep_On();
        }
        else if(strcmp(json_data_parse_value->valuestring,"off")==0)
        {
            printf("switch off\n");
            Beep_Off();
        }

    }

    cJSON_Delete(json_data_parse);
    

    return 1;
}*/





/*
{
	"temperature": 28,
	"humidity": 60,
	"speed": 39.3,
	"position": {
		"lon": "121.48",
		"lat": "38.96"
	}
}
*/

void create_mqtt_json(creat_json *pCreat_json)
{

    cJSON *root = cJSON_CreateObject();
   

    if (sht31_readTempHum()) 
    {		
        double Temperature = sht31_readTemperature();
        double Humidity = sht31_readHumidity();

        cJSON_AddItemToObject(root, "Temperature", cJSON_CreateNumber(Temperature));
        cJSON_AddItemToObject(root, "Humidity", cJSON_CreateNumber(Humidity)); 
        ESP_LOGI("SHT30", "Temperature=%.1f, Humidity=%.1f", Temperature, Humidity);
	} 
    else 
    {
		ESP_LOGE("SHT30", "SHT31_ReadTempHum : failed");
	}

    cJSON_AddItemToObject(root, "speed", cJSON_CreateNumber((int)speed));  
    if(longitude>0)
    {
        cJSON *next = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "position", next);
        cJSON_AddItemToObject(next, "lon", cJSON_CreateNumber(longitude));
        cJSON_AddItemToObject(next, "lat", cJSON_CreateNumber(latitude));
    }
    


    char *cjson_printunformat;
    cjson_printunformat=cJSON_PrintUnformatted(root);
    pCreat_json->creat_json_c=strlen(cjson_printunformat);
    bzero(pCreat_json->creat_json_b,sizeof(pCreat_json->creat_json_b));
    memcpy(pCreat_json->creat_json_b,cjson_printunformat,pCreat_json->creat_json_c);
    printf("len=%d,mqtt_json=%s\n",pCreat_json->creat_json_c,pCreat_json->creat_json_b);
    free(cjson_printunformat);
    cJSON_Delete(root);
    
}



