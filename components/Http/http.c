#include "Http.h"
#include "nvs.h"
#include "Json_parse.h"
#include "E2prom.h"
#include "libGSM.h"
#include "gnss.h"

#define WEB_SERVER "api.heclouds.com"
#define WEB_PORT 80

extern const int PPP_CONNECTED_BIT;
static char *TAG = "HTTP";


struct HTTP_STA
{
    char GET[10];
    char POST[10];
    char HEART_BEAT[64];

    char POST_URL1[64];
    char POST_URL_METADATA[16];
    char POST_URL_FIRMWARE[16];
    char POST_URL_SSID[16];
    char POST_URL_COMMAND_ID[16];

    char WEB_URL1[50];
    char WEB_URL2[20];
    char WEB_URL3[20];

    char HTTP_VERSION10[20];
    char HTTP_VERSION11[20];

    char HOST[30];
    char USER_AHENT[40];
    char CONTENT_LENGTH[30];
    char ENTER[10];
} http = {"GET ",
          "POST ",
          "http://api.ubibot.cn/heartbeat?api_key=",

          "http://api.heclouds.com/register_de?register_code=",
          "&metadata=true",
          "&firmware=",
          "&ssid=",
          "&command_id=",

          "http://api.ubibot.cn/products/",
          "/devices/",
          "/activate",

          " HTTP/1.0\r\n",
          " HTTP/1.1\r\n",

          "Host: api.heclouds.com\r\n",
          "User-Agent: dalian\r\n",
          "Content-Length:",
          "\r\n\r\n"};

TaskHandle_t httpHandle = NULL;
esp_timer_handle_t http_suspend_p = 0;

void http_suspends(void *arg)
{
    xTaskResumeFromISR(httpHandle);
}

esp_timer_create_args_t http_suspend = {
    .callback = &http_suspends,
    .arg = NULL,
    .name = "http_suspend"};



//×¢²áÁ÷³Ì
int http_activate(void)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;

    int32_t s = 0, r = 0;

    char build_post_url[512];
    char build_register_json[256];
    char recv_buf[1024];

    sprintf(build_register_json,"{\"sn\":\"%s\"}",SerialNum);

    sprintf(build_post_url, "%s%s%s%s%s%s%s%d%s%s", http.POST, http.POST_URL1, RegisterCode,
            http.HTTP_VERSION11, http.HOST, http.USER_AHENT, http.CONTENT_LENGTH, strlen(build_register_json), http.ENTER,build_register_json);

   

    printf("build_post_url= %s\n",build_post_url);
    while(1)
    {

        xEventGroupWaitBits(ppp_event_group, PPP_CONNECTED_BIT , false, true, portMAX_DELAY); 

        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

        if (err != 0 || res == NULL)
        {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.
        Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if (s < 0)
        {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
        {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
        if (write(s, build_post_url, strlen(build_post_url)) < 0)
        {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");
        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                        sizeof(receiving_timeout)) < 0)
        {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */

        bzero(recv_buf,sizeof(recv_buf));
        r = read(s, recv_buf, sizeof(recv_buf) - 1);
        printf("r=%d,activate recv_buf=%s\r\n",r, recv_buf);
        close(s);
             
        
        return parse_objects_http_active(strchr(recv_buf, '{'));
    }
}


