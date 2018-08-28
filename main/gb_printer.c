/*
 * This file is part of the GameBoy-Printer project.
 * Copyright (C) 2018  Tido Klaassen <tido_gbprinter@4gh.eu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "rom/cache.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_spiffs.h"

#include "soc/gpio_reg.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include <libesphttpd/esp.h>
#include <libesphttpd/httpd.h>
#include <libesphttpd/httpdespfs.h>
#include <libesphttpd/cgiflash.h>
#include <libesphttpd/auth.h>
#include <libesphttpd/espfs.h>
#include <libesphttpd/captdns.h>
#include <libesphttpd/webpages-espfs.h>
#include <libesphttpd/httpd-freertos.h>
#include <libesphttpd/route.h>
#include <libesphttpd/esp32_httpd_vfs.h>

#include "lodepng.h"

#define TAG "GB-Printer"

#define GPIO_MISO       4
#define GPIO_MOSI       13
#define GPIO_SCLK       14
#define WIFI_SSID       "GB-Printer"
#define MAX_STA_CONN    3
#define LISTEN_PORT     80u
#define MAX_CONNECTIONS 8u
#define NVS_NAMESPC     "GBPR"
#define IMGDIR          "/img"
#define IMGTMPL         "img%05d.png"
#define IMG_WIDTH       160 // Images are always 160 pixels wide
#define IMG_HEIGHT      144 // Maximum image height, may be less
#define IMG_SIZE        5760
#define MAX_DATA_SIZE   640
#define PR_PARAM_SIZE   4
#define PKT_RING_SIZE   4

#define HTML_HEADER     "<html>" \
                        "<head>" \
                        "<meta charset=\"utf-8\">" \
                            "<title>ESP32 GameBoy Printer</title>" \
                            "<link rel=\"stylesheet\" href=\"style.css\" " \
                                "media=\"screen\" title=\"no title\" " \
                                "charset=\"utf-8\">" \
                        "</head>"

/* Useful Linux-like defines */
#define ARRAY_SIZE(x)   (sizeof(x) / sizeof(*(x)))
#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)
#define mb()            asm volatile ("" : : : "memory")

#define STATUS_CHKSUM   (1u << 0)
#define STATUS_BUSY     (1u << 1)
#define STATUS_FULL     (1u << 2)
#define STATUS_UNPROC   (1u << 3)
#define STATUS_ERR0     (1u << 4)
#define STATUS_ERR1     (1u << 5)
#define STATUS_ERR2     (1u << 6)
#define STATUS_BATLOW   (1u << 7)

enum pr_state {
    state_sync0 = 0,
    state_sync1,
    state_cmd,
    state_compr,
    state_len_low,
    state_len_high,
    state_data,
    state_chk_low,
    state_chk_high,
    state_ack,
    state_status,
};

enum pr_cmd {
    cmd_init    = 0x01,
    cmd_print   = 0x02,
    cmd_data    = 0x04,
    cmd_break   = 0x08,
    cmd_inquiry = 0x0f,
};

enum pr_owner {
    own_isr = 0,
    own_task,
};

struct pr_packet {
    volatile enum pr_owner owner;
    enum pr_state state;
    uint8_t cmd;
    uint8_t compr;
    uint8_t data[MAX_DATA_SIZE];
    size_t data_len;
    size_t cur_len;
    uint16_t cur_sum;
    uint16_t chk_sum;
};

struct pr_data {
    uint8_t data[IMG_SIZE];
    size_t data_len;
    uint8_t params[PR_PARAM_SIZE];
    uint8_t status;
    unsigned int busy_cnt;
    unsigned int printed;
    size_t width;
    size_t hight;
};

static struct spi_ctrl {
    struct pr_packet work_packet;
    struct pr_packet packets[PKT_RING_SIZE];
    size_t clk_cnt;
    uint64_t clk_last;
    uint8_t rcv_data;
    uint8_t snd_data;
    volatile size_t wr_idx;
    volatile size_t rd_idx;
    volatile uint8_t status_isr;
    volatile uint8_t status_task;
    xQueueHandle packet_done;
} ctrl;

static uint8_t render_buff[IMG_SIZE];
static QueueHandle_t img_queue;
static EventGroupHandle_t wifi_event_group;
static char conn_mem[sizeof(RtosConnType) * MAX_CONNECTIONS];
static HttpdFreertosInstance httpd_instance;

CgiStatus cgi_del_all(HttpdConnData *conn);
CgiStatus cgi_reset_seq(HttpdConnData *conn);
CgiStatus tpl_index(HttpdConnData *conn, char *token, void **arg);

HttpdBuiltInUrl builtin_urls[]={
    ROUTE_REDIRECT("/", "/index.tpl"),
    ROUTE_REDIRECT("/index.html", "/index.tpl"),
    ROUTE_TPL("/index.tpl", tpl_index),
    ROUTE_CGI("/img/*", cgiEspVfsHook),
    ROUTE_CGI("/remove_all", cgi_del_all),
    ROUTE_CGI("/reset_seq", cgi_reset_seq),
    ROUTE_FILESYSTEM(),
    ROUTE_END()
};


CgiStatus tpl_index(HttpdConnData *conn, char *token, void **arg)
{
    struct dirent *ent;
    DIR *dir;
    char buff[128];
    CgiStatus result;

    dir = (DIR *) *arg;
    result = HTTPD_CGI_DONE;
    buff[0] = '\0';

    if(conn->isConnectionClosed){
        ESP_LOGD(TAG, "[%s] Conn closed", __func__);
        if(dir != NULL){
            closedir(dir);
        }
        goto err_out;
    }

    if(token == NULL){
       goto err_out;
    }

    if(strcmp(token, "GBPics") != 0){
        goto err_out;
    }

    if(dir == NULL){
        dir = opendir(IMGDIR);
        if(dir == NULL){
            ESP_LOGE(TAG, "[%s] opendir() failed", __func__);
            goto err_out;
        }

        *arg = dir;
    }

    ent = readdir(dir);
    if(ent == NULL){
        goto err_out;
    }

    snprintf(buff, sizeof(buff),
             "<br>"
             "<a href=\"%s/%s\">"
             "<img src=\"%s/%s\">"
             "</a>",
             IMGDIR, ent->d_name,
             IMGDIR, ent->d_name);

    result = HTTPD_CGI_MORE;

    ESP_LOGD(TAG, "[%s] Sending: %s", __func__, buff);
    httpdSend(conn, buff, -1);

err_out:
    if(result == HTTPD_CGI_DONE && dir != NULL){
        closedir(dir);
        *arg = NULL;
    }

    return result;
}

CgiStatus cgi_del_all(HttpdConnData *conn)
{
    struct dirent *ent;
    DIR *dir;
    char buff[128];
    int result;

    buff[0] = '\0';
    dir = NULL;
    result = 0;

    if(conn->isConnectionClosed){
        goto err_out;
    }

    dir = opendir(IMGDIR);
    if(dir == NULL){
        ESP_LOGE(TAG, "[%s] opendir() failed", __func__);
        result = -1;
        goto send_reply;
    }

    while((ent = readdir(dir)) != NULL){
        snprintf(buff, sizeof(buff), "%s/%s", IMGDIR, ent->d_name);
        result = unlink(buff);
        if(result != 0){
            ESP_LOGE(TAG, "[%s] Unlink failed for %s", __func__, buff);
            goto send_reply;
        }
    }

send_reply:
    if(result == 0){
        httpdRedirect(conn, "/index.tpl");
    } else {
        httpdStartResponse(conn, 500);
        httpdHeader(conn, "Content-Type", "text/html");
        httpdEndHeaders(conn);

        httpdSend(conn, HTML_HEADER, -1);
        httpdSend(conn, "<body>"
                            "<h1>ESP32 Gameboy Printer</h1>"
                            "<p>"
                                "Error while deleting file ", -1);
        httpdSend(conn, buff, -1);
        httpdSend(conn,     ".<br>"
                            "<a href=\"/index.tpl\">Go back</a>"
                            "<br>"
                            "<a href=\"/remove_all\">Retry</a>"
                        "</p>"
                        "</body>"
                        "</html>", -1);
    }

err_out:
    if(dir != NULL){
        closedir(dir);
    }

    return HTTPD_CGI_DONE;
}

CgiStatus cgi_reset_seq(HttpdConnData *conn)
{
    nvs_handle handle;
    int result;

    result = 0;

    if(conn->isConnectionClosed){
        goto err_out;
    }

    result = nvs_open(NVS_NAMESPC, NVS_READWRITE, &handle);
    if(result != ESP_OK){
        goto send_reply;
    }

    result = nvs_set_u32(handle, "next_idx", 0);
    nvs_commit(handle);
    nvs_close(handle);

send_reply:
    if(result == ESP_OK){
        httpdRedirect(conn, "/index.tpl");
    } else {
        httpdStartResponse(conn, 500);
        httpdHeader(conn, "Content-Type", "text/html");
        httpdEndHeaders(conn);

        httpdSend(conn, HTML_HEADER, -1);
        httpdSend(conn, "<body>"
                            "<h1>ESP32 Gameboy Printer</h1>"
                            "<p>"
                                "Error resetting image sequence."
                                "<br>"
                                "<a href=\"/index.tpl\">Go back</a>"
                                "<br>"
                                "<a href=\"/reset_seq\">Retry</a>"
                            "</p>"
                        "</body>"
                        "</html>", -1);
    }

err_out:
    return HTTPD_CGI_DONE;
}

esp_err_t http_srv_init(void)
{
    HttpdInitStatus status;
    esp_err_t result;

    result = ESP_OK;

    if(espFsInit((void*)(webpages_espfs_start)) != ESPFS_INIT_RESULT_OK){
        result = ESP_FAIL;
        goto err_out;
    }

    status = httpdFreertosInit(&httpd_instance,
	                           builtin_urls,
	                           LISTEN_PORT,
	                           conn_mem,
	                           MAX_CONNECTIONS,
	                           HTTPD_FLAG_NONE);

    if(status != InitializationSuccess){
        result = ESP_FAIL;
        goto err_out;
    }

    httpdFreertosStart(&httpd_instance);

err_out:
    return result;
}

void draw_tile(uint8_t *data, uint8_t *image)
{
    int x, y;
    uint16_t *tmp;

    for(y = 0; y < 8; ++y){
        tmp = (uint16_t *) &image[40 * y];
        *tmp = 0;
        for(x = 7; x >= 0; --x){
            *tmp |= ((data[1] >> x) & 0x1) << (2 * x + 1);
            *tmp |= ((data[0] >> x) & 0x1) << (2 * x);
        }
        *tmp = ~htons(*tmp);

        data += 2;
    }
}

esp_err_t draw_bitmap(struct pr_data *data)
{
    size_t w, h, x, y;
    uint8_t *tile_data, *tile_dest;
    unsigned int tile;
    esp_err_t result;

    result = ESP_OK;

    w = IMG_WIDTH;
    h = (data->data_len * 4) / IMG_WIDTH;

    if(h > IMG_HEIGHT){
        ESP_LOGE(TAG, "[%s] Invalid image hight: %d", __func__, h);
        result = ESP_ERR_INVALID_SIZE;
        goto err_out;
    }

    memset(render_buff, 0x0, sizeof(render_buff));

    for(tile = 0; tile < (w * h / 64); ++tile){
        x = tile % 20;
        y = 8 * (tile / 20);

        tile_data = &(data->data[tile * 16]);
        tile_dest = &(render_buff[(x * 2) + (y * 40)]);
        draw_tile(tile_data, tile_dest);
    }

    memmove(data->data, render_buff, data->data_len);
    data->width = w;
    data->hight = h;

err_out:
    return result;
}

void save_data(struct pr_data *data)
{
    struct stat statbuf;
    uint32_t idx;
    FILE *file;
    char path[128];
    nvs_handle handle;
    size_t written, png_size;
    uint8_t *png;
    LodePNGState state;
    esp_err_t result;

    png = NULL;
    file = NULL;

    result = nvs_open(NVS_NAMESPC, NVS_READWRITE, &handle);
    ESP_ERROR_CHECK(result);

    result = draw_bitmap(data);
    if(result != ESP_OK){
        goto err_out;
    }

    lodepng_state_init(&state);
    state.info_raw.colortype = LCT_GREY;
    state.info_raw.bitdepth = 2;
    state.info_png.color.colortype = LCT_GREY;
    state.info_png.color.bitdepth = 2;
    state.encoder.zlibsettings.btype = 0; // disable compression due to lack of memory

    lodepng_encode(&png, &png_size, data->data, data->width, data->hight, &state);
    result = state.error;
    lodepng_state_cleanup(&state);

    if(result != 0){
        ESP_LOGE(TAG, "[%s] lodepng failed: %s", __func__, lodepng_error_text(result));
        goto err_out;
    }

    result = nvs_get_u32(handle, "next_idx", &idx);
    if(result != ESP_OK){
        ESP_LOGI(TAG, "[%s] No image sequence number found", __func__);
        idx = 0;
    }

    /* Find first free image name. Start at index read from NVS and increment
     * until we find a free slot. */
    errno = 0;
    do{
        snprintf(path, sizeof(path) - 1, IMGDIR "/" IMGTMPL, idx);
        result = stat(path, &statbuf);
        if(result == 0){
            ESP_LOGD(TAG, "[%s] Found %s", __func__, path);
            ++idx;
        }
    }while(result == 0);

    if(errno != ENOENT){
        ESP_LOGE(TAG, "Saving data failed: %s", strerror(errno));
        goto err_out;
    }

    file = fopen(path, "w");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file %s: %s", path, strerror(errno));
        goto err_out;
    }

    written = fwrite(png, 1, png_size, file);
    if(written != png_size){
        ESP_LOGE(TAG, "Error writing data: %s", strerror(errno));
        result = EIO;
        goto err_out;
    }

    fclose(file);
    file = NULL;

    if(nvs_set_u32(handle, "next_idx", idx + 1) != ESP_OK){
        /* Not super critical. Worst case we get a duplicate image name if
         * picture storage gets erased before next image is saved */
        ESP_LOGW(TAG, "[%s] Error storing image sequence number", __func__);
    }

    nvs_commit(handle);

err_out:
    if(file != NULL){
        fclose(file);
        if(result != ESP_OK){
            unlink(path);
        }
    }

    if(png != NULL){
        free(png);
    }

    nvs_close(handle);

    return;
}

esp_err_t init_spiffs(void)
{
    esp_err_t result;
    esp_vfs_spiffs_conf_t cfg;

    ESP_LOGI(TAG, "Initializing SPIFFS");

    cfg.base_path = "/img";
    cfg.partition_label = NULL;
    cfg.max_files = MAX_CONNECTIONS;
    cfg.format_if_mount_failed = true;

    result = esp_vfs_spiffs_register(&cfg);

    switch(result){
    case ESP_OK:
        break;
    case ESP_FAIL:
        ESP_LOGE(TAG, "Failed to mount or format filesystem");
        break;
    case ESP_ERR_NOT_FOUND:
        ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        break;
    default:
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result));
        break;
    }

    return result;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                     MAC2STR(event->event_info.sta_connected.mac),
                     event->event_info.sta_connected.aid);
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                     MAC2STR(event->event_info.sta_disconnected.mac),
                     event->event_info.sta_disconnected.aid);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void wifi_init_softap()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

}

static void IRAM_ATTR sclk_isr_handler(void* arg)
{
    struct spi_ctrl *ctrl;
    struct pr_packet *packet;
    uint64_t curr_time;
    BaseType_t task_woken;

    ctrl = (struct spi_ctrl *) arg;
    packet = &(ctrl->work_packet);

    if(gpio_get_level(GPIO_SCLK) == 0){
        /* Falling edge. Check timeouts and set up MISO. */

        curr_time = esp_timer_get_time();

        /* Check for sync loss during byte transfer. Clock should run  */
        /* at ~8kHz (125us), so anything above 200us is definitely out */
        /* of spec.                                                    */
        if(unlikely(ctrl->clk_cnt != 0 && (curr_time - ctrl->clk_last) > 200)){
            ctrl->clk_cnt = 0;
            ctrl->rcv_data = 0;
            ctrl->snd_data = 0;
            ctrl->status_isr |= STATUS_ERR0;
            packet->state = state_sync0;
        }

        /* According to the Game Boy programming manual the maximum allowed */
        /* pause between sending bytes in a packet is 5ms. */
        if(unlikely(packet->state != state_sync0
                    && (curr_time - ctrl->clk_last) > 5100))
        {
            ctrl->status_isr |= STATUS_ERR0;
            packet->state = state_sync0;
        }

        ctrl->clk_last = curr_time;

        gpio_set_level(GPIO_MISO, (ctrl->snd_data & 0x80u) ? 1 : 0);
        ctrl->snd_data <<= 1;
    } else {
        // Rising edge. Sample MOSI and increase clock count.
        if(unlikely(ctrl->clk_cnt == 0)){
            ctrl->rcv_data = 0;
        }

        ctrl->rcv_data <<= 1;
        if(gpio_get_level(GPIO_MOSI)){
            ctrl->rcv_data |= 1;
        }

        ++ctrl->clk_cnt;
    }

    if(likely(ctrl->clk_cnt <= 7)){
        goto done;
    }

    ctrl->clk_cnt = 0;

    if(packet->state >= state_cmd && packet->state < state_chk_low){
        packet->cur_sum += ctrl->rcv_data;
    }

    /* Process the transferred byte. The state indicates how the byte that */
    /* has just completed is interpreted. If data needs to be sent out in  */
    /* state x, it must be copied to ctrl->snd_data during transition to   */
    /* that state.                                                         */
    switch(packet->state){
    case state_sync0:
        if(ctrl->rcv_data == 0x88){
            packet->cur_sum = 0;
            packet->data_len = 0;
            ctrl->status_isr &= ~(STATUS_CHKSUM | STATUS_BUSY);

            packet->state = state_sync1;
        }
        break;
    case state_sync1:
        if(ctrl->rcv_data == 0x33){
            packet->state = state_cmd;
        } else {
            packet->state = state_sync0;
            ctrl->status_isr |= STATUS_ERR0;
        }
        break;
    case state_cmd:
        packet->cmd = ctrl->rcv_data;
        packet->state = state_compr;
        break;
    case state_compr:
        packet->compr = ctrl->rcv_data;
        packet->state = state_len_low;
        break;
    case state_len_low:
        packet->cur_len = ctrl->rcv_data;
        packet->state = state_len_high;
        break;
    case state_len_high:
        packet->cur_len |= (ctrl->rcv_data << 8);

        if(likely(packet->cur_len <= sizeof(packet->data))){
            if(packet->cur_len > 0){
                packet->state = state_data;
            } else {
                packet->state = state_chk_low;
            }
        } else {
            /* Bogus data length. We could try inferring the real length
             * from the command byte, but since we probably have lost
             * synchronisation anyway, we just give up on this packet */
            packet->state = state_sync0;
            ctrl->status_isr |= STATUS_ERR0;
        }
        break;
    case state_data:
        packet->data[packet->data_len++] = ctrl->rcv_data;
        --packet->cur_len;

        if(unlikely(packet->cur_len == 0)){
            packet->state = state_chk_low;
        }
        break;
    case state_chk_low:
        packet->chk_sum = ctrl->rcv_data;
        packet->state = state_chk_high;
        break;
    case state_chk_high:
        packet->chk_sum |= (ctrl->rcv_data << 8);

        /* We need to send the ACK byte next. */
        ctrl->snd_data = 0x81;

        packet->state = state_ack;
        break;
    case state_ack:
        /* Packet is almost complete, we need to send out the checksum */
        /* in the next byte. Check for errors and update ISR status    */
        /* accordingly. If everything checks out, try copying packet   */
        /* to transfer ring.                                           */

        if(unlikely(packet->chk_sum != packet->cur_sum)){
            ctrl->status_isr |= (STATUS_CHKSUM | STATUS_ERR0);
        } else {
            switch(packet->cmd){
            case cmd_data:
                /* We expect to always receive complete bands (20 tiles) */
                /* of image data.                                        */
                if(packet->data_len % 40 == 0)
                {
                    ctrl->status_isr |= STATUS_UNPROC;
                } else {
                    ctrl->status_isr |= STATUS_ERR0;
                }
                break;
            case cmd_print:
                if(packet->data_len == PR_PARAM_SIZE){
                    ctrl->status_isr &= ~STATUS_UNPROC;
                    ctrl->status_isr |= STATUS_FULL;
                } else {
                    ctrl->status_isr |= STATUS_ERR0;
                }
                break;
            case cmd_inquiry:
                break;
            case cmd_init:
            case cmd_break:
                ctrl->status_isr = 0;
                break;
            }

            if(ctrl->packets[ctrl->wr_idx].owner == own_isr){
                memcpy(&(ctrl->packets[ctrl->wr_idx]), packet, sizeof(*packet));
            } else {
                /* Protocol task is too slow, signal busy error. */
                ctrl->status_isr |= (STATUS_ERR0 | STATUS_BUSY);
            }
        }
 
        /* Send combined error status of ISR and task */
        ctrl->snd_data = (ctrl->status_isr | ctrl->status_task);

        packet->state = state_status;
        break;
    case state_status:
        /* Status byte has been sent out, packet transfer is complete. */
        /* Hand packet over to protocol task, unless we had a checksum */
        /* error or packet ring was full.                              */
        if(likely((ctrl->status_isr & (STATUS_CHKSUM | STATUS_BUSY)) == 0)){
            mb();
            ctrl->packets[ctrl->wr_idx].owner = own_task;
            mb();

            xSemaphoreGiveFromISR(ctrl->packet_done, &task_woken);
            if(task_woken){
                portYIELD_FROM_ISR();
            }

            ++ctrl->wr_idx;
            ctrl->wr_idx %= ARRAY_SIZE(ctrl->packets);
        }

        /* Get ready to start reception of next packet */
        packet->state = state_sync0;
        ctrl->snd_data = 0;
        break;
    default:
        /* Should never be reached. */
        packet->state = state_sync0;
        ctrl->status_isr |= STATUS_ERR0;
        break;
    }

done:
    return;
}

void IRAM_ATTR packet_proto_task(void *pvParameters)
{
    struct pr_packet *packet;
    struct pr_data *data = NULL;

    ESP_LOGI(TAG, "Packet proto task started.");

    while(1) {
        /* Make sure we have an image data buffer ready before we accept */
        /* any command packages from the ISR.                            */

        if(data == NULL){
            data = calloc(1, sizeof(*data));
            if(data == NULL){
                ESP_LOGE(TAG, "Out of memory");
                ctrl.status_task = STATUS_BUSY;
                continue;
            }
        }

        /* If we have a completed image buffer, try handing it to the main */
        /* task for rendering and saving to flash.                         */
        if(data->printed != 0){
            if(xQueueSend(img_queue, &data, 0) != pdTRUE){
                ctrl.status_task |= STATUS_BUSY;
                ++data->busy_cnt;

                if(data->busy_cnt < 5){
                    /* Give main task a chance to process the queue. */
                    vTaskDelay(portTICK_PERIOD_MS);
                } else {
                    /* Give up and signal an error after five tries. */
                    memset(data, 0x0, sizeof(*data));
                    ctrl.status_task |= STATUS_ERR0;
                }
            } else {
                /* Data buffer belongs to main task now. We will allocate */
                /* a new one on next loop iteration.                      */
                data = NULL;
                ctrl.status_task &= ~STATUS_BUSY;
            }

            /* Restart loop for either retrying this buffer or allocating */
            /* a new one.                                                 */
            continue;
        }

        /* We have a non-completed data buffer now. Remove busy signal and */
        /* wait for command packet from ISR.                               */
        ctrl.status_task &= ~STATUS_BUSY;

        packet = &(ctrl.packets[ctrl.rd_idx]);
        if(packet->owner == own_isr){
            xSemaphoreTake(ctrl.packet_done, portMAX_DELAY);
        }

        /* No new packet availabe? Restart loop. */
        if(packet->owner == own_isr){
            continue;
        }

        ESP_LOGD(TAG, "Got packet: cmd: 0x%02x status_isr: 0x%02x status_task: 0x%02x",
                  packet->cmd, ctrl.status_isr, ctrl.status_task);

        switch(packet->cmd){
        case cmd_data:
            /* We expect to always receive complete bands (20 tiles)  */
            /* of image data. Copy data payload into the image buffer */
            /* and update the current offset.                         */

            ESP_LOGD(TAG, "cmd_data: len: 0x%x", packet->data_len);
            if(   (packet->data_len % 40 == 0)
               && (data->data_len + packet->data_len <= sizeof(data->data)))
            {
                memcpy(&(data->data[data->data_len]),
                       packet->data,
                       packet->data_len);

                data->data_len += packet->data_len;
                ctrl.status_task |= STATUS_UNPROC;
            } else {
                ESP_LOGE(TAG, "Invalid data buffer length: 0x%x",
                         packet->data_len);

                ctrl.status_task |= STATUS_ERR0;
            }
            break;
        case cmd_print:
            /* Image data transfer is complete. Prepare to hand data buffer */
            /* over to main task.                                           */

            ESP_LOGD(TAG, "cmd_print: len: 0x%x", packet->data_len);
            if(packet->data_len == sizeof(data->params)){
                memcpy(data->params, packet->data, packet->data_len);
                ctrl.status_task &= ~STATUS_UNPROC;
                ctrl.status_task |= STATUS_FULL;
                data->busy_cnt = 0;
                data->printed = 1;
            } else {
                ESP_LOGE(TAG, "Parameter buffer overrun");
                ctrl.status_task |= STATUS_ERR0;
            }
            break;
        case cmd_inquiry:
            /* Status inquiry is handled completely in ISR. */

            ESP_LOGD(TAG, "cmd_inquiry: len: 0x%x", packet->data_len);
            break;
        case cmd_init:
        case cmd_break:
            /* Hard reset the whole transaction. This is the only way to     */
            /* clear possible error states from the task's status indicator. */

            ESP_LOGD(TAG, "cmd_init/break");
            memset(data, 0x0, sizeof(*data));
            ctrl.status_task = 0;
            break;
        }

        /* Hand ownership of command packet back to ISR, move read index */
        /* to next element in ring.                                      */
        mb();
        packet->owner = own_isr;
        mb();

        ++ctrl.rd_idx;
        ctrl.rd_idx %= ARRAY_SIZE(ctrl.packets);
    }
}

void app_main()
{
    esp_err_t result;
    gpio_config_t io_cfg;
    struct pr_data *data = NULL;

    // Initialise NVS
    result = nvs_flash_init();
    if(   result == ESP_ERR_NVS_NO_FREE_PAGES
       || result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    memset(&ctrl, 0x0, sizeof(ctrl));

    memset(&io_cfg, 0x0, sizeof(io_cfg));
    io_cfg.intr_type = GPIO_INTR_ANYEDGE;
    io_cfg.mode = GPIO_MODE_INPUT;
    io_cfg.pull_up_en = 0;
    io_cfg.pin_bit_mask = (1u << GPIO_SCLK);

    result = gpio_config(&io_cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "gpio_config() failed for GPIO_SCLK");
        goto err_out;
    }

    io_cfg.intr_type = GPIO_INTR_DISABLE;
    io_cfg.mode = GPIO_MODE_INPUT;
    io_cfg.pull_up_en = 0;
    io_cfg.pin_bit_mask = (1u << GPIO_MOSI);

    result = gpio_config(&io_cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "gpio_config() failed for GPIO_MOSI");
        goto err_out;
    }

    io_cfg.intr_type = GPIO_INTR_DISABLE;
    io_cfg.mode = GPIO_MODE_OUTPUT_OD;
    io_cfg.pull_up_en = 0;
    io_cfg.pin_bit_mask = (1u << GPIO_MISO);

    result = gpio_config(&io_cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "gpio_config() failed for GPIO_MISO");
        goto err_out;
    }

    memset(&ctrl, 0x0, sizeof(ctrl));
    ctrl.packet_done = xSemaphoreCreateBinary();
    if(ctrl.packet_done == NULL){
        ESP_LOGE(TAG, "xSemaphoreCreateBinary() failed");
        goto err_out;
    }

    result = init_spiffs();
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] init_spiffs() failed", __func__);
        goto err_out;
    }

    result = gpio_install_isr_service(0);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_install_isr_service() failed", __func__);
        goto err_out;
    }

    result = gpio_isr_handler_add(GPIO_SCLK, sclk_isr_handler, &ctrl);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] gpio_isr_handler_add() failed", __func__);
        goto err_out;
    }

    img_queue = xQueueCreate(2, sizeof(struct pr_data *));
    if(img_queue == NULL){
        ESP_LOGE(TAG, "[%s] Creating packet queue failed.", __func__);
        goto err_out;
    }

#if 1
    wifi_init_softap();
    http_srv_init();
#endif

    xTaskCreate(&packet_proto_task, "packet_proto_task", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "Entering main loop");
    while(1){
        if(xQueueReceive(img_queue, &data, portMAX_DELAY) == pdTRUE){
            save_data(data);
            free(data);
        }
    }

err_out:
    return;
}
