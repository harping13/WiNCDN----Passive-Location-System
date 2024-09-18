#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_cpu.h" // 引入CPU相关的函数声明
#include <inttypes.h>
#include "esp_task_wdt.h"

#define TARGET_MAC "aa:bb:cc:11:22:33"

#define MAX_PACKETS 1000 // 根据实际需求调整这个值

typedef struct {
    uint32_t timestamp; // 时间戳
    char packet_type[4]; // 数据包类型，"QoS" 或 "ACK"
} packet_info_t;

static packet_info_t packet_info_array[MAX_PACKETS];
static int packet_count = 0; // 已接收数据包的数量

typedef struct {
    uint16_t frame_control;
    uint16_t duration_id;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    uint16_t seq_ctrl;
    // Add more fields if necessary, depending on the frame types you're dealing with
} ieee80211_hdr_t;

static const char *TAG = "wifi_sniffer";
static uint8_t target_mac[6];

void IRAM_ATTR timer_callback(void* arg) {


    for (int i = 0; i < packet_count; i++) {
        ESP_LOGI("Packet Info" ,"Type: %s, Timestamp: %" PRIu32,
                 packet_info_array[i].packet_type, packet_info_array[i].timestamp);
    }

    packet_count = 0; // 重置计数器
}




// Function to convert target MAC address from string to byte array
void IRAM_ATTR parse_mac_address(const char *mac_str, uint8_t *mac) {
    int values[6];
    if (6 == sscanf(mac_str, "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5])) {
        for (int i = 0; i < 6; ++i) {
            mac[i] = (uint8_t)values[i];
        }
    }
}

// Helper function to compare MAC addresses
bool IRAM_ATTR compare_mac(const uint8_t *mac1, const uint8_t *mac2) {
    return memcmp(mac1, mac2, 6) == 0;
}

// Wi-Fi promiscuous callback function focused on logging timestamp
void IRAM_ATTR wifi_promiscuous_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (packet_count >= MAX_PACKETS) return; // 防止数组溢出
    //if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const uint8_t *frame = pkt->payload;
    const size_t frame_len = pkt->rx_ctrl.sig_mode;

  //  if (frame_len < sizeof(ieee80211_hdr_t)) return;

    const ieee80211_hdr_t *hdr = (const ieee80211_hdr_t *)frame;
    uint8_t frame_type = (hdr->frame_control & 0x0C) >> 2;
    uint8_t frame_subtype = (hdr->frame_control & 0xF0) >> 4;

    const uint8_t *addr_to_compare = NULL;
    if ((frame_type == 2 && frame_subtype == 12) || (frame_type == 1 && frame_subtype == 13)) {
        addr_to_compare = (frame_type == 1 && frame_subtype == 13) ? hdr->addr1 : hdr->addr2;
        if (compare_mac(addr_to_compare, target_mac)) {
        uint32_t timestamp_cycles = esp_cpu_get_cycle_count();
        
        // 根据帧类型填充packet_info_array
        if (frame_type == 2 && frame_subtype == 12) { // QoS
            strcpy(packet_info_array[packet_count].packet_type, "QoS");
        } else if (frame_type == 1 && frame_subtype == 13) { // ACK
            strcpy(packet_info_array[packet_count].packet_type, "ACK");
        }
        packet_info_array[packet_count].timestamp = timestamp_cycles;
        packet_count++;
        }
    }
}

// Initialize Wi-Fi in promiscuous mode
void wifi_init_promiscuous() {
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    // Create the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set Wi-Fi to promiscuous mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));


    // Configure the promiscuous filter to capture all types of frames
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));

    // Set control frame filter specifically
    wifi_promiscuous_filter_t ctrl_filter = {
        .filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ALL
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&ctrl_filter));

    

    

    ESP_LOGI(TAG, "Wi-Fi initialized in promiscuous mode");
    // Set the Wi-Fi channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Wi-Fi initialized in promiscuous mode, listening on channel 2 ");
}

void app_main(void) {
    // Parse the target MAC address
    parse_mac_address(TARGET_MAC, target_mac);

    // Initialize the Wi-Fi system in promiscuous mode
    wifi_init_promiscuous();

    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "periodic_timer"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10000000)); // 每1分钟触发一次


    while (1) {
        // taskYIELD();; // Effectively idle this task
       // esp_task_wdt_reset();
    }
}


