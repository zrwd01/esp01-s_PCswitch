/**
 * @file ESP8266_PC_Controller.ino
 * @brief 用于PC远程控制的智能开关固件 (专业重构版 - 最终稳定版)
 * 
 * 功能说明：
 *  1. 通过WiFi连接巴法云平台，实现PC远程控制
 *  2. 支持开机、关机、强制关机操作
 *  3. 实时检测PC状态（开机/关机/休眠）
 *  4. 支持Airkiss智能配网
 *  5. 支持OTA远程升级
 *  6. 系统状态监控和远程配置
 *  7. 多重看门狗和自愈机制
 * 
 * @version 4.3.8 (Final Stable - Critical Fixes)
 * @date 2025.8.21
 * @author fengye816
 * @QQ 376666816
 */

//======================================================================
// ====================== 1. 头文件与库引入 ========================
//======================================================================
#include <ESP8266WiFi.h>          // ESP8266 WiFi功能库
#include <ESP8266httpUpdate.h>    // OTA升级功能库
#include <ESP8266HTTPClient.h>    // HTTP客户端功能库
#include <EEPROM.h>               // EEPROM存储功能库
#include <ArduinoJson.h>          // JSON解析库（用于远程配置）

//======================================================================
// ====================== 2. 全局配置与宏定义 =======================
//======================================================================

// --- 版本号 ---
#define FIRMWARE_VERSION "4.3.8"    // 固件版本号

// --- 调试模式总开关 ---
// 描述: 设置为 1 将启用调试模式。在该模式下，设备会连接到测试用的主题。发布时请务必设置为 0。
#define DEBUG_MODE 0
// --- 设备与用户信息 ---
const char* uid = "xxxx"; // 你的巴法云私钥


#if DEBUG_MODE == 1
// --- 调试模式下的主题与URL配置 ---
#define TOPIC_CONTROL "tespc1"         // 调试模式控制主题
#define TOPIC_PC_STATE "testPCState1"       // 调试模式状态主题
#define TOPIC_TEST_DATE "testDate"          // 调试模式测试主题
#define FIRMWARE_URL_STR "xxxx" // 调试模式OTA地址
#else
// --- 发布模式下的主题与URL配置 ---
#define TOPIC_CONTROL "pc001"           // 发布模式控制主题
#define TOPIC_PC_STATE "PCState"            // 发布模式状态主题
#define TOPIC_TEST_DATE ""                  // 发布模式测试主题（空）
#define FIRMWARE_URL_STR "xxxx" // 发布模式OTA地址
#endif


// --- 调试与日志控制 ---
uint8_t runtime_log_level = 3;    // 运行时日志级别, 默认: 3(E+W+I)。可通过指令动态修改 (0-4)
#define LOG_OUTPUT Serial         // 日志输出目标，默认为串口
#define LOG_PREFIX __func__, __LINE__ // 日志自带函数名和行号前缀

// 定义分级日志宏，只有当 `runtime_log_level` 大于等于指定级别时，日志才会被打印
#if !defined(LOG_E)
#define LOG_E(format, ...) do { if (runtime_log_level >= 1) LOG_OUTPUT.printf_P(PSTR("[E] %s:%d: " format "\r\n"), LOG_PREFIX, ##__VA_ARGS__); } while(0) // 错误
#define LOG_W(format, ...) do { if (runtime_log_level >= 2) LOG_OUTPUT.printf_P(PSTR("[W] %s:%d: " format "\r\n"), LOG_PREFIX, ##__VA_ARGS__); } while(0) // 警告
#define LOG_I(format, ...) do { if (runtime_log_level >= 3) LOG_OUTPUT.printf_P(PSTR("[I] %s:%d: " format "\r\n"), LOG_PREFIX, ##__VA_ARGS__); } while(0) // 信息
#define LOG_D(format, ...) do { if (runtime_log_level >= 4) LOG_OUTPUT.printf_P(PSTR("[D] %s:%d: " format "\r\n"), LOG_PREFIX, ##__VA_ARGS__); } while(0) // 调试
#endif

// --- PC状态检测高级配置 ---
#define LED_SAMPLING_PERIOD_MS 2000         // LED状态采样周期 (毫秒)，推荐2000-3000ms，以完整覆盖1-2个闪烁周期
#define LED_ON_DUTY_CYCLE_THRESHOLD 90      // 判定为"开机"(LED常亮)的高电平时间占比阈值 (%) (已从95调整为90以增加兼容性)
#define LED_OFF_DUTY_CYCLE_THRESHOLD 5      // 判定为"关机"(LED熄灭)的高电平时间占比阈值 (%)
#define STATE_DEBOUNCE_TIME 5000            // 状态防抖时间 (毫秒)，防止状态抖动

// --- EEPROM优化配置 ---
#define EEPROM_WRITE_DELAY 5000             // EEPROM写入延迟 (毫秒)，延迟写入以减少写入次数
#define MIN_EEPROM_WRITE_INTERVAL 60000     // 最小EEPROM写入间隔 (毫秒)，防止频繁写入

// --- 系统监控配置 ---
#define STATUS_REPORT_INTERVAL 300000      // 系统状态报告间隔 (毫秒)，5分钟报告一次
#define MEMORY_FRAGMENTATION_THRESHOLD 80  // 内存碎片率告警阈值 (%)
#define RSSI_THRESHOLD -90                 // WiFi信号强度告警阈值 (dBm)
#define ERROR_COUNT_THRESHOLD 10           // 错误次数告警阈值

// --- 常量定义 ---
#define HOST_NAME "bemfa"                   // 设备主机名前缀
#define MAGIC_NUMBER 0xAA                   // EEPROM有效性校验码，用于判断是否首次烧录或配置是否损坏
#define TCP_SERVER_ADDR "bemfa.com"         // TCP服务器地址
#define TCP_SERVER_PORT 8344                // TCP服务器端口
#define TCP_BUFFER_SIZE 512                 // TCP接收缓冲区大小
#define KEEPALIVE_INTERVAL 10000            // TCP心跳包发送间隔 (毫秒)

// --- 硬件引脚定义 ---
const int PIN_PC_POWER = 0;                 // GPIO0: 控制PC电源继电器的引脚
const int PIN_LED_STATE = 2;                // GPIO2: 检测PC电源LED状态的引脚 (需接上拉电阻)

//======================================================================
// ====================== 3. 全局变量与对象 ========================
//======================================================================

// --- EEPROM配置结构体 ---
struct ConfigType { 
    char stassid[32];
    char stapsw[64];
    uint8_t reboot;
    uint8_t magic;
    uint8_t checksum;
    char tcp_server[32];
    uint16_t tcp_port;
    uint16_t heartbeat_interval;
    uint16_t reconnect_delay;
    uint8_t on_threshold;
    uint8_t off_threshold;
    uint16_t debounce_time;
    uint16_t short_press;
    uint16_t long_press;
    uint8_t log_level;
    uint16_t report_interval;
    uint16_t auto_reboot;
    uint8_t debug_mode;
    uint8_t remote_config;
    char config_password[16];
    uint8_t reserved[32];
};
ConfigType config;
uint8_t *p_config = (uint8_t *)(&config);

// --- 枚举定义 ---
enum PcPowerState { PC_STATE_OFF = 0, PC_STATE_ON = 1, PC_STATE_SLEEP = 2 };
enum PcControlState { IDLE, PRESSING_BUTTON, WAITING_FOR_RESULT };
enum ActionType { ACTION_NONE, ACTION_STARTUP, ACTION_SHUTDOWN, ACTION_FORCE_SHUTDOWN };
enum ConfigChangeType { CFG_NONE = 0, CFG_REBOOT_COUNTER, CFG_WIFI_SETTINGS, CFG_ALL };

// --- 状态变量 ---
PcPowerState pc_on_state = PC_STATE_OFF;
PcControlState pc_control_state = IDLE;
ActionType current_action = ACTION_NONE;

// --- 系统状态结构体 ---
struct SystemStatus {
    char firmware_version[16];
    unsigned long uptime_seconds;
    unsigned long last_boot_time;
    uint32_t free_heap;
    uint8_t heap_fragmentation;
    uint32_t flash_write_count;
    int wifi_rssi;
    bool wifi_connected;
    bool tcp_connected;
    uint32_t tcp_sent_packets;
    uint32_t tcp_received_packets;
    uint32_t last_heartbeat_time;
    PcPowerState pc_state;
    PcControlState control_state;
    ActionType last_action;
    unsigned long last_action_time;
    uint16_t reboot_count;
    uint16_t wifi_fail_count;
    uint16_t tcp_fail_count;
    uint16_t eeprom_fail_count;
    uint8_t status_flags;
};
SystemStatus current_status;

// --- 状态标志位定义 ---
#define STATUS_FLAG_OTA_UPDATING    0x01
#define STATUS_FLAG_SMARTCONFIG     0x02
#define STATUS_FLAG_ERROR_STATE     0x04
#define STATUS_FLAG_LOW_MEMORY      0x08
#define STATUS_FLAG_WEAK_SIGNAL     0x10

// --- 计时器与标志位 ---
unsigned long action_start_time = 0;
unsigned long shutdown_request_time = 0;
char config_flag = 0;
WiFiClient tcp_client;
HTTPClient http_client;
WiFiClient http_wifi_client;                // 专门用于HTTPClient (微信) 的WiFi客户端
WiFiClient ota_wifi_client;                 // 专门用于OTA升级的WiFi客户端
char tcp_recv_buffer[TCP_BUFFER_SIZE];
size_t tcp_buffer_len = 0;
unsigned long pre_heartbeat_tick = 0;
unsigned long pre_tcp_connect_tick = 0;
bool is_tcp_previously_connected = false;
int tcp_connect_attempts = 0;
unsigned long tcp_reconnect_delay = 5000;
unsigned long last_10d_reboot_time = 0;
unsigned long smartconfig_start_time = 0;

// --- EEPROM优化变量 ---
unsigned long last_eeprom_write_time = 0;
bool eeprom_dirty = false;
ConfigChangeType pending_change = CFG_NONE;
unsigned long last_eeprom_write_completion_time = 0;

// --- 系统监控变量 ---
unsigned long last_status_report_time = 0;
bool memory_alert_sent = false;
bool rssi_alert_sent = false;
bool error_alert_sent = false;
bool is_reporting_info = false; // 用于防止info指令并发的标志

// --- 高级状态检测专用变量 ---
unsigned long led_sampling_start_time = 0;
unsigned long led_high_duration_ms = 0;
unsigned long last_led_read_time = 0;
bool last_led_state = LOW;
PcPowerState last_stable_state = PC_STATE_OFF;
unsigned long state_change_time = 0;

//======================================================================
// =================== 4. 函数原型 (Forward Declarations) ==============
//======================================================================

void safeRestart(float t);
void handleSerialCommands();
void reportSystemStatus();
void checkSystemAlerts();
void sendSystemAlert(const char* format, ...);
void urlEncode(const char* src, char* dst, int dstSize);
uint8_t calculateChecksum(const ConfigType& cfg);
void saveConfig();
void restoreFactory();
void loadConfig();
void resetRebootCounter();
void handleEepromWrites();
void doSmartconfig();
void handleSmartconfig();
void initWiFi();
void update_started();
void update_finished();
void update_progress(int cur, int total);
void update_error(int err);
void updateBin();
void sendWeChatNotification();
void sendSystemInfoToWeChat();
void reportInfoOnDemand();
void startTCPClient();
void doTCPClientTick();
void processTcpCommand(char* command);
void processRemoteConfig(const char* config_data);
void processSecureConfig(const char* command);
void processConfigQuery();
void processDiagnosticCommand(const char* command);
void sendConfigResponse(const char* status, const char* message);
void sendDiagnosticResponse(const char* message);
void getCurrentConfig(char* buffer, size_t buffer_size);
void applyNewConfig(const ConfigType& new_config);
bool validateConfig(const ConfigType& cfg);
bool checkConfigPermission(const char* password);
void updatePcPowerState();
void reportPcState();
void requestPcOn();
void requestPcOff();
void handlePcControl();
void initSystemStatus();
void updateSystemStatus();
void generateStatusReport(char* buffer, size_t buffer_size);
void sendStatusReport();
void handleStatusMonitoring();

//======================================================================
// ==================== 5. 系统工具与辅助函数 ======================
//======================================================================

void safeRestart(float t) {
    LOG_I("系统将在 %.1f 秒后重启...", t);
    delay((unsigned long)(t * 1000));
    ESP.restart();
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        static char cmd_buffer[32];
        static int cmd_len = 0;
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buffer[cmd_len] = '\0';
                LOG_I("收到串口指令: %s", cmd_buffer);
                if (strncmp(cmd_buffer, "log=", 4) == 0) {
                    uint8_t new_level = atoi(cmd_buffer + 4);
                    if (new_level <= 4) {
                        runtime_log_level = new_level;
                        LOG_I("日志级别已设置为: %d", runtime_log_level);
                    } else {
                        LOG_W("无效的日志级别 (请输入0-4)");
                    }
                } else if (strcmp(cmd_buffer, "reboot") == 0) {
                    safeRestart(1.0);
                }
                cmd_len = 0;
            }
        } else if (cmd_len < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_len++] = c;
        }
    }
}

void reportSystemStatus() {
    char status_json[512];
    generateStatusReport(status_json, sizeof(status_json));
    LOG_I("系统状态: %s", status_json);
    if (tcp_client.connected()) {
        char command[600];
        snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=STATUS: %s\r\n", 
                uid, TOPIC_PC_STATE, status_json);
        tcp_client.print(command);
    }
}

void checkSystemAlerts() {
    if (current_status.heap_fragmentation > MEMORY_FRAGMENTATION_THRESHOLD && !memory_alert_sent) {
        sendSystemAlert("内存碎片率过高: %d%%", current_status.heap_fragmentation);
        memory_alert_sent = true;
    } else if (current_status.heap_fragmentation <= MEMORY_FRAGMENTATION_THRESHOLD) {
        memory_alert_sent = false;
    }
    if (current_status.wifi_rssi < RSSI_THRESHOLD && !rssi_alert_sent) {
        sendSystemAlert("WiFi信号弱: %d dBm", current_status.wifi_rssi);
        rssi_alert_sent = true;
    } else if (current_status.wifi_rssi >= RSSI_THRESHOLD) {
        rssi_alert_sent = false;
    }
    if ((current_status.wifi_fail_count > ERROR_COUNT_THRESHOLD || 
         current_status.tcp_fail_count > ERROR_COUNT_THRESHOLD) && !error_alert_sent) {
        sendSystemAlert("错误次数过多: WiFi=%u, TCP=%u", 
                 current_status.wifi_fail_count, current_status.tcp_fail_count);
        error_alert_sent = true;
    } else if (current_status.wifi_fail_count <= ERROR_COUNT_THRESHOLD && 
               current_status.tcp_fail_count <= ERROR_COUNT_THRESHOLD) {
        error_alert_sent = false;
    }
}

void sendSystemAlert(const char* format, ...) {
    char alert_msg[128];
    va_list args;
    va_start(args, format);
    vsnprintf(alert_msg, sizeof(alert_msg), format, args);
    va_end(args);
    LOG_W("系统告警: %s", alert_msg);
    if (tcp_client.connected()) {
        char command[200];
        snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=ALERT: %s\r\n", 
                uid, TOPIC_PC_STATE, alert_msg);
        tcp_client.print(command);
    }
}

void urlEncode(const char* src, char* dst, int dstSize) {
    const char *hex = "0123456789ABCDEF";
    int len = strlen(src);
    int j = 0;
    for (int i = 0; i < len && j < dstSize - 4; ++i) {
        char c = src[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            dst[j++] = c;
        } else {
            dst[j++] = '%';
            dst[j++] = hex[((unsigned char)c) >> 4];
            dst[j++] = hex[((unsigned char)c) & 0x0F];
        }
    }
    dst[j] = '\0';
}

//======================================================================
// ===================== 6. EEPROM 配置管理 ========================
//======================================================================
uint8_t calculateChecksum(const ConfigType& cfg) {
    uint8_t sum = 0;
    const uint8_t* p = (const uint8_t*)&cfg;
    for (size_t i = 0; i < offsetof(ConfigType, checksum); i++) {
        sum += p[i];
    }
    return ~sum;
}

void saveConfig() {
    LOG_I("正在保存配置到EEPROM...");
    config.checksum = calculateChecksum(config);
    EEPROM.begin(512);
    for (int i = 0; i < sizeof(config); i++) {
        EEPROM.write(i, *(p_config + i));
    }
    if (EEPROM.commit()) {
        LOG_I("EEPROM 保存成功.");
        last_eeprom_write_completion_time = millis();
    } else {
        LOG_E("EEPROM 保存失败!");
        current_status.eeprom_fail_count++;
    }
    EEPROM.end();
}

void restoreFactory() {
    LOG_W("正在恢复出厂设置...");
    config.magic = 0x00;
    strcpy(config.stassid, "");
    strcpy(config.stapsw, "");
    config.reboot = 0;
    config.checksum = 0;
    saveConfig();
    safeRestart(1.5);
    while (1) { delay(100); }
}

void loadConfig() {
    LOG_I("正在从EEPROM加载配置...");
    EEPROM.begin(512);
    for (int i = 0; i < sizeof(config); i++) {
        *(p_config + i) = EEPROM.read(i);
    }
    EEPROM.end();
    uint8_t stored_checksum = config.checksum;
    uint8_t calculated_checksum = calculateChecksum(config);
    if (config.magic != MAGIC_NUMBER || stored_checksum != calculated_checksum) {
        LOG_W("配置校验失败 (魔数: 0x%02X, 校验和: 0x%02X/0x%02X), 需配网", 
              config.magic, stored_checksum, calculated_checksum);
        config_flag = 1;
        return;
    }
    if (strlen(config.stassid) == 0 || strlen(config.stapsw) == 0) {
        LOG_W("配置中的SSID或密码为空, 需配网");
        config_flag = 1;
        return;
    }
    config.reboot++;
    LOG_I("重启计数器: %d", config.reboot);
    if (config.reboot >= 8) {
        LOG_E("重启次数过多，恢复出厂设置");
        restoreFactory();
    } else {
        eeprom_dirty = true;
        pending_change = CFG_REBOOT_COUNTER;
        last_eeprom_write_time = millis();
    }
}

void resetRebootCounter() {
    if (config.reboot > 0) {
        LOG_I("网络连接成功, 重置重启计数器");
        config.reboot = 0;
        eeprom_dirty = true;
        pending_change = CFG_REBOOT_COUNTER;
        last_eeprom_write_time = millis();
    }
}

void handleEepromWrites() {
    if (pending_change != CFG_NONE && 
        millis() - last_eeprom_write_time >= EEPROM_WRITE_DELAY &&
        millis() - last_eeprom_write_completion_time >= MIN_EEPROM_WRITE_INTERVAL) {
        config.checksum = calculateChecksum(config);
        EEPROM.begin(512);
        switch (pending_change) {
            case CFG_REBOOT_COUNTER:
                EEPROM.write(offsetof(ConfigType, reboot), config.reboot);
                EEPROM.write(offsetof(ConfigType, checksum), config.checksum);
                break;
            case CFG_WIFI_SETTINGS:
            case CFG_ALL:
                for (int i = 0; i < sizeof(config); i++) {
                    EEPROM.write(i, *(p_config + i));
                }
                break;
            default: break;
        }
        if (EEPROM.commit()) {
            LOG_D("配置已保存到EEPROM (类型: %d)", pending_change);
            eeprom_dirty = false;
            pending_change = CFG_NONE;
            last_eeprom_write_completion_time = millis();
        } else {
            LOG_E("EEPROM写入失败");
            current_status.eeprom_fail_count++;
            last_eeprom_write_time = millis();
        }
        EEPROM.end();
    }
}

//======================================================================
// ======================= 7. WiFi 与配网 ==========================
//======================================================================
void doSmartconfig() {
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    LOG_I("开始Airkiss智能配网 (3分钟超时)...");
    WiFi.beginSmartConfig();
    smartconfig_start_time = millis();
    current_status.status_flags |= STATUS_FLAG_SMARTCONFIG;
}

void handleSmartconfig() {
    LOG_OUTPUT.print(".");
    if (WiFi.smartConfigDone()) {
        LOG_I("\nAirkiss配网成功!");
        strcpy(config.stassid, WiFi.SSID().c_str());
        strcpy(config.stapsw, WiFi.psk().c_str());
        config.magic = MAGIC_NUMBER;
        config.reboot = 0;
        saveConfig(); 
        eeprom_dirty = false;
        pending_change = CFG_NONE;
        current_status.status_flags &= ~STATUS_FLAG_SMARTCONFIG;
        LOG_I("配置已成功写入EEPROM, 3秒后重启...");
        delay(3000);
        ESP.restart();
    }
    if (millis() - smartconfig_start_time > 3 * 60 * 1000) {
        LOG_E("\n智能配网超时, 正在重启");
        WiFi.stopSmartConfig();
        current_status.status_flags &= ~STATUS_FLAG_SMARTCONFIG;
        delay(1000);
        ESP.restart();
    }
    delay(500);
}

void initWiFi() {
    char temp_hostname[32];
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(temp_hostname, sizeof(temp_hostname), "%s_%02X%02X%02X", HOST_NAME, mac[3], mac[4], mac[5]);
    WiFi.hostname(temp_hostname);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    wifi_set_sleep_type(LIGHT_SLEEP_T);
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(config.stassid, config.stapsw);
        LOG_I("正在连接WiFi: %s", config.stassid);
    }
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        LOG_OUTPUT.print(F("."));
        attempts++;
        yield();
        if (attempts > 200) {
            LOG_E("\n连接WiFi失败，恢复出厂设置");
            current_status.wifi_fail_count++;
            restoreFactory();
        }
    }
    LOG_I("\nWiFi已连接! IP地址: %s", WiFi.localIP().toString().c_str());
    resetRebootCounter();
}

//======================================================================
// ====================== 8. 固件更新 (OTA) ========================
//======================================================================
void update_started() {
    LOG_I("OTA更新开始");
    current_status.status_flags |= STATUS_FLAG_OTA_UPDATING;
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "cmd=2&uid=%s&topic=%s&msg=updateing\r\n", uid, TOPIC_CONTROL);
    if(tcp_client.connected()) tcp_client.print(buffer);
    snprintf(buffer, sizeof(buffer), "cmd=2&uid=%s&topic=%s&msg=PCB升级中\r\n", uid, TOPIC_PC_STATE);
    if(tcp_client.connected()) tcp_client.print(buffer);
}

void update_finished() { LOG_I("OTA更新完成, 系统将重启"); current_status.status_flags &= ~STATUS_FLAG_OTA_UPDATING; }
void update_progress(int cur, int total) { LOG_D("OTA更新进度: %d%%", (cur * 100 / total)); }
void update_error(int err) { LOG_E("OTA更新错误: #%d", err); current_status.status_flags &= ~STATUS_FLAG_OTA_UPDATING; }

void updateBin() {
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);
    LOG_I("开始从URL更新固件: %s", FIRMWARE_URL_STR);
    t_httpUpdate_return ret = ESPhttpUpdate.update(ota_wifi_client, FIRMWARE_URL_STR);
    if(ret != HTTP_UPDATE_OK) {
        LOG_E("固件更新失败: %s", ESPhttpUpdate.getLastErrorString().c_str());
    }
}

//======================================================================
// ==================== 9. TCP通信与微信通知 =====================
//======================================================================
void sendWeChatNotification() {
    if (WiFi.status() != WL_CONNECTED) return;
    LOG_I("准备发送微信启动通知...");
    const char* deviceName = (DEBUG_MODE == 1) ? "PC开关测试板" : "PC开关";
    
    char message[128];
    snprintf(message, sizeof(message), "IP:%s MAC:%s Ver:%s", WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str(), FIRMWARE_VERSION);

    char* encoded_message = new char[sizeof(message) * 3];
    char* url = new char[512];
    if (encoded_message == nullptr || url == nullptr) {
        LOG_E("内存分配失败，无法发送微信通知");
        if(encoded_message) delete[] encoded_message;
        if(url) delete[] url;
        return;
    }
    
    urlEncode(message, encoded_message, sizeof(message) * 3);
    snprintf(url, 512, "http://apis.bemfa.com/vb/wechat/v1/wechatWarn?uid=%s&device=%s&message=%s", uid, deviceName, encoded_message);
    
    LOG_D("请求 URL: %s", url);
    http_client.begin(http_wifi_client, url);
    http_client.setTimeout(5000);
    int httpCode = http_client.GET();
    if (httpCode > 0) {
        LOG_I("微信通知发送完毕, HTTP响应码: %d", httpCode);
    } else {
        LOG_E("微信通知发送失败, 错误: %s", http_client.errorToString(httpCode).c_str());
    }
    http_client.end();
    
    delete[] encoded_message;
    delete[] url;
}

void sendSystemInfoToWeChat() {
    if (WiFi.status() != WL_CONNECTED) {
        LOG_W("微信推送状态失败: WiFi未连接");
        return;
    }
    LOG_I("准备通过微信推送系统状态...");

    char status_summary[256];
    const char* pc_state_str;
    switch (pc_on_state) {
        case PC_STATE_ON:    pc_state_str = "开机"; break;
        case PC_STATE_OFF:   pc_state_str = "关机"; break;
        case PC_STATE_SLEEP: pc_state_str = "休眠"; break;
        default:             pc_state_str = "未知"; break;
    }
    unsigned long uptime_days = current_status.uptime_seconds / 86400;
    unsigned long uptime_hours = (current_status.uptime_seconds % 86400) / 3600;
    snprintf(status_summary, sizeof(status_summary), 
        "PC状态:%s, WiFi信号:%ddBm, 已运行:%lu天%lu小时, 剩余内存:%uB, 版本:%s",
        pc_state_str, current_status.wifi_rssi, uptime_days, uptime_hours,
        current_status.free_heap, FIRMWARE_VERSION);

    char* encoded_summary = new char[sizeof(status_summary) * 3];
    char* url = new char[512];
    if (encoded_summary == nullptr || url == nullptr) {
        LOG_E("内存分配失败，无法发送微信通知");
        if(encoded_summary) delete[] encoded_summary;
        if(url) delete[] url;
        return;
    }

    urlEncode(status_summary, encoded_summary, sizeof(status_summary) * 3);
    snprintf(url, 512, "http://apis.bemfa.com/vb/wechat/v1/wechatWarn?uid=%s&device=PC开关状态&message=%s", uid, encoded_summary);
    LOG_D("请求 URL: %s", url);

    http_client.begin(http_wifi_client, url);
    http_client.setTimeout(5000);
    int httpCode = http_client.GET();
    if (httpCode > 0) {
        LOG_I("微信状态推送完毕, HTTP响应码: %d", httpCode);
    } else {
        LOG_E("微信状态推送失败, 错误: %s", http_client.errorToString(httpCode).c_str());
    }
    http_client.end();

    delete[] encoded_summary;
    delete[] url;
}

void reportInfoOnDemand() {
    if (is_reporting_info) {
        LOG_W("忽略'info'指令，因为上一个报告正在处理中。");
        return;
    }
    is_reporting_info = true;
    updateSystemStatus();
    LOG_I("收到'info'指令，开始全渠道状态报告...");

    char status_json[512];
    generateStatusReport(status_json, sizeof(status_json));
    LOG_I("串口系统状态 (JSON): %s", status_json);

    if (tcp_client.connected()) {
        // 为编码后的JSON和最终指令动态分配内存，防止溢出
        size_t encoded_len = strlen(status_json) * 3 + 1;
        char* encoded_json = new char[encoded_len];
        char* command = new char[encoded_len + 200]; // 额外200字节给其他参数

        if(encoded_json && command) {
            urlEncode(status_json, encoded_json, encoded_len);
            snprintf(command, encoded_len + 200, "cmd=2&uid=%s&topic=%s&msg=INFO: %s\r\n", uid, TOPIC_PC_STATE, encoded_json);
            tcp_client.print(command);
            LOG_I("已将详细状态推送到主题: %s", TOPIC_PC_STATE);
        } else {
            LOG_E("内存分配失败，无法将状态推送到主题");
        }
        
        if(encoded_json) delete[] encoded_json;
        if(command) delete[] command;
    } else {
        LOG_W("TCP未连接，无法将状态推送到主题");
    }

    sendSystemInfoToWeChat();
    is_reporting_info = false;
}

#if DEBUG_MODE == 1
void publishDebugInfo() {
    char message[128];
    snprintf(message, sizeof(message), "IP:%s MAC:%s Ver:%s", WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str(), FIRMWARE_VERSION);
    char command[256];
    snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=%s\r\n", uid, TOPIC_TEST_DATE, message);
    LOG_D("推送启动信息到 %s", TOPIC_TEST_DATE);
    if(tcp_client.connected()) tcp_client.print(command);
}
#endif

void startTCPClient() {
    LOG_I("尝试连接TCP服务器: %s:%d...", TCP_SERVER_ADDR, TCP_SERVER_PORT);
    if (tcp_client.connect(TCP_SERVER_ADDR, TCP_SERVER_PORT)) {
        LOG_I("TCP连接成功!");
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "cmd=1&uid=%s&topic=%s\r\n", uid, TOPIC_CONTROL);
        tcp_client.print(buffer);
        snprintf(buffer, sizeof(buffer), "cmd=1&uid=%s&topic=%s\r\n", uid, TOPIC_PC_STATE);
        tcp_client.print(buffer);
        #if DEBUG_MODE == 1
        snprintf(buffer, sizeof(buffer), "cmd=1&uid=%s&topic=%s\r\n", uid, TOPIC_TEST_DATE);
        tcp_client.print(buffer);
        publishDebugInfo();
        #endif
        is_tcp_previously_connected = true;
        pre_heartbeat_tick = millis();
        tcp_client.setNoDelay(true);
        tcp_connect_attempts = 0;
        tcp_reconnect_delay = 5000;
        tcp_buffer_len = 0;
        LOG_I("TCP已连接, 正在上报初始PC状态...");
        reportPcState();
        sendWeChatNotification();
    } else {
        LOG_E("TCP连接失败");
        tcp_client.stop();
        is_tcp_previously_connected = false;
        tcp_connect_attempts++;
        current_status.tcp_fail_count++;
    }
    pre_tcp_connect_tick = millis();
}

void doTCPClientTick() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!tcp_client.connected()) {
        if (is_tcp_previously_connected) {
            is_tcp_previously_connected = false;
            pre_tcp_connect_tick = millis();
            tcp_client.stop();
            LOG_W("TCP连接已断开");
            tcp_reconnect_delay = 1000;
        }
        if (millis() - pre_tcp_connect_tick > tcp_reconnect_delay) {
            if (tcp_connect_attempts >= 12) {
                LOG_E("连接TCP失败12次, 重启");
                ESP.restart();
            }
            startTCPClient();
            tcp_reconnect_delay = min(tcp_reconnect_delay * 2, 60000UL);
            LOG_I("下一次TCP重连在 %lu ms后", tcp_reconnect_delay);
        }
    } else {
        if (tcp_client.available()) {
            size_t max_read = sizeof(tcp_recv_buffer) - tcp_buffer_len - 1;
            if (max_read > 0) {
                size_t bytes_read = tcp_client.read((uint8_t*)tcp_recv_buffer + tcp_buffer_len, max_read);
                if (bytes_read > 0) {
                    tcp_buffer_len += bytes_read;
                    current_status.tcp_received_packets++;
                    tcp_recv_buffer[tcp_buffer_len] = '\0';
                }
            } else {
                LOG_W("TCP接收缓冲区已满，清空");
                tcp_buffer_len = 0;
            }
        }
        if (millis() - pre_heartbeat_tick >= KEEPALIVE_INTERVAL) {
            pre_heartbeat_tick = millis();
            tcp_client.print(F("cmd=0&msg=keep\r\n"));
            current_status.last_heartbeat_time = millis();
            current_status.tcp_sent_packets++;
            LOG_D("已发送心跳包");
        }
    }
    
    char* line_start = tcp_recv_buffer;
    char* line_end;
    while ((line_end = strchr(line_start, '\n')) != NULL) {
        *line_end = '\0';
        if (line_end > line_start && *(line_end - 1) == '\r') {
            *(line_end - 1) = '\0';
        }
        if (line_end - line_start < TCP_BUFFER_SIZE) {
            processTcpCommand(line_start);
        } else {
            LOG_W("忽略过长的TCP命令");
        }
        line_start = line_end + 1;
    }
    
    if (line_start > tcp_recv_buffer) {
        size_t remaining = tcp_buffer_len - (line_start - tcp_recv_buffer);
        memmove(tcp_recv_buffer, line_start, remaining);
        tcp_buffer_len = remaining;
        tcp_recv_buffer[tcp_buffer_len] = '\0';
    }
    
    if (tcp_buffer_len >= sizeof(tcp_recv_buffer) - 1) {
        LOG_E("TCP缓冲区溢出，清空");
        tcp_buffer_len = 0;
    }
}

void processTcpCommand(char* command) {
    LOG_D("收到TCP指令: %s", command);
    char topic_search_str[128];
    snprintf(topic_search_str, sizeof(topic_search_str), "&topic=%s", TOPIC_CONTROL);
    if (strstr(command, topic_search_str) != NULL) {
        if (strstr(command, "&msg=on") != NULL) { requestPcOn(); } 
        else if (strstr(command, "&msg=off") != NULL) { requestPcOff(); } 
        else if (strstr(command, "&msg=update") != NULL) { updateBin(); } 
        else if (strstr(command, "&msg=reboot") != NULL) { safeRestart(1.0); } 
        else if (strstr(command, "&msg=info") != NULL) { reportInfoOnDemand(); } 
        else if (strstr(command, "&msg=log=") != NULL) {
            const char* level_str = strstr(command, "log=") + 4;
            if (level_str) {
                uint8_t new_level = atoi(level_str);
                if (new_level <= 4) {
                    runtime_log_level = new_level;
                    LOG_I("通过远程指令将日志级别设置为: %d", runtime_log_level);
                    char resp_msg[64];
                    snprintf(resp_msg, sizeof(resp_msg), "Log level set to %d", new_level);
                    sendConfigResponse("success", resp_msg);
                } else {
                    LOG_W("无效的远程日志级别指令: %s", level_str);
                    sendConfigResponse("error", "Invalid log level (0-4)");
                }
            }
        } else if (strstr(command, "&msg=config:") != NULL) {
            const char* config_data = strstr(command, "config:") + 7;
            processRemoteConfig(config_data);
        } else if (strstr(command, "&msg=secure_config:") != NULL) {
            const char* config_cmd = strstr(command, "secure_config:") + 13;
            processSecureConfig(config_cmd);
        } else if (strstr(command, "&msg=get_config") != NULL) {
            processConfigQuery();
        } else if (strstr(command, "&msg=diag:") != NULL) {
            processDiagnosticCommand(command);
        }
    }
}

void processRemoteConfig(const char* config_data) {
    if (!config.remote_config) {
        LOG_W("远程配置已禁用");
        sendConfigResponse("error", "Remote config disabled");
        return;
    }
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, config_data);
    if (error) {
        LOG_E("配置解析失败: %s", error.c_str());
        sendConfigResponse("error", "Invalid config format");
        return;
    }
    ConfigType new_config = config;
    if (doc.containsKey("wifi")) {
        JsonObject wifi = doc["wifi"];
        strlcpy(new_config.stassid, wifi["ssid"] | "", sizeof(new_config.stassid));
        strlcpy(new_config.stapsw, wifi["password"] | "", sizeof(new_config.stapsw));
    }
    if (doc.containsKey("tcp")) {
        JsonObject tcp = doc["tcp"];
        strlcpy(new_config.tcp_server, tcp["server"] | "", sizeof(new_config.tcp_server));
        new_config.tcp_port = tcp["port"] | TCP_SERVER_PORT;
        new_config.heartbeat_interval = tcp["heartbeat"] | (KEEPALIVE_INTERVAL/1000);
        new_config.reconnect_delay = tcp["reconnect"] | 5;
    }
    if (doc.containsKey("thresholds")) {
        JsonObject thresholds = doc["thresholds"];
        new_config.on_threshold = thresholds["on"] | LED_ON_DUTY_CYCLE_THRESHOLD;
        new_config.off_threshold = thresholds["off"] | LED_OFF_DUTY_CYCLE_THRESHOLD;
        new_config.debounce_time = thresholds["debounce"] | (STATE_DEBOUNCE_TIME/1000);
    }
    if (doc.containsKey("timing")) {
        JsonObject timing = doc["timing"];
        new_config.short_press = timing["short_press"] | 800;
        new_config.long_press = timing["long_press"] | 8000;
    }
    if (doc.containsKey("system")) {
        JsonObject system = doc["system"];
        new_config.log_level = system["log_level"] | 3;
        new_config.report_interval = system["report_interval"] | (STATUS_REPORT_INTERVAL/60000);
        new_config.auto_reboot = system["auto_reboot"] | 10;
        new_config.debug_mode = system["debug_mode"] | DEBUG_MODE;
    }
    if (!validateConfig(new_config)) {
        LOG_E("新配置验证失败");
        sendConfigResponse("error", "Invalid config parameters");
        return;
    }
    applyNewConfig(new_config);
    config = new_config;
    saveConfig();
    sendConfigResponse("success", "Configuration updated");
}

void processSecureConfig(const char* command) {
    const char* password_start = strchr(command, ':');
    if (!password_start) {
        sendConfigResponse("error", "Missing password");
        return;
    }
    const char* data_start = strchr(password_start + 1, ':');
    if (!data_start) {
        sendConfigResponse("error", "Missing config data");
        return;
    }
    char password[32];
    size_t password_len = data_start - password_start - 1;
    if (password_len >= sizeof(password)) {
        sendConfigResponse("error", "Password too long");
        return;
    }
    strncpy(password, password_start + 1, password_len);
    password[password_len] = '\0';
    if (!checkConfigPermission(password)) {
        LOG_W("配置密码错误");
        sendConfigResponse("error", "Invalid password");
        return;
    }
    processRemoteConfig(data_start + 1);
}

void processConfigQuery() {
    char config_json[512];
    getCurrentConfig(config_json, sizeof(config_json));
    if (tcp_client.connected()) {
        char command[600];
        snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=CONFIG: %s\r\n", 
                uid, TOPIC_CONTROL, config_json);
        tcp_client.print(command);
    }
}

void processDiagnosticCommand(const char* command) {
    if (strstr(command, "&msg=diag:") != NULL) {
        const char* diag_cmd = strstr(command, "diag:") + 5;
        if (strcmp(diag_cmd, "memory") == 0) {
            uint32_t free_heap = ESP.getFreeHeap();
            uint32_t max_heap = ESP.getMaxFreeBlockSize();
            uint8_t frag = ESP.getHeapFragmentation();
            char diag_msg[128];
            snprintf(diag_msg, sizeof(diag_msg), 
                    "Memory: Free=%u, MaxBlock=%u, Fragmentation=%u%%",
                    free_heap, max_heap, frag);
            sendDiagnosticResponse(diag_msg);
        } else if (strcmp(diag_cmd, "network") == 0) {
            char diag_msg[256];
            snprintf(diag_msg, sizeof(diag_msg),
                    "Network: WiFi=%s, RSSI=%d dBm, TCP=%s, IP=%s",
                    WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                    WiFi.RSSI(),
                    tcp_client.connected() ? "Connected" : "Disconnected",
                    WiFi.localIP().toString().c_str());
            sendDiagnosticResponse(diag_msg);
        } else if (strcmp(diag_cmd, "status") == 0) {
            char status_json[512];
            generateStatusReport(status_json, sizeof(status_json));
            sendDiagnosticResponse(status_json);
        } else if (strcmp(diag_cmd, "reset_stats") == 0) {
            current_status.wifi_fail_count = 0;
            current_status.tcp_fail_count = 0;
            current_status.reboot_count = 0;
            sendDiagnosticResponse("Statistics reset");
        }
    }
}

void sendConfigResponse(const char* status, const char* message) {
    char response[128];
    snprintf(response, sizeof(response), "{\"status\":\"%s\",\"message\":\"%s\"}", status, message);
    if (tcp_client.connected()) {
        char command[200];
        snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=CONFIG_RESP: %s\r\n", 
                uid, TOPIC_CONTROL, response);
        tcp_client.print(command);
    }
}

void sendDiagnosticResponse(const char* message) {
    if (tcp_client.connected()) {
        char command[300];
        snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=DIAG: %s\r\n", 
                uid, TOPIC_CONTROL, message);
        tcp_client.print(command);
    }
}

void getCurrentConfig(char* buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size,
        "{\"wifi\":{\"ssid\":\"%s\"},"
        "\"tcp\":{\"server\":\"%s\",\"port\":%u,\"heartbeat\":%u},"
        "\"thresholds\":{\"on\":%u,\"off\":%u,\"debounce\":%u},"
        "\"timing\":{\"short_press\":%u,\"long_press\":%u},"
        "\"system\":{\"log_level\":%u,\"report_interval\":%u,\"auto_reboot\":%u,\"debug_mode\":%u},"
        "\"security\":{\"remote_config\":%u}}",
        config.stassid, config.tcp_server, config.tcp_port, config.heartbeat_interval,
        config.on_threshold, config.off_threshold, config.debounce_time,
        config.short_press, config.long_press, config.log_level,
        config.report_interval, config.auto_reboot, config.debug_mode, config.remote_config
    );
}

void applyNewConfig(const ConfigType& new_config) {
    runtime_log_level = new_config.log_level;
    if (new_config.debug_mode != DEBUG_MODE) {
        LOG_W("调试模式变更，需要重启才能完全生效");
    }
    if (strcmp(new_config.tcp_server, config.tcp_server) != 0 || 
        new_config.tcp_port != config.tcp_port) {
        LOG_I("TCP服务器配置变更，将重新连接");
        tcp_client.stop();
    }
    LOG_I("新配置已应用");
}

bool validateConfig(const ConfigType& cfg) {
    if (cfg.magic != MAGIC_NUMBER || cfg.checksum != calculateChecksum(cfg)) return false;
    if (strlen(cfg.stassid) == 0 || strlen(cfg.stapsw) == 0) return false;
    if (strlen(cfg.tcp_server) == 0 || cfg.tcp_port == 0) return false;
    if (cfg.on_threshold <= cfg.off_threshold) return false;
    if (cfg.on_threshold > 100 || cfg.off_threshold > 100) return false;
    if (cfg.heartbeat_interval == 0 || cfg.heartbeat_interval > 3600) return false;
    if (cfg.debounce_time > 60) return false;
    if (cfg.short_press == 0 || cfg.long_press == 0) return false;
    if (cfg.short_press >= cfg.long_press) return false;
    if (cfg.log_level > 4) return false;
    if (cfg.report_interval > 1440) return false;
    if (cfg.auto_reboot > 365) return false;
    return true;
}

bool checkConfigPermission(const char* password) {
    if (strlen(config.config_password) == 0) return true;
    return strcmp(password, config.config_password) == 0;
}

//======================================================================
// =================== 10. 核心电源管理模块 =======================
//======================================================================
void updatePcPowerState() {
    unsigned long current_time = millis();
    bool current_led_state = digitalRead(PIN_LED_STATE);
    if (current_time < last_led_read_time) {
        led_sampling_start_time = current_time;
        led_high_duration_ms = 0;
        last_led_state = current_led_state;
        last_led_read_time = current_time;
        return;
    }
    if (last_led_state == HIGH) {
        led_high_duration_ms += (current_time - last_led_read_time);
    }
    last_led_state = current_led_state;
    last_led_read_time = current_time;
    if (current_time - led_sampling_start_time >= LED_SAMPLING_PERIOD_MS) {
        int duty_cycle = (led_high_duration_ms * 100) / (current_time - led_sampling_start_time);
        PcPowerState new_state;
        if (duty_cycle >= LED_ON_DUTY_CYCLE_THRESHOLD) { new_state = PC_STATE_ON; } 
        else if (duty_cycle <= LED_OFF_DUTY_CYCLE_THRESHOLD) { new_state = PC_STATE_OFF; } 
        else { new_state = PC_STATE_SLEEP; }
        if (new_state != pc_on_state) {
            if (new_state != last_stable_state) {
                last_stable_state = new_state;
                state_change_time = current_time;
            } else if (current_time - state_change_time >= STATE_DEBOUNCE_TIME) {
                LOG_I("PC状态变化: %d -> %d (高电平占比: %d%%)", pc_on_state, new_state, duty_cycle);
                pc_on_state = new_state;
                reportPcState();
            }
        } else {
             last_stable_state = pc_on_state;
        }
        led_sampling_start_time = current_time;
        led_high_duration_ms = 0;
    }
}

void reportPcState() {
    const char* state_msg;
    switch (pc_on_state) {
        case PC_STATE_ON:    state_msg = "电脑已开机"; break;
        case PC_STATE_OFF:   state_msg = "电脑已关机"; break;
        case PC_STATE_SLEEP: state_msg = "电脑已休眠"; break;
        default:             state_msg = "状态未知";   break;
    }
    char state_str[64];
    snprintf(state_str, sizeof(state_str), "%s, v%s", state_msg, FIRMWARE_VERSION);
    LOG_I("上报PC状态: %s (状态值: %d)", state_str, pc_on_state);
    char command[256];
    snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=%s\r\n", uid, TOPIC_PC_STATE, state_str);
    if(tcp_client.connected()) {
        tcp_client.print(command);
    } else {
        LOG_W("TCP未连接，无法上报状态");
    }
}

void requestPcOn() {
    if ((pc_on_state == PC_STATE_OFF || pc_on_state == PC_STATE_SLEEP) && pc_control_state == IDLE) {
        LOG_I("指令ON (当前状态: %d): 空闲->开机", pc_on_state);
        current_action = ACTION_STARTUP;
        pc_control_state = PRESSING_BUTTON;
    } else {
        LOG_W("指令ON (当前状态: %d): 已忽略 (原因: PC已开机或状态机正忙)", pc_on_state);
        if (pc_control_state != IDLE) {
            LOG_I("取消当前操作");
            digitalWrite(PIN_PC_POWER, HIGH);
            pc_control_state = IDLE;
            current_action = ACTION_NONE;
        }
    }
}

void requestPcOff() {
    if (pc_on_state == PC_STATE_ON && pc_control_state == IDLE) {
        LOG_I("指令OFF (当前状态: %d): 空闲->关机", pc_on_state);
        current_action = ACTION_SHUTDOWN;
        pc_control_state = PRESSING_BUTTON;
    } else {
        LOG_W("指令OFF (当前状态: %d): 已忽略 (原因: PC未开机或状态机正忙)", pc_on_state);
    }
}

void handlePcControl() {
    if (pc_control_state == IDLE) return;
    switch (pc_control_state) {
        case PRESSING_BUTTON:
            LOG_D("动作: 按下电源键");
            digitalWrite(PIN_PC_POWER, LOW);
            action_start_time = millis();
            if (current_action == ACTION_SHUTDOWN) {
                shutdown_request_time = millis();
            }
            pc_control_state = WAITING_FOR_RESULT;
            break;
        case WAITING_FOR_RESULT:
            {
                unsigned long press_duration = millis() - action_start_time;
                if (digitalRead(PIN_PC_POWER) == LOW) {
                    bool should_release = false;
                    if (current_action == ACTION_STARTUP && press_duration >= 800) {
                        LOG_D("动作: 释放电源键(开机)");
                        should_release = true;
                    } else if (current_action == ACTION_SHUTDOWN && press_duration >= 1000) {
                        LOG_D("动作: 释放电源键(关机)");
                        should_release = true;
                    } else if (current_action == ACTION_FORCE_SHUTDOWN && press_duration >= 8000) {
                        LOG_W("动作: 释放电源键(强关)");
                        should_release = true;
                    }
                    if (should_release) {
                        digitalWrite(PIN_PC_POWER, HIGH);
                    }
                }
                if (shutdown_request_time > 0 && pc_on_state == PC_STATE_OFF) {
                    LOG_I("电脑已成功关机 -> 状态机复位");
                    shutdown_request_time = 0;
                    current_action = ACTION_NONE;
                    pc_control_state = IDLE;
                } else if (shutdown_request_time > 0 && millis() - shutdown_request_time > 300000) {
                    LOG_W("5分钟未关机，执行强关");
                    shutdown_request_time = 0;
                    current_action = ACTION_FORCE_SHUTDOWN;
                    pc_control_state = PRESSING_BUTTON;
                } else if ((current_action == ACTION_STARTUP && pc_on_state == PC_STATE_ON) || 
                          (current_action == ACTION_FORCE_SHUTDOWN && pc_on_state == PC_STATE_OFF)) {
                    LOG_I("动作完成 -> 状态机复位");
                    current_action = ACTION_NONE;
                    pc_control_state = IDLE;
                } else if (press_duration > 15000 && shutdown_request_time == 0) {
                    LOG_W("动作超时 (非关机等待) -> 状态机复位");
                    digitalWrite(PIN_PC_POWER, HIGH);
                    current_action = ACTION_NONE;
                    pc_control_state = IDLE;
                }
            }
            break;
        default:
            pc_control_state = IDLE;
            break;
    }
}

//======================================================================
// =================== 11. 系统状态监控 =======================
//======================================================================
void initSystemStatus() {
    memset(&current_status, 0, sizeof(SystemStatus));
    strcpy(current_status.firmware_version, FIRMWARE_VERSION);
    current_status.last_boot_time = millis();
    current_status.last_heartbeat_time = millis();
}

void updateSystemStatus() {
    current_status.uptime_seconds = millis() / 1000;
    current_status.free_heap = ESP.getFreeHeap();
    current_status.heap_fragmentation = ESP.getHeapFragmentation();
    current_status.wifi_rssi = WiFi.RSSI();
    current_status.wifi_connected = (WiFi.status() == WL_CONNECTED);
    current_status.tcp_connected = tcp_client.connected();
    current_status.pc_state = pc_on_state;
    current_status.control_state = pc_control_state;
    current_status.last_action = current_action;
    current_status.last_action_time = action_start_time;
    current_status.status_flags = 0;
    if (current_status.heap_fragmentation > 70) {
        current_status.status_flags |= STATUS_FLAG_LOW_MEMORY;
    }
    if (current_status.wifi_rssi < -80) {
        current_status.status_flags |= STATUS_FLAG_WEAK_SIGNAL;
    }
}

void generateStatusReport(char* buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size,
        "{\"firmware\":\"%s\","
        "\"uptime\":%lu,"
        "\"memory\":{\"free\":%u,\"fragmentation\":%u},"
        "\"network\":{\"wifi\":%d,\"tcp\":%d,\"rssi\":%d},"
        "\"pc\":{\"state\":%d,\"control\":%d},"
        "\"errors\":{\"reboot\":%u,\"wifi\":%u,\"tcp\":%u},"
        "\"flags\":%u}",
        current_status.firmware_version, current_status.uptime_seconds,
        current_status.free_heap, current_status.heap_fragmentation,
        current_status.wifi_connected ? 1 : 0, current_status.tcp_connected ? 1 : 0,
        current_status.wifi_rssi, current_status.pc_state, current_status.control_state,
        current_status.reboot_count, current_status.wifi_fail_count, current_status.tcp_fail_count,
        current_status.status_flags
    );
}

void sendStatusReport() {
    char status_json[512];
    generateStatusReport(status_json, sizeof(status_json));
    LOG_D("状态报告: %s", status_json);
    if (tcp_client.connected()) {
        char command[600];
        snprintf(command, sizeof(command), "cmd=2&uid=%s&topic=%s&msg=STATUS: %s\r\n", 
                uid, TOPIC_PC_STATE, status_json);
        tcp_client.print(command);
    }
}

void handleStatusMonitoring() {
    updateSystemStatus();
    if (millis() - last_status_report_time >= STATUS_REPORT_INTERVAL) {
        sendStatusReport();
        last_status_report_time = millis();
    }
}

//======================================================================
// ==================== 12. 系统初始化和主循环 =====================
//======================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    
    LOG_I("\n\n--- 电脑管家 v%s 启动中 ---", FIRMWARE_VERSION);
    #if DEBUG_MODE == 1
    LOG_W("--- **调试模式已激活** ---");
    #endif

    pinMode(PIN_PC_POWER, OUTPUT);
    digitalWrite(PIN_PC_POWER, HIGH);
    pinMode(PIN_LED_STATE, INPUT);
    
    ESP.wdtEnable(WDTO_8S);
    
    initSystemStatus();
    loadConfig();
    
    if (config_flag == 1) {
        doSmartconfig();
    } else {
        initWiFi();
        startTCPClient();
    }
    
    last_10d_reboot_time = millis();
    last_status_report_time = millis();
    led_sampling_start_time = millis();
    last_led_read_time = millis();
    last_led_state = digitalRead(PIN_LED_STATE);
    last_stable_state = pc_on_state;
}

void loop() {
    if (config_flag == 1) {
        handleSmartconfig();
        return;
    }

    ESP.wdtFeed();
    
    handleSerialCommands();
    doTCPClientTick();
    updatePcPowerState();
    handlePcControl();
    handleEepromWrites();
    
    handleStatusMonitoring();
    checkSystemAlerts();

    if (millis() - last_10d_reboot_time > 10UL * 24 * 60 * 60 * 1000) {
        LOG_I("已稳定运行10天, 计划性重启");
        ESP.restart();
    }
    
    delay(50);
    wifi_set_sleep_type(LIGHT_SLEEP_T);
}
