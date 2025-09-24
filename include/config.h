/**
 * Cấu hình cho hệ thống Theo dõi và Chống trộm Xe ESP32
 *
 * Cập nhật lần cuối: 2025-06-18 17:53:40
 * Người phát triển: nguongthienTieu
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Phiên bản ứng dụng
#define APP_VERSION "1.3.1"
#define DEVELOPER "nguongthienTieu"

// Cấu hình chân kết nối
#define BUZZER_PIN 13
#define BATTERY_PIN 35
#define MPU_INT_PIN 2
#define RF_DATA_PIN 4
#define RF_POWER_PIN 25
#define GPS_POWER_PIN 33
// SIM_PWRKEY_PIN được giữ lại để tương thích với mã hiện có,
// nhưng sẽ không được sử dụng để điều khiển nguồn
#define SIM_PWRKEY_PIN 15

// Thông số cho pin
#define BATTERY_MAX_VOLTAGE 8.4    // 2 cell 18650 Li-ion (4.2V * 2)
#define BATTERY_MIN_VOLTAGE 6.0    // Điện áp ngắt (3.0V * 2)
#define LOW_BATTERY_THRESHOLD 20   // Cảnh báo khi pin dưới 20%
#define HIGH_BATTERY_THRESHOLD 80  // Đặt lại cảnh báo khi pin trên 80%

// Mã RF433
#define USER_RF_CODE 123456      // Mã nhận diện người dùng
#define NETWORK_ACTIVATE_CODE 789012 // Mã kích hoạt mạng
#define RESET_RF_CODE 345678     // Mã reset

// Mã SMS
#define SMS_SOS_CODE "SOS"
#define SMS_STATUS_CODE "STATUS"
#define SMS_GPS_CODE "GPS"
#define SMS_STOP_CODE "STOP"
#define SMS_SLEEP_CODE "SLEEP"
#define SMS_WAKE_CODE "WAKE"

// Thông số thời gian
#define RF_ACTIVE_DURATION 1000     // Thời gian kích hoạt RF (ms)
#define RF_CHECK_INTERVAL 50000    // Kiểm tra RF mỗi 10 giây khi ngủ
#define POSITION_UPDATE_INTERVAL 300000 // Cập nhật vị trí mỗi 5 phút
#define NETWORK_SESSION_TIMEOUT 300000 // Phiên kết nối mạng hết hạn sau 5 phút
#define WIFI_TIMEOUT 10000         // Timeout kết nối WiFi (10 giây)
#define GPS_STAGE2_INTERVAL 30000  // Cập nhật GPS mỗi 30 giây trong giai đoạn 2
#define WARNING_INACTIVITY_TIMEOUT 120000 // Thời gian không hoạt động để thoát giai đoạn 1 (2 phút)
#define STAGE2_TIMEOUT 60000       // Thời gian tối đa cho giai đoạn 2 (1 phút)
#define STAGE2_MAX_CHECKS 3        // Số lần kiểm tra tối đa trong giai đoạn 2

// Ngưỡng cảnh báo
#define ACCEL_THRESHOLD 1.0        // Ngưỡng gia tốc để phát hiện chuyển động (m/s²)
#define DISTANCE_THRESHOLD 10.0    // Ngưỡng khoảng cách để phát hiện di chuyển (mét)

// AT commands cho GPS
#define GPS_SLEEP_CMD "$PMTK161,0*28\r\n"  // Command đặt GPS vào chế độ ngủ
#define GPS_WAKE_CMD "$PMTK101*32\r\n"     // Command hot start để đánh thức GPS

// Thông tin Blynk và WiFi
#define BLYNK_AUTH_TOKEN "_SBxmQ0A765Sob-8a3H-lCX7nT2yzriY"
#define BLYNK_SERVER "blynk.dke.vn"
#define BLYNK_PORT 8888
#define WIFI_SSID "116B9"
#define WIFI_PASS "12356789"

// Thông tin SIM
#define PHONE_NUMBER "+84815643460"
#define SIM_APN "m3-world"

// Các chân ảo Blynk
#define VPIN_BATTERY V0
#define VPIN_OWNER V1
#define VPIN_ALARM_STATE V2
#define VPIN_MOTION V3
#define VPIN_CONNECTION V4
#define VPIN_CONTROL_TEST V5

// Cấu hình cho hệ thống báo động
#define WARNING_BEEP_INTERVAL 500
#define WARNING_BEEPS_PER_SEQUENCE 3
#define WARNING_SEQUENCE_COUNT 3
#define WARNING_SEQUENCE_DELAY 120000

// Trạng thái báo động
enum AlarmStage {
    STAGE_NONE = 0,       // Không có báo động
    STAGE_WARNING = 1,    // Cảnh báo - Tiếng bíp
    STAGE_ALERT = 2,      // Báo động - Còi liên tục
    STAGE_TRACKING = 3    // Theo dõi - Gửi vị trí
};

// Trạng thái lỗi
enum ErrorState {
    ERROR_NONE = 0,      // Không có lỗi
    ERROR_WIFI = 1,      // Lỗi WiFi
    ERROR_SIM = 2,       // Lỗi module SIM
    ERROR_GPS = 3,       // Lỗi module GPS
    ERROR_MPU = 4,       // Lỗi cảm biến MPU6050
    ERROR_CRITICAL = 5   // Lỗi nghiêm trọng
};

// Loại SMS cảnh báo (cho cooldown)
enum SMSAlertType {
    SMS_ALERT_ALARM = 0,        // Cảnh báo xâm nhập
    SMS_ALERT_MOVEMENT = 1,     // Cảnh báo di chuyển
    SMS_ALERT_POSITION = 2,     // Cập nhật vị trí
    SMS_ALERT_GPS_LOST = 3,     // Mất tín hiệu GPS
    SMS_ALERT_LOW_BATTERY = 4,  // Pin thấp
    SMS_ALERT_MODULE_RESET = 5, // Reset module
    SMS_ALERT_SYSTEM_ERROR = 6  // Lỗi hệ thống
};

// Cấu hình hệ thống từ điển tiếng Việt
#define DICT_ENABLE_COMPOUND_WORDS true    // Bật hỗ trợ từ ghép
#define DICT_ENABLE_DEAD_WORDS true        // Bật quản lý từ chết
#define DICT_ENABLE_SMS_COMMANDS true      // Bật lệnh SMS cho từ điển
#define DICT_MAX_WORDS 1000                // Tối đa số từ (tuỳ theo RAM)
#define DICT_MAX_COMPOUND_WORDS 500        // Tối đa từ ghép
#define DICT_DEAD_WORD_TIMEOUT 86400000    // 24h timeout cho từ chết
#define DICT_AUTO_CLEANUP_INTERVAL 86400000 // Dọn dẹp tự động mỗi 24h

// Mã SMS cho từ điển
#define SMS_DICT_STATS_CODE "DICT STATS"
#define SMS_DICT_ADD_CODE "DICT ADD"
#define SMS_DICT_COMPOUND_CODE "DICT COMPOUND"
#define SMS_DICT_DEAD_CODE "DICT DEAD"
#define SMS_DICT_REVIVE_CODE "DICT REVIVE"
#define SMS_DICT_SEARCH_CODE "DICT SEARCH"
#define SMS_DICT_CLEANUP_CODE "DICT CLEANUP"

// Biến toàn cục để lưu trạng thái hiện tại
extern ErrorState currentError;

#endif // CONFIG_H