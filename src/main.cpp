/**
 * Hệ thống Theo dõi và Chống trộm Xe ESP32 - Phiên bản Tích Hợp 2025
 * Sử dụng Blynk v1 (Legacy) - PlatformIO Edition
 *
 * Tính năng:
 * - Kết nối chỉ theo yêu cầu (WiFi/4G) khi kích hoạt qua RF433
 * - Phát hiện chuyển động sử dụng cảm biến gia tốc MPU6050
 * - Theo dõi vị trí GPS (ATGM336H) chỉ khi cần thiết (chỉ gửi qua SMS)
 * - Nhận dạng người dùng RF433MHz với 3 mã khác nhau (user, network, reset)
 * - Quản lý năng lượng tối ưu để kéo dài thời lượng pin
 * - Tích hợp Blynk thông qua WiFi hoặc 4G (A7682S) (không gửi GPS)
 * - Báo động 3 giai đoạn theo cấu hình cụ thể
 * - Cảnh báo pin thấp qua SMS khi dưới 20%
 * - Chế độ ngủ cho module GPS ATGM336H
 * - Chế độ Light Sleep và Deep Sleep cho ESP32
 * - Watchdog timer để tự động reset khi phát hiện treo
 * - Chống spam SMS với cơ chế cooldown
 * - Quản lý nguồn MPU6050 khi có chủ sở hữu
 * - Tích hợp bộ lọc Kalman để cải thiện độ chính xác GPS và phát hiện chuyển động
 *
 * Cập nhật lần cuối: 2025-06-18 19:07:47
 * Người phát triển: nguongthienTieu
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TinyGPS++.h>
#include <RCSwitch.h>
#include <Preferences.h>  // Thư viện Preferences
#include "config.h"
#include "vietnamese_dictionary.h"  // Hệ thống từ điển tiếng Việt
#include "esp_sleep.h"       // Thư viện ESP sleep
#include "driver/rtc_io.h"   // Thư viện cho RTC IO
#include "esp_task_wdt.h"    // Thư viện Watchdog Timer

// QUAN TRỌNG: Ghi đè định nghĩa LED_PIN để vô hiệu hóa
#undef LED_PIN
#define LED_PIN -1        // Đánh dấu LED không được sử dụng (vô hiệu hóa)

// Thêm biến kiểm soát thời gian log GPS
unsigned long lastGpsSerialLog = 0;
#define GPS_LOG_INTERVAL 10000  // Log mỗi 10 giây

// Forward declarations of global variables
bool mpuSleepState = false;     // Trạng thái sleep của MPU6050

// Class definitions for Kalman filters
class GPSKalmanFilter {
private:
    // Tham số bộ lọc
    float _measurementUncertainty;  // Độ không chắc chắn đo lường
    float _estimationUncertainty;   // Độ không chắc chắn ước lượng
    float _processNoise;            // Nhiễu quá trình

    // Trạng thái hiện tại
    float _latEstimate;             // Ước lượng vĩ độ
    float _lngEstimate;             // Ước lượng kinh độ
    float _latUncertainty;          // Độ không chắc chắn vĩ độ
    float _lngUncertainty;          // Độ không chắc chắn kinh độ
    bool _initialized;              // Đã khởi tạo chưa

public:
    // Hàm khởi tạo với các tham số mặc định
    GPSKalmanFilter(float measurementUncertainty = 0.1,
                    float estimationUncertainty = 1.0,
                    float processNoise = 0.01)
            : _measurementUncertainty(measurementUncertainty),
              _estimationUncertainty(estimationUncertainty),
              _processNoise(processNoise),
              _latEstimate(0.0),
              _lngEstimate(0.0),
              _latUncertainty(estimationUncertainty),
              _lngUncertainty(estimationUncertainty),
              _initialized(false) {}

    // Thiết lập lại bộ lọc
    void reset(float initialLat = 0.0, float initialLng = 0.0) {
        _latEstimate = initialLat;
        _lngEstimate = initialLng;
        _latUncertainty = _estimationUncertainty;
        _lngUncertainty = _estimationUncertainty;
        _initialized = (initialLat != 0.0 || initialLng != 0.0);
    }

    // Cập nhật bộ lọc với dữ liệu mới
    void updatePosition(float measuredLat, float measuredLng, float* filteredLat, float* filteredLng) {
        // Nếu chưa khởi tạo, sử dụng vị trí đầu tiên như là điểm xuất phát
        if (!_initialized && (measuredLat != 0.0 || measuredLng != 0.0)) {
            _latEstimate = measuredLat;
            _lngEstimate = measuredLng;
            _initialized = true;
        }

        if (_initialized) {
            // Cập nhật vĩ độ
            _latUncertainty += _processNoise;
            float kalmanGainLat = _latUncertainty / (_latUncertainty + _measurementUncertainty);
            _latEstimate += kalmanGainLat * (measuredLat - _latEstimate);
            _latUncertainty = (1 - kalmanGainLat) * _latUncertainty;

            // Cập nhật kinh độ
            _lngUncertainty += _processNoise;
            float kalmanGainLng = _lngUncertainty / (_lngUncertainty + _measurementUncertainty);
            _lngEstimate += kalmanGainLng * (measuredLng - _lngEstimate);
            _lngUncertainty = (1 - kalmanGainLng) * _lngUncertainty;
        }

        // Trả về kết quả đã lọc
        *filteredLat = _latEstimate;
        *filteredLng = _lngEstimate;
    }
};

// Bộ lọc Kalman cho MPU6050
class MPUKalmanFilter {
private:
    // Tham số bộ lọc
    float _measurementUncertainty;  // Độ không chắc chắn đo lường
    float _estimationUncertainty;   // Độ không chắc chắn ước lượng
    float _processNoise;            // Nhiễu quá trình

    // Trạng thái hiện tại
    float _xEstimate;               // Ước lượng gia tốc trục x
    float _yEstimate;               // Ước lượng gia tốc trục y
    float _zEstimate;               // Ước lượng gia tốc trục z
    float _xUncertainty;            // Độ không chắc chắn trục x
    float _yUncertainty;            // Độ không chắc chắn trục y
    float _zUncertainty;            // Độ không chắc chắn trục z
    bool _initialized;              // Đã khởi tạo chưa

public:
    // Hàm khởi tạo với các tham số mặc định
    MPUKalmanFilter(float measurementUncertainty = 0.5,
                    float estimationUncertainty = 1.0,
                    float processNoise = 0.05)
            : _measurementUncertainty(measurementUncertainty),
              _estimationUncertainty(estimationUncertainty),
              _processNoise(processNoise),
              _xEstimate(0.0),
              _yEstimate(0.0),
              _zEstimate(9.8), // Gia tốc trọng trường
              _xUncertainty(estimationUncertainty),
              _yUncertainty(estimationUncertainty),
              _zUncertainty(estimationUncertainty),
              _initialized(false) {}

    // Thiết lập lại bộ lọc
    void reset() {
        _xEstimate = 0.0;
        _yEstimate = 0.0;
        _zEstimate = 9.8; // Gia tốc trọng trường
        _xUncertainty = _estimationUncertainty;
        _yUncertainty = _estimationUncertainty;
        _zUncertainty = _estimationUncertainty;
        _initialized = false;
    }

    // Cập nhật bộ lọc với dữ liệu mới
    void updateAcceleration(float measuredX, float measuredY, float measuredZ,
                            float* filteredX, float* filteredY, float* filteredZ) {
        // Nếu chưa khởi tạo, sử dụng giá trị đầu tiên
        if (!_initialized) {
            _xEstimate = measuredX;
            _yEstimate = measuredY;
            _zEstimate = measuredZ;
            _initialized = true;
        }

        if (_initialized) {
            // Cập nhật trục x
            _xUncertainty += _processNoise;
            float kalmanGainX = _xUncertainty / (_xUncertainty + _measurementUncertainty);
            _xEstimate += kalmanGainX * (measuredX - _xEstimate);
            _xUncertainty = (1 - kalmanGainX) * _xUncertainty;

            // Cập nhật trục y
            _yUncertainty += _processNoise;
            float kalmanGainY = _yUncertainty / (_yUncertainty + _measurementUncertainty);
            _yEstimate += kalmanGainY * (measuredY - _yEstimate);
            _yUncertainty = (1 - kalmanGainY) * _yUncertainty;

            // Cập nhật trục z
            _zUncertainty += _processNoise;
            float kalmanGainZ = _zUncertainty / (_zUncertainty + _measurementUncertainty);
            _zEstimate += kalmanGainZ * (measuredZ - _zEstimate);
            _zUncertainty = (1 - kalmanGainZ) * _zUncertainty;
        }

        // Trả về kết quả đã lọc
        *filteredX = _xEstimate;
        *filteredY = _yEstimate;
        *filteredZ = _zEstimate;
    }
};

// Khởi tạo bộ lọc Kalman
// Tham số: độ không chắc chắn đo lường, độ không chắc chắn ước lượng, nhiễu quá trình
GPSKalmanFilter gpsKalman(0.1, 1.0, 0.01);
MPUKalmanFilter mpuKalman(0.5, 1.0, 0.05);

// Forward declaration of functions used in other files
void sendSmsAlert(const char* message);

// Now include the utility headers
#include "sms_cooldown.h"         // Hệ thống quản lý cooldown cho SMS
#include "mpu_power_management.h"  // Hệ thống quản lý nguồn MPU6050

// Định nghĩa thời gian cho các chế độ ngủ
#define LIGHT_SLEEP_TIMEOUT 30000      // 30 giây không hoạt động -> light sleep
#define DEEP_SLEEP_TIMEOUT 600000      // 10 phút không hoạt động -> deep sleep
#define SLEEP_WAKEUP_CHECK_INTERVAL 3000 // 3 giây kiểm tra điều kiện để vào sleep

// Thời gian timeout cho các module
#define WIFI_MODULE_TIMEOUT 30000      // 30 giây timeout cho WiFi
#define SIM_MODULE_TIMEOUT 30000       // 30 giây timeout cho SIM
#define GPS_MODULE_TIMEOUT 60000       // 60 giây timeout cho GPS

// Cấu hình watchdog timer
#define WDT_TIMEOUT 30               // 30 giây timeout cho watchdog
#define MAX_RESET_ATTEMPTS 3         // Số lần thử reset module trước khi reset toàn bộ

// Timer IDs
int motionTimerId = -1;
int rfTimerId = -1;
int batteryTimerId = -1;
int wifiTimerId = -1;
int blynkSyncTimerId = -1;
int sleepCheckTimerId = -1;
int moduleCheckTimerId = -1;       // Timer để kiểm tra các module

// Các interval gốc và hiện tại
const unsigned long MOTION_BASE_INTERVAL = 1000;     // 1 giây
const unsigned long RF_BASE_INTERVAL = 5000;         // 5 giây
const unsigned long BATTERY_BASE_INTERVAL = 900000;  // 15 phút
const unsigned long WIFI_BASE_INTERVAL = 30000;      // 30 giây
const unsigned long BLYNK_BASE_INTERVAL = 60000;     // 1 phút
const unsigned long SLEEP_CHECK_BASE_INTERVAL = 3000; // 3 giây
const unsigned long MODULE_CHECK_INTERVAL = 15000;   // 15 giây

// Interval hiện tại (có thể thay đổi theo trạng thái)
unsigned long motionInterval = MOTION_BASE_INTERVAL;
unsigned long rfInterval = RF_BASE_INTERVAL;
unsigned long batteryInterval = BATTERY_BASE_INTERVAL;
unsigned long wifiInterval = WIFI_BASE_INTERVAL;
unsigned long blynkSyncInterval = BLYNK_BASE_INTERVAL;
unsigned long sleepCheckInterval = SLEEP_CHECK_BASE_INTERVAL;

// Biến theo dõi module
unsigned long lastWifiResponse = 0;
unsigned long lastSimResponse = 0;
unsigned long lastGpsResponse = 0;
bool wifiResponding = true;
bool simResponding = true;
bool gpsResponding = true;

// Biến toàn cục
bool wifiConnected = false;
bool simActive = false;
bool alarmState = false;
bool ownerPresent = false;
bool motionDetected = false;
bool vehicleMoving = false;
bool gpsActive = false;
bool gpsPowerState = true;      // GPS luôn được cấp nguồn (true)
bool rfEnabled = true;          // RF433 enabled by default
bool rfPowerState = true;      // RF luôn được cấp nguồn (true)
bool hadValidFix = false;       // Đã từng có tín hiệu GPS hợp lệ
bool lowBatteryAlerted = false; // Theo dõi đã gửi cảnh báo pin thấp hay chưa
bool signalLost = false;        // Đánh dấu khi mất tín hiệu
bool blynkOverSIM = false;      // Đang sử dụng Blynk qua 4G
float lastLat = 0, lastLng = 0;
float currentLat = 0, currentLng = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastGpsUpdate = 0;
unsigned long lastRfCheck = 0;
unsigned long lastRfActivation = 0;
unsigned long alarmStartTime = 0;
unsigned long lastDistanceCheck = 0;
unsigned long motionDetectedTime = 0;
unsigned long sequenceEndTime = 0;
unsigned long gpsActivationTime = 0;
unsigned long lastBatteryCheck = 0;
unsigned long lastBlynk4GSync = 0;  // Thời gian cuối cùng đồng bộ Blynk qua 4G
unsigned long lastActivityTime = 0;  // Thời gian hoạt động cuối cùng - dùng cho sleep mode
unsigned long lastSleepTime = 0;     // Thời điểm vào sleep
float batteryVoltage = 0;
int batteryPercentage = 0;
int stage2GpsUpdateCount = 0;    // Đếm số lần cập nhật GPS ở giai đoạn 2
bool manualReset = false;        // Biến để theo dõi lệnh reset thủ công
bool isWakingUp = false;         // Đánh dấu đang trong quá trình thức dậy từ sleep
esp_sleep_wakeup_cause_t wakeupCause; // Nguyên nhân thức dậy từ sleep

// Biến cho kết nối theo yêu cầu
bool networkModeActive = false;          // Trạng thái kết nối mạng theo yêu cầu
unsigned long networkActivationTime = 0;  // Thời điểm kích hoạt mạng
bool usingSIM = false;                   // Đang sử dụng kết nối SIM

// Biến cho trạng thái báo động
AlarmStage alarmStage = STAGE_NONE;
AlarmStage previousAlarmStage = STAGE_NONE;
unsigned long lastBeepTime = 0;
bool notificationSent = false;
bool initialPositionSet = false;

// Biến mới cho trạng thái báo động cập nhật
int beepIndex = 0;                   // Chỉ số tiếng bíp hiện tại trong chuỗi
int sequenceIndex = 0;               // Chỉ số chuỗi bíp hiện tại
unsigned long lastBeepEndTime = 0;   // Thời điểm kết thúc tiếng bíp cuối cùng
unsigned long lastMotionTime = 0;    // Thời điểm phát hiện chuyển động cuối cùng
bool inBeepSequence = false;         // Đang trong chuỗi bíp
bool sequenceCompleted = false;      // Đã hoàn thành chuỗi bíp
int stage2CheckCount = 0;            // Số lần kiểm tra trong giai đoạn 2
float initialStage2Lat = 0, initialStage2Lng = 0; // Vị trí ban đầu trong giai đoạn 2
bool stage2PositionSet = false;      // Đã thiết lập vị trí ban đầu cho giai đoạn 2

// Biến mới để đếm số lần phát hiện chuyển động trong giai đoạn 1
int motionDetectionCount = 0;
unsigned long firstMotionTime = 0;
bool initialBeepsDone = false;

// Biến trạng thái hệ thống
bool isLowBatteryMode = false;
ErrorState currentError = ERROR_NONE;

// Biến toàn cục cho RTC
int bootCount = 0;
int wifiResetCount = 0;
int simResetCount = 0;
int gpsResetCount = 0;

// Khai báo các đối tượng
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
RCSwitch rfReceiver = RCSwitch();
BlynkTimer timer;

// Khai báo UART
HardwareSerial SerialGPS(1);  // UART1 cho GPS ATGM336H
HardwareSerial SerialSIM(2);  // UART2 cho module SIM A7682S

// ----------------------
// Khai báo prototype hàm
// ----------------------
// Timer management functions
void setupBaseTimers();
void deleteTimerIfExists(int &timerId);
void adjustTimersForLowBattery(bool isLowBattery);
void adjustTimersForAlarm(AlarmStage stage);
void manageWifiTimer(bool activate);
void manageBlynkSyncTimer(bool activate);
void prepareTimersForSleep();
void restoreTimersAfterWakeup();
void resetAllTimers();

// Watchdog and module reset functions
void setupWatchdog();
void feedWatchdog();
void checkModuleStatus();
void resetWiFiModule();
void resetSimModule();
void resetGpsModule();
void performHardwareReset();

// Sleep mode functions
void goToLightSleep(uint64_t sleepTime);
void goToDeepSleep(uint64_t sleepTime);
void checkSleepConditions();
void setupWakeupSources();
void handleWakeup();
bool canEnterLightSleep();
bool canEnterDeepSleep();
void prepareForSleep();
void restoreFromSleep();
void updateActivityTime();

// Existing function prototypes
void wakeGPSFromSleep();
void updateGpsPosition();
void putGPSToSleep();
void updateBlynkData();
void setup4GConnection();
void acknowledgeUserPresence();
void activateRF();
void deactivateRF();
void disableRF433();
void activateSim();
void deactivateSim();
void wakeSimFromSleep();
bool checkSimResponse();
void activateGPS();
void deactivateGPS();
void sendSmsAlert(const char* message);
float getBatteryVoltage();
int calculateBatteryPercentage(float voltage);
void checkBatteryStatus();
void activateNetworkMode();
void deactivateNetworkMode();
void checkNetworkTimeout();
void syncBlynk4G();
void sendBlynkNotification(const char* message);
void checkWifiConnection();
void handleMotionDetection();
void checkUserPresence();
void handleAlarmStages();
void handleInitialBeeps();
float calculateDistance();
void reportSystemStatus();

// Hàm mới cho logic báo động cập nhật
bool isStrongMotion();
bool isSignificantMovement();
float calculateDistanceFromInitial(float initLat, float initLng, float currLat, float currLng);

// Xử lý ngắt MPU6050
volatile bool mpuInterrupt = false;
void IRAM_ATTR mpuISR() {
    mpuInterrupt = true;
    lastActivityTime = millis(); // Cập nhật thời gian hoạt động
}

// Xử lý ngắt RTC cho chế độ deep sleep
void IRAM_ATTR rtcIOISR() {
    // Ngắt này chỉ để đánh thức từ deep sleep
    // Không cần làm gì ở đây vì ESP32 sẽ thức dậy và chạy setup()
}

// Đánh thức ESP32 từ light sleep khi có hoạt động từ MPU6050 hoặc RF433
void IRAM_ATTR externalWakeupISR() {
    // Chỉ cần đánh thức ESP32, không cần làm gì thêm
}

// ----- TIMER CALLBACK FUNCTIONS -----
// Các callback không kiểm tra thời gian nữa vì BlynkTimer đã làm việc đó
void motionTimerCallback() {
    handleMotionDetection();
    feedWatchdog(); // Feed watchdog trong callback quan trọng
}

void rfTimerCallback() {
    checkUserPresence();
    feedWatchdog(); // Feed watchdog trong callback quan trọng
}

void batteryTimerCallback() {
    checkBatteryStatus();
}

void wifiTimerCallback() {
    if (networkModeActive) {
        checkWifiConnection();
    }
}

void blynkSyncTimerCallback() {
    if (networkModeActive && usingSIM) {
        syncBlynk4G();
    }
}

void sleepCheckTimerCallback() {
    checkSleepConditions();
}

// Kiểm tra trạng thái các module
void moduleCheckTimerCallback() {
    checkModuleStatus();
}

// Xóa timer nếu đã tồn tại
void deleteTimerIfExists(int &timerId) {
    if (timerId != -1) {
        timer.deleteTimer(timerId);
        timerId = -1;
    }
}

// Thiết lập tất cả timer cơ bản
void setupBaseTimers() {
    // Xóa tất cả timer hiện có để tránh trùng lặp
    timer.deleteTimer(timer.getNumTimers());

    // Tạo lại các timer cơ bản với interval hiện tại
    motionTimerId = timer.setInterval(motionInterval, motionTimerCallback);
    rfTimerId = timer.setInterval(rfInterval, rfTimerCallback);
    batteryTimerId = timer.setInterval(batteryInterval, batteryTimerCallback);
    sleepCheckTimerId = timer.setInterval(sleepCheckInterval, sleepCheckTimerCallback);
    moduleCheckTimerId = timer.setInterval(MODULE_CHECK_INTERVAL, moduleCheckTimerCallback);

    // WiFi và Blynk Sync chỉ tạo khi cần
    wifiTimerId = -1;
    blynkSyncTimerId = -1;

    Serial.println("Đã thiết lập tất cả timer cơ bản");
}

// Điều chỉnh interval và tạo lại timer cho pin yếu
void adjustTimersForLowBattery(bool isLowBattery) {
    if (isLowBattery) {
        // Khi pin yếu, giảm tần suất kiểm tra
        motionInterval = MOTION_BASE_INTERVAL * 2;     // 2 giây
        rfInterval = RF_BASE_INTERVAL * 2;             // 10 giây
        batteryInterval = BATTERY_BASE_INTERVAL * 1.5; // 22.5 phút
        wifiInterval = WIFI_BASE_INTERVAL * 2;         // 60 giây
        blynkSyncInterval = BLYNK_BASE_INTERVAL * 2;   // 2 phút
    } else {
        // Trở về bình thường
        motionInterval = MOTION_BASE_INTERVAL;
        rfInterval = RF_BASE_INTERVAL;
        batteryInterval = BATTERY_BASE_INTERVAL;
        wifiInterval = WIFI_BASE_INTERVAL;
        blynkSyncInterval = BLYNK_BASE_INTERVAL;
    }

    // Tạo lại các timer với interval mới
    deleteTimerIfExists(motionTimerId);
    deleteTimerIfExists(rfTimerId);
    deleteTimerIfExists(batteryTimerId);

    // Tạo lại timer
    motionTimerId = timer.setInterval(motionInterval, motionTimerCallback);
    rfTimerId = timer.setInterval(rfInterval, rfTimerCallback);
    batteryTimerId = timer.setInterval(batteryInterval, batteryTimerCallback);

    // Tạo lại timer WiFi và Blynk nếu đang hoạt động
    if (wifiTimerId != -1) {
        deleteTimerIfExists(wifiTimerId);
        wifiTimerId = timer.setInterval(wifiInterval, wifiTimerCallback);
    }

    if (blynkSyncTimerId != -1) {
        deleteTimerIfExists(blynkSyncTimerId);
        blynkSyncTimerId = timer.setInterval(blynkSyncInterval, blynkSyncTimerCallback);
    }

    Serial.println(isLowBattery ? "Đã điều chỉnh timer cho chế độ pin yếu" : "Đã điều chỉnh timer về chế độ bình thường");
}

// Điều chỉnh interval và tạo lại timer cho trạng thái báo động
void adjustTimersForAlarm(AlarmStage stage) {
    if (stage != STAGE_NONE) {
        // Khi có báo động, tăng tần suất kiểm tra
        motionInterval = MOTION_BASE_INTERVAL / 2;     // 0.5 giây
        rfInterval = RF_BASE_INTERVAL / 2;             // 2.5 giây
        wifiInterval = WIFI_BASE_INTERVAL / 2;         // 15 giây
        blynkSyncInterval = BLYNK_BASE_INTERVAL / 2;   // 30 giây
    } else {
        // Trở về bình thường, nhưng vẫn giữ chế độ pin yếu nếu có
        if (isLowBatteryMode) {
            motionInterval = MOTION_BASE_INTERVAL * 2;
            rfInterval = RF_BASE_INTERVAL * 2;
            wifiInterval = WIFI_BASE_INTERVAL * 2;
            blynkSyncInterval = BLYNK_BASE_INTERVAL * 2;
        } else {
            motionInterval = MOTION_BASE_INTERVAL;
            rfInterval = RF_BASE_INTERVAL;
            wifiInterval = WIFI_BASE_INTERVAL;
            blynkSyncInterval = BLYNK_BASE_INTERVAL;
        }
    }

    // Tạo lại các timer với interval mới
    deleteTimerIfExists(motionTimerId);
    deleteTimerIfExists(rfTimerId);

    // Tạo lại timer
    motionTimerId = timer.setInterval(motionInterval, motionTimerCallback);
    rfTimerId = timer.setInterval(rfInterval, rfTimerCallback);

    // Tạo lại timer WiFi và Blynk nếu đang hoạt động
    if (wifiTimerId != -1) {
        deleteTimerIfExists(wifiTimerId);
        wifiTimerId = timer.setInterval(wifiInterval, wifiTimerCallback);
    }

    if (blynkSyncTimerId != -1) {
        deleteTimerIfExists(blynkSyncTimerId);
        blynkSyncTimerId = timer.setInterval(blynkSyncInterval, blynkSyncTimerCallback);
    }

    Serial.print("Đã điều chỉnh timer cho trạng thái báo động: ");
    Serial.println(stage);
}

// Thêm/xóa timer WiFi
void manageWifiTimer(bool activate) {
    if (activate) {
        if (wifiTimerId == -1) {
            wifiTimerId = timer.setInterval(wifiInterval, wifiTimerCallback);
            Serial.println("Đã kích hoạt timer kiểm tra WiFi");
        }
    } else {
        deleteTimerIfExists(wifiTimerId);
        Serial.println("Đã vô hiệu hóa timer kiểm tra WiFi");
    }
}

// Thêm/xóa timer Blynk sync
void manageBlynkSyncTimer(bool activate) {
    if (activate) {
        if (blynkSyncTimerId == -1) {
            blynkSyncTimerId = timer.setInterval(blynkSyncInterval, blynkSyncTimerCallback);
            Serial.println("Đã kích hoạt timer đồng bộ Blynk");
        }
    } else {
        deleteTimerIfExists(blynkSyncTimerId);
        Serial.println("Đã vô hiệu hóa timer đồng bộ Blynk");
    }
}

// Chuẩn bị trước khi vào sleep
void prepareTimersForSleep() {
    // Vô hiệu hóa tất cả timer
    if (motionTimerId != -1) timer.disable(motionTimerId);
    if (rfTimerId != -1) timer.disable(rfTimerId);
    if (batteryTimerId != -1) timer.disable(batteryTimerId);
    if (wifiTimerId != -1) timer.disable(wifiTimerId);
    if (blynkSyncTimerId != -1) timer.disable(blynkSyncTimerId);
    if (sleepCheckTimerId != -1) timer.disable(sleepCheckTimerId);
    if (moduleCheckTimerId != -1) timer.disable(moduleCheckTimerId);

    // Ghi lại thời điểm vào sleep
    lastSleepTime = millis();

    Serial.println("Đã tạm dừng tất cả timer trước khi vào sleep");
}

// Khôi phục sau khi thức dậy từ sleep
void restoreTimersAfterWakeup() {
    // Tính thời gian đã ngủ
    unsigned long sleepDuration = millis() - lastSleepTime;

    // Kích hoạt lại tất cả timer
    if (motionTimerId != -1) timer.enable(motionTimerId);
    if (rfTimerId != -1) timer.enable(rfTimerId);
    if (batteryTimerId != -1) timer.enable(batteryTimerId);
    if (wifiTimerId != -1) timer.enable(wifiTimerId);
    if (blynkSyncTimerId != -1) timer.enable(blynkSyncTimerId);
    if (sleepCheckTimerId != -1) timer.enable(sleepCheckTimerId);
    if (moduleCheckTimerId != -1) timer.enable(moduleCheckTimerId);

    Serial.println("Đã kích hoạt lại tất cả timer sau khi thức dậy");

    // Gọi các callback cần thiết nếu đã qua thời gian đáng kể
    if (sleepDuration >= batteryInterval) {
        checkBatteryStatus();
    }
}

// Reset tất cả timer về trạng thái ban đầu
void resetAllTimers() {
    // Xóa tất cả timer hiện có
    timer.deleteTimer(timer.getNumTimers());

    // Reset các interval về trạng thái mặc định
    motionInterval = MOTION_BASE_INTERVAL;
    rfInterval = RF_BASE_INTERVAL;
    batteryInterval = BATTERY_BASE_INTERVAL;
    wifiInterval = WIFI_BASE_INTERVAL;
    blynkSyncInterval = BLYNK_BASE_INTERVAL;
    sleepCheckInterval = SLEEP_CHECK_BASE_INTERVAL;

    // Thiết lập lại timer cơ bản
    setupBaseTimers();

    Serial.println("Đã reset tất cả timer về trạng thái mặc định");
}

// ----- WATCHDOG FUNCTIONS -----
// Khởi tạo watchdog timer
void setupWatchdog() {
    Serial.println("Khởi tạo Watchdog Timer...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // Timeout sau 30 giây, reset ESP32 khi timeout
    esp_task_wdt_add(NULL);               // Thêm task hiện tại vào watchdog
    Serial.println("Watchdog Timer đã được khởi tạo với timeout 30 giây");
}

// Feed watchdog để ngăn reset
void feedWatchdog() {
    esp_task_wdt_reset();
}

// Kiểm tra trạng thái của các module
void checkModuleStatus() {
    // Feed watchdog trong mỗi lần kiểm tra module
    feedWatchdog();

    // Kiểm tra WiFi nếu đang kết nối
    if (networkModeActive && !usingSIM) {
        if (WiFi.status() != WL_CONNECTED) {
            if (wifiResponding && (millis() - lastWifiResponse > WIFI_MODULE_TIMEOUT)) {
                Serial.println("Phát hiện WiFi không phản hồi!");
                wifiResponding = false;

                // Kiểm tra số lần reset trước khi thực hiện
                if (wifiResetCount < MAX_RESET_ATTEMPTS) {
                    resetWiFiModule();
                } else {
                    Serial.println("Đã vượt quá số lần thử reset WiFi, thực hiện hardware reset");
                    performHardwareReset();
                }
            }
        } else {
            lastWifiResponse = millis();
            wifiResponding = true;
        }
    }

    // Kiểm tra SIM nếu đang hoạt động
    if (simActive) {
        bool simResponse = false;

        // Gửi lệnh AT để kiểm tra SIM
        SerialSIM.println("AT");
        delay(100);

        // Đọc phản hồi
        unsigned long checkStart = millis();
        while (millis() - checkStart < 1000) { // Chờ tối đa 1 giây
            if (SerialSIM.available()) {
                if (SerialSIM.find("OK")) {
                    simResponse = true;
                    break;
                }
            }
            delay(10);
        }

        if (simResponse) {
            lastSimResponse = millis();
            simResponding = true;
        } else if (simResponding && (millis() - lastSimResponse > SIM_MODULE_TIMEOUT)) {
            Serial.println("Phát hiện SIM không phản hồi!");
            simResponding = false;

            // Kiểm tra số lần reset trước khi thực hiện
            if (simResetCount < MAX_RESET_ATTEMPTS) {
                resetSimModule();
            } else {
                Serial.println("Đã vượt quá số lần thử reset SIM, thực hiện hardware reset");
                performHardwareReset();
            }
        }
    }

    // Kiểm tra GPS nếu đang hoạt động
    if (gpsActive) {
        // Đọc dữ liệu GPS để kiểm tra
        bool gpsResponse = false;
        unsigned long checkStart = millis();

        while (millis() - checkStart < 1000 && SerialGPS.available() > 0) { // Chờ tối đa 1 giây
            if (gps.encode(SerialGPS.read())) {
                gpsResponse = true;
                break;
            }
        }

        if (gpsResponse) {
            lastGpsResponse = millis();
            gpsResponding = true;
        } else if (gpsResponding && (millis() - lastGpsResponse > GPS_MODULE_TIMEOUT)) {
            Serial.println("Phát hiện GPS không phản hồi!");
            gpsResponding = false;

            // Kiểm tra số lần reset trước khi thực hiện
            if (gpsResetCount < MAX_RESET_ATTEMPTS) {
                resetGpsModule();
            } else {
                // Gửi cảnh báo mất GPS
                if (simActive) {
                    sendSMSWithCooldown("CANH BAO: Module GPS khong phan hoi sau nhieu lan thu reset. Can kiem tra thiet bi.", SMS_ALERT_GPS_LOST);
                }

                Serial.println("Đã vượt quá số lần thử reset GPS, thực hiện hardware reset");
                performHardwareReset();
            }
        }
    }
}

// Reset module WiFi
void resetWiFiModule() {
    Serial.println("Thực hiện reset module WiFi...");

    // Ngắt kết nối WiFi hiện tại
    WiFi.disconnect(true);
    delay(1000);

    // Khởi động lại WiFi
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_STA);

    // Kết nối lại nếu đang ở chế độ network
    if (networkModeActive && !usingSIM) {
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

    wifiResetCount++;
    Serial.print("Đã reset WiFi, số lần reset: ");
    Serial.println(wifiResetCount);
}

// Reset module SIM - Đã loại bỏ điều khiển PWRKEY
void resetSimModule() {
    Serial.println("Thực hiện reset module SIM...");

    // Dùng AT command để soft reset
    SerialSIM.println("AT+CFUN=0");  // Soft reset thông qua AT command
    delay(1000);
    SerialSIM.println("AT+CFUN=1");  // Khởi động lại modem
    delay(3000);  // Đợi khởi động lại

    // Thiết lập lại các thông số cơ bản
    SerialSIM.println("AT");
    delay(500);
    SerialSIM.println("AT+CMGF=1");  // Đặt chế độ văn bản cho SMS
    delay(1000);

    simResetCount++;
    Serial.print("Đã reset SIM, số lần reset: ");
    Serial.println(simResetCount);

    // Nếu đang sử dụng SIM cho mạng, thiết lập lại kết nối
    if (networkModeActive && usingSIM) {
        setup4GConnection();
    }
}

// Kiểm tra phản hồi SIM
bool checkSimResponse() {
    unsigned long startTime = millis();
    SerialSIM.println("AT");

    while (millis() - startTime < 1000) {
        if (SerialSIM.available()) {
            String response = SerialSIM.readString();
            if (response.indexOf("OK") >= 0) {
                return true;
            }
        }
        delay(10);
    }
    return false;
}

// Reset module GPS
void resetGpsModule() {
    Serial.println("Thực hiện reset module GPS...");

    // Tắt GPS bằng cách đặt vào chế độ ngủ
    putGPSToSleep();
    delay(1000);

    // Đánh thức GPS
    wakeGPSFromSleep();

    gpsResetCount++;
    Serial.print("Đã reset GPS, số lần reset: ");
    Serial.println(gpsResetCount);
}

// Thực hiện hardware reset toàn bộ hệ thống
void performHardwareReset() {
    Serial.println("Thực hiện HARDWARE RESET toàn bộ hệ thống...");

    // Gửi thông báo về trạng thái reset nếu SIM đang hoạt động
    if (simActive) {
        String resetMessage = "CANH BAO: He thong bi treo, thuc hien reset toan bo!";

        // Sử dụng hàm mới với cooldown
        sendSMSWithCooldown(resetMessage.c_str(), SMS_ALERT_MODULE_RESET);
        delay(3000); // Chờ SMS gửi xong
    }

    // Reset các biến đếm số lần reset
    wifiResetCount = 0;
    simResetCount = 0;
    gpsResetCount = 0;

    // Reset ESP32 bằng Watchdog
    while(1) {
        // Không feed watchdog, để nó tự reset
        delay(WDT_TIMEOUT * 1000 + 1000);
    }
}

// Cập nhật thời gian hoạt động mỗi khi có tương tác
void updateActivityTime() {
    lastActivityTime = millis();
    feedWatchdog(); // Feed watchdog khi có hoạt động
}

// Kiểm tra xem có thể vào chế độ light sleep không
bool canEnterLightSleep() {
    // Không vào sleep nếu đang ở các trạng thái báo động
    if (alarmStage != STAGE_NONE) return false;

    // Không vào sleep nếu đang có kết nối mạng đang hoạt động
    if (networkModeActive) return false;

    // Không vào sleep nếu đang phát hiện chuyển động
    if (motionDetected) return false;

    // Không vào sleep nếu mới có hoạt động gần đây
    if (millis() - lastActivityTime < LIGHT_SLEEP_TIMEOUT) return false;

    return true;
}

// Kiểm tra xem có thể vào chế độ deep sleep không
bool canEnterDeepSleep() {
    // Các điều kiện tương tự light sleep, nhưng thời gian dài hơn
    if (!canEnterLightSleep()) return false;

    // Thêm điều kiện thời gian không hoạt động dài hơn nhiều
    if (millis() - lastActivityTime < DEEP_SLEEP_TIMEOUT) return false;

    return true;
}

// Chuẩn bị cho việc vào chế độ sleep
void prepareForSleep() {
    // Tạm dừng tất cả timer
    prepareTimersForSleep();

    // Ghi lại thời điểm vào sleep
    lastSleepTime = millis();

    // Đặt GPS vào chế độ ngủ
    if (gpsActive) {
        putGPSToSleep();
    }

    // Đặt MPU vào chế độ ngủ
    if (!mpuSleepState) {
        putMPUToSleep();
    }

    // Tắt các thiết bị ngoại vi không cần thiết
    if (networkModeActive) {
        deactivateNetworkMode();
    }

    if (simActive) {
        deactivateSim();
    }

    // RF luôn được cấp nguồn, không cần bật/tắt

    // Bỏ qua việc tắt LED vì đã vô hiệu hóa
    Serial.println("Đã chuẩn bị xong cho chế độ sleep");
}

// Thiết lập các nguồn đánh thức
void setupWakeupSources() {
    // Thiết lập chân ngắt MPU6050 làm nguồn đánh thức
    esp_sleep_enable_ext0_wakeup((gpio_num_t)MPU_INT_PIN, HIGH);

    // Thiết lập chân dữ liệu RF làm nguồn đánh thức
    esp_sleep_enable_ext1_wakeup(1ULL << RF_DATA_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Thiết lập timer đánh thức
    esp_sleep_enable_timer_wakeup(RF_CHECK_INTERVAL * 1000); // Đơn vị microseconds
}

// Đi vào chế độ light sleep
void goToLightSleep(uint64_t sleepTime) {
    Serial.println("Vào chế độ Light Sleep để tiết kiệm năng lượng...");

    // Chuẩn bị cho việc vào sleep
    prepareForSleep();

    // Thiết lập thời gian đánh thức nếu được chỉ định
    if (sleepTime > 0) {
        esp_sleep_enable_timer_wakeup(sleepTime * 1000); // Đơn vị microseconds
    } else {
        // Mặc định là RF_CHECK_INTERVAL nếu không chỉ định
        esp_sleep_enable_timer_wakeup(RF_CHECK_INTERVAL * 1000);
    }

    // Thiết lập các nguồn đánh thức khác
    setupWakeupSources();

    // Đi vào light sleep
    esp_light_sleep_start();

    // Đoạn code sau này sẽ được thực thi khi ESP32 thức dậy từ light sleep
    wakeupCause = esp_sleep_get_wakeup_cause();
    isWakingUp = true;

    Serial.println("Đã thức dậy từ Light Sleep!");
    Serial.print("Nguyên nhân thức dậy: ");

    switch(wakeupCause) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("Ngắt từ MPU6050 (chuyển động)");
            mpuInterrupt = true;  // Xử lý như ngắt MPU
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            Serial.println("Ngắt từ RF433 (tín hiệu RF)");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Hết thời gian ngủ - kiểm tra định kỳ");
            break;
        default:
            Serial.println("Nguyên nhân khác");
            break;
    }

    // Cập nhật thời gian hoạt động
    updateActivityTime();

    // Khôi phục timer sau khi thức dậy
    restoreTimersAfterWakeup();

    // Đánh thức MPU nếu chủ sở hữu không có mặt
    if (!ownerPresent && mpuSleepState) {
        wakeMPUFromSleep();
    }
}

// Đi vào chế độ deep sleep
void goToDeepSleep(uint64_t sleepTime) {
    Serial.println("Vào chế độ Deep Sleep để tiết kiệm năng lượng tối đa...");

    // Chuẩn bị cho việc vào sleep
    prepareForSleep();

    // Thiết lập thời gian đánh thức
    if (sleepTime > 0) {
        esp_sleep_enable_timer_wakeup(sleepTime * 1000); // Đơn vị microseconds
    } else {
        // Mặc định là 5 phút nếu không chỉ định
        esp_sleep_enable_timer_wakeup(300000000); // 5 phút
    }

    // Thiết lập các nguồn đánh thức khác
    setupWakeupSources();

    // Ghi bootCount vào biến thông thường
    bootCount++;

    // Đi vào deep sleep
    esp_deep_sleep_start();

    // Code sau dòng này sẽ không được thực thi vì ESP32 sẽ khởi động lại sau deep sleep
}

// Xử lý thức dậy từ sleep
void handleWakeup() {
    wakeupCause = esp_sleep_get_wakeup_cause();

    if (wakeupCause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        Serial.println("Thức dậy từ Deep Sleep!");
        Serial.print("Boot count: ");
        Serial.println(bootCount);
        Serial.print("Nguyên nhân thức dậy: ");

        switch(wakeupCause) {
            case ESP_SLEEP_WAKEUP_EXT0:
                Serial.println("Ngắt từ MPU6050 (chuyển động)");
                break;
            case ESP_SLEEP_WAKEUP_EXT1:
                Serial.println("Ngắt từ RF433 (tín hiệu RF)");
                break;
            case ESP_SLEEP_WAKEUP_TIMER:
                Serial.println("Hết thời gian ngủ - kiểm tra định kỳ");
                break;
            default:
                Serial.println("Nguyên nhân khác");
                break;
        }

        // Cập nhật thời gian hoạt động
        updateActivityTime();

        isWakingUp = true;
    } else {
        // Khởi động bình thường, không phải từ sleep
        isWakingUp = false;
        bootCount = 0; // Reset boot count nếu là khởi động bình thường
    }
}

// Kiểm tra các điều kiện để vào chế độ sleep
void checkSleepConditions() {
    // Kiểm tra deep sleep trước (vì nó có điều kiện khắt khe hơn)
    if (canEnterDeepSleep()) {
        goToDeepSleep(0); // 0 = sử dụng thời gian mặc định
        return;
    }

    // Nếu không vào được deep sleep, kiểm tra light sleep
    if (canEnterLightSleep()) {
        goToLightSleep(0); // 0 = sử dụng thời gian mặc định
        return;
    }
}

// Hàm phát hiện người dùng qua RF - phát âm thanh xác nhận
void acknowledgeUserPresence() {
    // Phát ra 2 tiếng bíp ngắn để xác nhận đã nhận diện người dùng
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Đã xác nhận nhận diện chủ sở hữu với âm thanh");
    updateActivityTime(); // Cập nhật thời gian hoạt động
}

// Kích hoạt module RF - Đã sửa đổi vì RF luôn được cấp nguồn
void activateRF() {
    // RF luôn được cấp nguồn, không cần bật/tắt
    // Chỉ cập nhật trạng thái
    rfPowerState = true;
    updateActivityTime(); // Cập nhật thời gian hoạt động
}

// Tắt module RF - Đã sửa đổi, RF luôn bật
void deactivateRF() {
    // RF luôn được cấp nguồn, không cần bật/tắt
    // Nhưng vẫn giữ hàm này để tương thích với code cũ
    // Chỉ cập nhật trạng thái logic
    rfPowerState = true; // Luôn giữ trạng thái là true
}

// Vô hiệu hóa RF433 (khi xe đã bị lấy trộm) - Đã sửa đổi
void disableRF433() {
    rfEnabled = false;
    // Không tắt nguồn RF, chỉ đánh dấu là bị vô hiệu hóa
    Serial.println("Đã vô hiệu hóa RF433 do xe đã bị lấy trộm");
    updateActivityTime(); // Cập nhật thời gian hoạt động
}

// Đặt GPS vào chế độ ngủ - ATGM336H
void putGPSToSleep() {
    Serial.println("Đặt ATGM336H vào chế độ ngủ");

    // Gửi lệnh PMTK để đặt ATGM336H vào chế độ ngủ
    SerialGPS.print(GPS_SLEEP_CMD);

    Serial.println("ATGM336H đã vào chế độ ngủ");
}

// Đánh thức GPS từ chế độ ngủ - ATGM336H
void wakeGPSFromSleep() {
    Serial.println("Đánh thức ATGM336H từ chế độ ngủ");

    // ATGM336H có thể được đánh thức bằng bất kỳ byte nào hoặc lệnh hot start
    SerialGPS.print(GPS_WAKE_CMD);

    // Giảm thời gian đợi vì hot start rất nhanh
    delay(800);  // Đợi GPS khởi động - giảm xuống còn 800ms

    Serial.println("ATGM336H đã được đánh thức");
    updateActivityTime(); // Cập nhật thời gian hoạt động

    // Cập nhật thời gian phản hồi GPS
    lastGpsResponse = millis();
    gpsResponding = true;
}

// Cập nhật vị trí GPS - Đã tích hợp bộ lọc Kalman và giới hạn log
void updateGpsPosition() {
    if (gpsActive) {
        bool currentlyValid = gps.location.isValid();
        unsigned long currentTime = millis();
        bool shouldLogNow = (currentTime - lastGpsSerialLog >= GPS_LOG_INTERVAL);

        if (currentlyValid) {
            // Cập nhật thời gian phản hồi GPS
            lastGpsResponse = millis();
            gpsResponding = true;

            // Lấy dữ liệu vị trí thô từ GPS
            float rawLat = gps.location.lat();
            float rawLng = gps.location.lng();

            // Áp dụng bộ lọc Kalman
            float filteredLat, filteredLng;
            gpsKalman.updatePosition(rawLat, rawLng, &filteredLat, &filteredLng);

            // Cập nhật vị trí đã lọc
            currentLat = filteredLat;
            currentLng = filteredLng;

            // Cập nhật trạng thái tín hiệu GPS
            if (!hadValidFix) {
                hadValidFix = true;
                signalLost = false;

                // Khởi tạo lại bộ lọc Kalman với vị trí đầu tiên
                gpsKalman.reset(rawLat, rawLng);

                // Luôn log khi có sự kiện quan trọng
                Serial.println("Đã nhận được tín hiệu GPS hợp lệ lần đầu tiên");
            } else if (signalLost) {
                signalLost = false;
                Serial.println("Tín hiệu GPS đã được khôi phục");
            }

            // Chỉ log mỗi 10 giây
            if (shouldLogNow) {
                Serial.print("Vị trí GPS: ");
                Serial.print(currentLat, 6);
                Serial.print(", ");
                Serial.println(currentLng, 6);

                if (gps.speed.isValid()) {
                    Serial.print("Tốc độ: ");
                    Serial.print(gps.speed.kmph());
                    Serial.println(" km/h");
                }

                lastGpsSerialLog = currentTime;  // Cập nhật thời gian log
            }

            // Lưu vị trí này làm vị trí cuối cùng hợp lệ
            lastLat = currentLat;
            lastLng = currentLng;

            // Cập nhật lại thời gian bắt đầu kích hoạt GPS
            gpsActivationTime = millis();
            updateActivityTime();
        } else {
            // Kiểm tra xem đã từng có tín hiệu GPS hợp lệ chưa
            if (hadValidFix && !signalLost) {
                // Đánh dấu là đã mất tín hiệu và gửi cảnh báo nếu cần
                signalLost = true;
                Serial.println("Đã mất tín hiệu GPS - Sử dụng vị trí cuối cùng đã biết");

                // Gửi cảnh báo nếu đang trong trạng thái theo dõi
                if (alarmStage == STAGE_TRACKING && simActive) {
                    sendSMSWithCooldown("CANH BAO: Da mat tin hieu GPS. Dang su dung vi tri cuoi cung da biet.", SMS_ALERT_GPS_LOST);
                }
            } else if (!hadValidFix && shouldLogNow) {
                Serial.println("Không thể lấy vị trí GPS hợp lệ");
                lastGpsSerialLog = currentTime;  // Cập nhật thời gian log
            }
        }
    }
}

// Kích hoạt module SIM A7682S - Đã loại bỏ điều khiển PWRKEY
void activateSim() {
    if (!simActive) {
        Serial.println("Kích hoạt module SIM A7682S");

        // Không sử dụng GPIO để điều khiển nguồn SIM
        // Thay vào đó, chỉ sử dụng AT command

        // Kiểm tra module
        SerialSIM.println("AT");
        delay(500);

        // Đặt chế độ văn bản cho SMS
        SerialSIM.println("AT+CMGF=1");
        delay(1000);

        simActive = true;

        // Cập nhật thời gian phản hồi SIM
        lastSimResponse = millis();
        simResponding = true;

        updateActivityTime(); // Cập nhật thời gian hoạt động
    }
}

// Tắt module SIM A7682S - Sửa đổi để sử dụng chế độ ngủ
void deactivateSim() {
    if (simActive && !networkModeActive && alarmStage != STAGE_TRACKING) {
        Serial.println("Đặt module SIM vào chế độ tiết kiệm năng lượng");

        // Thay vì tắt nguồn hoàn toàn, sử dụng AT command để vào chế độ tiết kiệm năng lượng
        SerialSIM.println("AT+CSCLK=1");  // Cho phép module vào chế độ ngủ
        delay(500);

        // Vẫn giữ simActive = true, nhưng đánh dấu trạng thái ngủ
        simActive = true;  // Module vẫn hoạt động nhưng ở chế độ ngủ
        blynkOverSIM = false;

        Serial.println("Module SIM đã vào chế độ tiết kiệm năng lượng");
    }
}

// Đánh thức SIM từ chế độ ngủ
void wakeSimFromSleep() {
    if (simActive) {
        // Gửi AT command để đánh thức module
        SerialSIM.println("AT");  // Bất kỳ AT command nào cũng đánh thức module
        delay(500);
        SerialSIM.println("AT+CSCLK=0");  // Tắt chế độ ngủ
        delay(500);

        // Kiểm tra phản hồi
        if (checkSimResponse()) {
            Serial.println("Module SIM đã được đánh thức từ chế độ ngủ");
        } else {
            // Nếu không phản hồi, thử reset nhẹ nhàng
            SerialSIM.println("AT+CFUN=1,1");  // Soft reset
            delay(3000);
        }
    }
}

// Kích hoạt module GPS ATGM336H - Chỉ đánh thức từ chế độ ngủ
void activateGPS() {
    if (!gpsActive) {
        Serial.println("Kích hoạt module GPS ATGM336H");

        // Đánh thức GPS từ chế độ ngủ
        wakeGPSFromSleep();

        gpsActive = true;
        gpsActivationTime = millis();  // Ghi lại thời điểm bắt đầu kích hoạt

        // Đọc dữ liệu GPS ban đầu để lấy vị trí hiện tại
        unsigned long startTime = millis();
        while (millis() - startTime < 3000 && !gps.location.isValid()) {  // Giảm thời gian đợi xuống 3s
            while (SerialGPS.available() > 0) {
                gps.encode(SerialGPS.read());
            }
            delay(10);
        }

        if (gps.location.isValid()) {
            updateGpsPosition();
            hadValidFix = true;
            signalLost = false;

            // Cập nhật thời gian phản hồi GPS
            lastGpsResponse = millis();
            gpsResponding = true;
        } else {
            hadValidFix = false;
            signalLost = false;
        }

        updateActivityTime(); // Cập nhật thời gian hoạt động
    }
}

// Tắt GPS ATGM336H - Chỉ đặt vào chế độ ngủ, không cắt nguồn
void deactivateGPS() {
    if (gpsActive && alarmStage == STAGE_NONE && !networkModeActive) {
        Serial.println("Đặt module GPS ATGM336H vào chế độ ngủ để tiết kiệm năng lượng");

        // Đặt GPS vào chế độ ngủ
        putGPSToSleep();

        // Chỉ thay đổi trạng thái logic, không cắt nguồn
        gpsActive = false;
        gpsActivationTime = 0;
        hadValidFix = false;
        signalLost = false;
    }
}

// Tính khoảng cách di chuyển - Sử dụng vị trí đã lọc Kalman
float calculateDistance() {
    if (!initialPositionSet || !hadValidFix) {
        return 0;  // Không có vị trí trước đó hoặc GPS không hoạt động hoặc vị trí không hợp lệ
    }

    // Chuyển đổi vĩ độ/kinh độ từ độ sang radian
    float lat1 = lastLat * 0.01745329252;
    float lon1 = lastLng * 0.01745329252;
    float lat2 = currentLat * 0.01745329252;
    float lon2 = currentLng * 0.01745329252;

    // Công thức Haversine
    float dlon = lon2 - lon1;
    float dlat = lat2 - lat1;
    float a = pow(sin(dlat/2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = 6371000 * c;  // Bán kính Trái Đất tính bằng mét

    Serial.print("Khoảng cách di chuyển: ");
    Serial.print(distance);
    Serial.println(" mét");

    return distance;
}

// Gửi SMS qua A7682S - Hàm gốc, được gọi từ sendSMSWithCooldown
void sendSmsAlert(const char* message) {
    if (!simActive) {
        activateSim();
        delay(1000);
    } else if (!checkSimResponse()) {
        // Nếu SIM đang ở chế độ ngủ, đánh thức nó
        wakeSimFromSleep();
        delay(1000);
    }

    // Gửi SMS
    SerialSIM.println("AT+CMGF=1");  // Đặt chế độ văn bản
    delay(1000);

    // Đặt số điện thoại người nhận
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(PHONE_NUMBER);
    SerialSIM.println("\"");
    delay(1000);

    // Gửi nội dung tin nhắn
    SerialSIM.print(message);
    SerialSIM.write(26);  // Ctrl+Z để kết thúc tin nhắn
    delay(5000);  // Đợi SMS được gửi

    Serial.print("Đã gửi Cảnh báo SMS: ");
    Serial.println(message);

    // Cập nhật thời gian phản hồi SIM
    lastSimResponse = millis();
    simResponding = true;

    updateActivityTime(); // Cập nhật thời gian hoạt động
}

// Hàm tính toán điện áp pin thực tế
float getBatteryVoltage() {
    // Đọc điện áp analog từ chân giám sát pin
    int adcValue = analogRead(BATTERY_PIN);

    // Chuyển đổi giá trị ADC thành điện áp
    float vOut = adcValue * (3.3 / 4095.0);

    // Tính toán điện áp pin thực tế với mạch chia áp R1=20k, R2=100k
    float voltage = vOut * 6.0;  // Vin = Vout * (R1+R2)/R1 = Vout * 6

    // Giới hạn giá trị điện áp (đề phòng đọc sai)
    if (voltage > 8.5) voltage = 8.5;
    if (voltage < 5.0) voltage = 5.0;

    return voltage;
}

// Hàm tính phần trăm pin dựa trên điện áp
int calculateBatteryPercentage(float voltage) {
    // Tính phần trăm dựa trên điện áp
    int percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100;

    // Giới hạn giá trị trong khoảng 0-100%
    if (percentage > 100) percentage = 100;
    if (percentage < 0) percentage = 0;

    return percentage;
}

// Kiểm tra trạng thái pin
void checkBatteryStatus() {
    // Đọc điện áp pin
    batteryVoltage = getBatteryVoltage();

    // Tính phần trăm pin
    batteryPercentage = calculateBatteryPercentage(batteryVoltage);

    Serial.print("Điện áp pin: ");
    Serial.print(batteryVoltage, 2);
    Serial.print("V (");
    Serial.print(batteryPercentage);
    Serial.println("%)");

    // Gửi thông tin lên Blynk nếu đã kết nối
    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
        Blynk.virtualWrite(VPIN_BATTERY, batteryPercentage);  // Gửi phần trăm pin
    }

    // Kiểm tra pin thấp
    if (batteryPercentage <= LOW_BATTERY_THRESHOLD && !lowBatteryAlerted) {
        // Gửi cảnh báo pin thấp qua SMS
        if (!simActive) {
            activateSim();
            delay(1000);
        }

        String batteryMessage = "CANH BAO: Pin thap, chi con ";
        batteryMessage += String(batteryPercentage);
        batteryMessage += "%. Vui long sac pin ngay.";

        // Sử dụng hàm mới với cooldown
        if (sendSMSWithCooldown(batteryMessage.c_str(), SMS_ALERT_LOW_BATTERY)) {
            // Đánh dấu đã gửi cảnh báo
            lowBatteryAlerted = true;

            // Điều chỉnh timer cho chế độ pin yếu
            if (!isLowBatteryMode) {
                isLowBatteryMode = true;
                adjustTimersForLowBattery(true);
            }

            Serial.print("Đã gửi cảnh báo pin thấp: ");
            Serial.println(batteryPercentage);
        }

        updateActivityTime(); // Cập nhật thời gian hoạt động
    }

    // Reset trạng thái cảnh báo pin khi pin đã được sạc trên 80%
    if (batteryPercentage >= HIGH_BATTERY_THRESHOLD && lowBatteryAlerted) {
        lowBatteryAlerted = false;

        // Trở lại chế độ bình thường nếu đang ở chế độ pin yếu
        if (isLowBatteryMode) {
            isLowBatteryMode = false;
            adjustTimersForLowBattery(false);
        }

        Serial.println("Pin đã được sạc đầy, reset trạng thái cảnh báo pin thấp");
    }
}

// Cập nhật dữ liệu lên Blynk (không bao gồm GPS)
void updateBlynkData() {
    if (Blynk.connected() || blynkOverSIM) {
        Blynk.virtualWrite(VPIN_BATTERY, batteryPercentage);
        Blynk.virtualWrite(VPIN_OWNER, ownerPresent ? 1 : 0);
        Blynk.virtualWrite(VPIN_ALARM_STATE, (int)alarmStage);
        Blynk.virtualWrite(VPIN_MOTION, motionDetected ? 1 : 0);
        updateActivityTime(); // Cập nhật thời gian hoạt động

        // Cập nhật thời gian phản hồi WiFi
        lastWifiResponse = millis();
        wifiResponding = true;
    }
}

// Thiết lập kết nối qua 4G với Blynk - cho A7682S
void setup4GConnection() {
    // Gửi AT commands để thiết lập kết nối 4G cho A7682S
    Serial.println("Thiết lập kết nối 4G với A7682S...");

    // Kiểm tra nếu module đang ở chế độ ngủ
    wakeSimFromSleep();
    delay(1000);

    // Đảm bảo module đã sẵn sàng
    SerialSIM.println("AT");
    delay(1000);

    // Cập nhật thời gian phản hồi SIM
    lastSimResponse = millis();
    simResponding = true;

    // Kiểm tra đăng ký mạng
    SerialSIM.println("AT+CREG?");
    delay(1000);

    // Thiết lập chế độ kết nối
    SerialSIM.println("AT+CGDCONT=1,\"IP\",\"" + String(SIM_APN) + "\""); // Cấu hình APN
    delay(1000);

    // Kích hoạt kết nối dữ liệu (cho A7682S)
    SerialSIM.println("AT+CGACT=1,1");
    delay(3000);

    // Lấy địa chỉ IP
    SerialSIM.println("AT+CGPADDR=1");
    delay(1000);

    // Khởi tạo Blynk với SIM module - triển khai thực tế
    Serial.println("Kết nối Blynk qua 4G...");
    Blynk.config(BLYNK_AUTH_TOKEN, BLYNK_SERVER, BLYNK_PORT);

    // Khởi tạo kết nối TCP cho Blynk
    SerialSIM.print("AT+CIPSTART=\"TCP\",\"");
    SerialSIM.print(BLYNK_SERVER);
    SerialSIM.print("\",");
    SerialSIM.println(BLYNK_PORT);
    delay(3000);

    // Kiểm tra kết nối
    SerialSIM.println("AT+CIPSTATUS");
    delay(1000);

    // Đọc và kiểm tra phản hồi
    String response = "";
    while (SerialSIM.available()) {
        response += (char)SerialSIM.read();
    }

    if (response.indexOf("CONNECT OK") >= 0 || response.indexOf("CONNECTED") >= 0) {
        blynkOverSIM = true;
        Serial.println("Kết nối TCP đến server Blynk thành công");
        if (Blynk.connected()) {
            Blynk.virtualWrite(VPIN_CONNECTION, 2); // 2 = 4G
            updateBlynkData();
        }
    } else {
        blynkOverSIM = false;
        Serial.println("Không thể kết nối Blynk qua 4G - chỉ sử dụng SMS");
    }

    updateActivityTime(); // Cập nhật thời gian hoạt động
    feedWatchdog(); // Feed watchdog trong quá trình kết nối mạng
}

// Kích hoạt chế độ kết nối mạng
void activateNetworkMode() {
    if (!networkModeActive) {
        Serial.println("Kích hoạt chế độ kết nối mạng theo yêu cầu");
        networkModeActive = true;
        networkActivationTime = millis();

        // Feed watchdog trước khi thực hiện kết nối mạng
        feedWatchdog();

        // Thử kết nối WiFi trước
        Serial.println("Đang thử kết nối WiFi...");
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        // Chờ kết nối WiFi với timeout
        unsigned long wifiStartTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < WIFI_TIMEOUT) {
            delay(500);
            Serial.print(".");
            feedWatchdog(); // Feed watchdog trong quá trình chờ kết nối
        }

        if (WiFi.status() == WL_CONNECTED) {
            // WiFi đã kết nối thành công
            Serial.println("\nĐã kết nối WiFi");
            Serial.print("Địa chỉ IP: ");
            Serial.println(WiFi.localIP());
            usingSIM = false;
            wifiConnected = true;
            blynkOverSIM = false;

            // Cập nhật thời gian phản hồi WiFi
            lastWifiResponse = millis();
            wifiResponding = true;

            // Kết nối Blynk qua WiFi
            Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS, BLYNK_SERVER, BLYNK_PORT);
            // Cập nhật trạng thái lên Blynk
            if (Blynk.connected()) {
                Blynk.virtualWrite(VPIN_CONNECTION, 1); // 1 = WiFi
                updateBlynkData();
            }

            // Kích hoạt timer kiểm tra WiFi
            manageWifiTimer(true);
        } else {
            // WiFi không khả dụng, chuyển sang SIM
            Serial.println("\nWiFi không khả dụng, chuyển sang sử dụng 4G");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;

            // Kích hoạt module SIM
            activateSim();
            delay(2000);

            // Feed watchdog trước khi setup kết nối 4G
            feedWatchdog();

            // Thiết lập kết nối 4G và Blynk qua SIM
            setup4GConnection();
            usingSIM = true;

            // Kích hoạt timer đồng bộ Blynk qua 4G
            if (blynkOverSIM) {
                manageBlynkSyncTimer(true);
            }
        }

        // Kích hoạt GPS nếu cần xem vị trí
        if (!gpsActive) {
            activateGPS();
        }

        updateActivityTime(); // Cập nhật thời gian hoạt động
    } else {
        // Đặt lại thời gian hết hạn nếu đã được kích hoạt
        networkActivationTime = millis();
        updateActivityTime(); // Cập nhật thời gian hoạt động
    }
}

// Hủy kết nối mạng
void deactivateNetworkMode() {
    if (networkModeActive) {
        Serial.println("Hủy chế độ kết nối mạng");

        // Ngắt kết nối Blynk
        if (Blynk.connected()) {
            Blynk.disconnect();
        }

        if (usingSIM) {
            // Đóng kết nối 4G/TCP
            if (blynkOverSIM) {
                SerialSIM.println("AT+CIPCLOSE");
                delay(1000);
            }

            SerialSIM.println("AT+CGACT=0,1");
            delay(1000);

            // Đặt SIM vào chế độ ngủ nếu không cần thiết
            if (alarmStage == STAGE_NONE) {
                // Sử dụng chế độ ngủ thay vì tắt hoàn toàn
                SerialSIM.println("AT+CSCLK=1");
                delay(500);
            }

            blynkOverSIM = false;

            // Hủy timer Blynk sync
            manageBlynkSyncTimer(false);
        } else {
            // Tắt WiFi
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;

            // Hủy timer WiFi
            manageWifiTimer(false);
        }

        // Đặt GPS vào chế độ ngủ nếu không cần thiết
        if (gpsActive && alarmStage == STAGE_NONE) {
            deactivateGPS();
        }

        networkModeActive = false;
        usingSIM = false;
    }
}

// Kiểm tra nếu kết nối mạng đã timeout
void checkNetworkTimeout() {
    if (networkModeActive && millis() - networkActivationTime > NETWORK_SESSION_TIMEOUT) {
        Serial.println("Thời gian kết nối mạng đã hết");
        deactivateNetworkMode();
    }
}

// Đồng bộ dữ liệu với Blynk qua 4G
void syncBlynk4G() {
    if (usingSIM && blynkOverSIM) {
        // Gửi dữ liệu lên Blynk qua 4G (không gửi GPS)
        updateBlynkData();

        // Xử lý dữ liệu đến
        Blynk.run();

        Serial.println("Đã đồng bộ dữ liệu với Blynk qua 4G");

        // Cập nhật thời gian phản hồi SIM
        lastSimResponse = millis();
        simResponding = true;

        updateActivityTime(); // Cập nhật thời gian hoạt động
    }
}

// Hàm gửi thông báo qua Blynk
void sendBlynkNotification(const char* message) {
    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
        Blynk.notify(message);
        Serial.print("Đã gửi thông báo Blynk: ");
        Serial.println(message);

        // Cập nhật thời gian phản hồi WiFi hoặc SIM
        if (!usingSIM) {
            lastWifiResponse = millis();
            wifiResponding = true;
        } else {
            lastSimResponse = millis();
            simResponding = true;
        }

        updateActivityTime(); // Cập nhật thời gian hoạt động
    }
}

// Kiểm tra kết nối WiFi
void checkWifiConnection() {
    bool previousWifiState = wifiConnected;
    wifiConnected = (WiFi.status() == WL_CONNECTED);

    // Cập nhật thời gian phản hồi WiFi
    if (wifiConnected) {
        lastWifiResponse = millis();
        wifiResponding = true;
    }

    // Nếu trạng thái WiFi thay đổi
    if (previousWifiState != wifiConnected) {
        if (wifiConnected) {
            Serial.println("WiFi đã kết nối");
            Serial.print("Địa chỉ IP: ");
            Serial.println(WiFi.localIP());

            // Tắt SIM nếu không ở giai đoạn 3
            if (simActive && alarmStage != STAGE_TRACKING) {
                deactivateSim();
            }

            // Khởi động Blynk nếu chưa khởi động
            if (!Blynk.connected()) {
                Blynk.connect();
            }

            blynkOverSIM = false;
            usingSIM = false;

            if (Blynk.connected()) {
                Blynk.virtualWrite(VPIN_CONNECTION, 1); // 1 = WiFi
            }

            updateActivityTime(); // Cập nhật thời gian hoạt động
        } else {
            Serial.println("WiFi đã ngắt kết nối");

            // Chỉ kích hoạt SIM ở giai đoạn 3 hoặc nếu đang ở chế độ kết nối mạng
            if ((alarmStage == STAGE_TRACKING || networkModeActive) && !simActive) {
                activateSim();
                if (networkModeActive) {
                    // Feed watchdog trước khi setup kết nối 4G
                    feedWatchdog();

                    setup4GConnection();
                    usingSIM = true;

                    // Kích hoạt timer đồng bộ Blynk qua 4G
                    if (blynkOverSIM) {
                        manageBlynkSyncTimer(true);
                    }
                }
            }
        }
    }
}

// Kiểm tra xem có phải chuyển động mạnh không - Sử dụng bộ lọc Kalman
bool isStrongMotion() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Dữ liệu gia tốc thô
    float accel_x = a.acceleration.x;
    float accel_y = a.acceleration.y;
    float accel_z = a.acceleration.z;

    // Áp dụng bộ lọc Kalman
    float filtered_x, filtered_y, filtered_z;
    mpuKalman.updateAcceleration(accel_x, accel_y, accel_z, &filtered_x, &filtered_y, &filtered_z);

    // Tính độ lớn gia tốc đã lọc
    float filteredAccelMagnitude = sqrt(filtered_x * filtered_x +
                                        filtered_y * filtered_y +
                                        filtered_z * filtered_z);

    // Nếu gia tốc đã lọc vượt quá 4g (khoảng 39.2 m/s²), coi là chuyển động mạnh
    return abs(filteredAccelMagnitude - 9.8) > 39.2; // 4g
}

// Kiểm tra xem có dịch chuyển đáng kể không - Sử dụng bộ lọc Kalman
bool isSignificantMovement() {
    // Đo gia tốc góc (dùng gyro để phát hiện xoay)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Kiểm tra tốc độ góc
    float gyroMagnitude = sqrt(g.gyro.x * g.gyro.x +
                               g.gyro.y * g.gyro.y +
                               g.gyro.z * g.gyro.z);

    // Nếu có tốc độ góc lớn, có thể xe đang di chuyển
    return gyroMagnitude > 1.5; // rad/s
}

// Tính khoảng cách từ vị trí ban đầu
float calculateDistanceFromInitial(float initLat, float initLng, float currLat, float currLng) {
    // Chuyển đổi vĩ độ/kinh độ từ độ sang radian
    float lat1 = initLat * 0.01745329252;
    float lon1 = initLng * 0.01745329252;
    float lat2 = currLat * 0.01745329252;
    float lon2 = currLng * 0.01745329252;

    // Công thức Haversine
    float dlon = lon2 - lon1;
    float dlat = lat2 - lat1;
    float a = pow(sin(dlat/2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = 6371000 * c;  // Bán kính Trái Đất tính bằng mét

    return distance;
}

// Xử lý phát hiện chuyển động từ MPU6050 - Sử dụng bộ lọc Kalman
void handleMotionDetection() {
    // Nếu MPU đang ở chế độ sleep, bỏ qua việc xử lý chuyển động
    if (mpuSleepState) {
        return;
    }

    // Đọc giá trị gia tốc từ MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Dữ liệu gia tốc thô
    float accel_x = a.acceleration.x;
    float accel_y = a.acceleration.y;
    float accel_z = a.acceleration.z;

    // Áp dụng bộ lọc Kalman
    float filtered_x, filtered_y, filtered_z;
    mpuKalman.updateAcceleration(accel_x, accel_y, accel_z, &filtered_x, &filtered_y, &filtered_z);

    // Tính toán độ lớn của vector gia tốc đã lọc
    float filteredAccelMagnitude = sqrt(filtered_x * filtered_x +
                                        filtered_y * filtered_y +
                                        filtered_z * filtered_z);

    // Tính toán độ lớn của vector gia tốc thô
    float rawAccelMagnitude = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    // Giá trị gia tốc trọng trường (khoảng 9.8 m/s²)
    const float GRAVITY = 9.8;

    // Kiểm tra nếu có chuyển động (độ lớn gia tốc khác nhiều so với trọng trường)
    if (abs(filteredAccelMagnitude - GRAVITY) > 1.0) {  // Ngưỡng phát hiện chuyển động
        if (!motionDetected) {
            motionDetected = true;
            motionDetectedTime = millis();
            lastMotionTime = millis(); // Cập nhật thời gian phát hiện chuyển động cuối cùng

            Serial.println("Đã phát hiện chuyển động!");
            Serial.print("Gia tốc thô: ");
            Serial.print(rawAccelMagnitude);
            Serial.print(" m/s², Gia tốc đã lọc: ");
            Serial.print(abs(filteredAccelMagnitude - GRAVITY));
            Serial.println(" m/s²");

            updateActivityTime(); // Cập nhật thời gian hoạt động
            feedWatchdog(); // Feed watchdog khi phát hiện chuyển động

            // Cập nhật trạng thái lên Blynk nếu đang kết nối
            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_MOTION, 1);
            }

            // Nếu chủ sở hữu không có mặt, bắt đầu quy trình báo động
            if (!ownerPresent) {
                // MỚI: Nếu đang ở trạng thái báo động giai đoạn 1, tăng đếm
                if (alarmStage == STAGE_WARNING) {
                    motionDetectionCount++;
                    Serial.print("Số lần phát hiện chuyển động: ");
                    Serial.println(motionDetectionCount);

                    // Nếu đã phát hiện đủ 3 lần, chuyển sang giai đoạn 2
                    if (motionDetectionCount >= 3) {
                        alarmStage = STAGE_ALERT;
                        alarmStartTime = millis();
                        Serial.println("Đã phát hiện chuyển động 3 lần - Chuyển sang giai đoạn BÁO ĐỘNG");
                    }
                }
                    // MỚI: Nếu chưa ở trạng thái báo động, bắt đầu giai đoạn 1
                else if (alarmStage == STAGE_NONE) {
                    alarmStage = STAGE_WARNING;
                    alarmStartTime = millis();
                    firstMotionTime = millis();
                    motionDetectionCount = 1; // Đây là lần phát hiện đầu tiên
                    initialBeepsDone = false; // Chưa phát tiếng bíp ban đầu

                    Serial.println("Bắt đầu giai đoạn CẢNH BÁO do phát hiện chuyển động khi chủ sở hữu không có mặt");

                    // Điều chỉnh timer cho trạng thái báo động
                    adjustTimersForAlarm(alarmStage);
                    previousAlarmStage = alarmStage;

                    // Cập nhật trạng thái lên Blynk nếu đang kết nối
                    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                        Blynk.virtualWrite(VPIN_ALARM_STATE, 1);
                    }
                }
            }
        } else {
            // Cập nhật thời gian phát hiện chuyển động cuối cùng
            lastMotionTime = millis();
        }
    } else {
        // Nếu không phát hiện chuyển động trong một khoảng thời gian
        if (motionDetected && (millis() - motionDetectedTime > 5000)) {
            motionDetected = false;
            Serial.println("Chuyển động đã dừng");

            // Cập nhật trạng thái lên Blynk nếu đang kết nối
            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_MOTION, 0);
            }
        }
    }
}

// Kiểm tra hiện diện của chủ sở hữu thông qua RF - Đã sửa đổi
void checkUserPresence() {
    // Feed watchdog khi kiểm tra RF
    feedWatchdog();

    // Nếu RF đã bị vô hiệu hóa, không kiểm tra
    if (!rfEnabled) return;

    // RF luôn bật nguồn, không cần kiểm tra rfPowerState nữa

    bool previousOwnerState = ownerPresent;
    bool detectedRF = false;
    bool detectedNetworkActivation = false;

    // Kiểm tra thẻ RF trong một khoảng thời gian ngắn
    unsigned long rfStartTime = millis();
    while (millis() - rfStartTime < RF_ACTIVE_DURATION) {
        if (rfReceiver.available()) {
            unsigned long receivedCode = rfReceiver.getReceivedValue();
            rfReceiver.resetAvailable();
            updateActivityTime(); // Cập nhật thời gian hoạt động
            feedWatchdog(); // Feed watchdog khi nhận tín hiệu RF

            if (receivedCode == USER_RF_CODE) {
                // Mã 1: Nhận diện người dùng
                ownerPresent = true;
                lastRfCheck = millis();
                detectedRF = true;
                Serial.println("Đã phát hiện mã RF của chủ sở hữu");

                // Thêm mới: Phát âm thanh xác nhận
                acknowledgeUserPresence();
            }
            else if (receivedCode == NETWORK_ACTIVATE_CODE) {
                // Mã 2: Kích hoạt kết nối mạng
                detectedNetworkActivation = true;
                Serial.println("Đã phát hiện mã kích hoạt kết nối mạng");
            }
            else if (receivedCode == RESET_RF_CODE) {
                // Mã 3: Reset hệ thống về chế độ chờ
                manualReset = true;
                Serial.println("Đã phát hiện mã reset hệ thống");

                // Reset trạng thái hiện diện của chủ sở hữu
                ownerPresent = false;

                // Reset hệ thống về trạng thái chờ
                alarmStage = STAGE_NONE;
                beepIndex = 0;
                sequenceIndex = 0;
                inBeepSequence = false;
                sequenceCompleted = false;
                notificationSent = false;
                initialPositionSet = false;
                stage2GpsUpdateCount = 0;
                motionDetectionCount = 0;  // Reset biến đếm số lần phát hiện chuyển động
                firstMotionTime = 0;       // Reset thời gian phát hiện đầu tiên
                initialBeepsDone = false;  // Reset trạng thái phát tiếng bíp ban đầu
                digitalWrite(BUZZER_PIN, LOW);

                // Điều chỉnh timer về trạng thái bình thường
                if (previousAlarmStage != STAGE_NONE) {
                    adjustTimersForAlarm(STAGE_NONE);
                    previousAlarmStage = STAGE_NONE;
                }

                // Reset lại tất cả timer
                resetAllTimers();

                // Reset biến đếm số lần reset module
                wifiResetCount = 0;
                simResetCount = 0;
                gpsResetCount = 0;

                if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                    Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                    Blynk.virtualWrite(VPIN_MOTION, 0);
                    Blynk.virtualWrite(VPIN_OWNER, 0); // Đã reset, chủ sở hữu không hiện diện
                }

                // Phát âm thanh xác nhận reset (3 tiếng bíp)
                for (int i = 0; i < 3; i++) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(100);
                    digitalWrite(BUZZER_PIN, LOW);
                    delay(100);
                }

                // Đặt GPS vào chế độ ngủ nếu đang hoạt động
                if (gpsActive && alarmStage != STAGE_TRACKING) {
                    deactivateGPS();
                }

                // Đánh thức MPU6050 khi chủ sở hữu không có mặt
                if (mpuSleepState) {
                    wakeMPUFromSleep();
                }

                // Feed watchdog sau khi reset
                feedWatchdog();
            }
        }
        delay(10);  // Đợi chút để không chiếm CPU
    }

    // Nếu phát hiện tín hiệu kích hoạt mạng
    if (detectedNetworkActivation) {
        // Kích hoạt chế độ kết nối mạng
        activateNetworkMode();
    }

    // Nếu trạng thái hiện diện của chủ sở hữu thay đổi
    if (previousOwnerState != ownerPresent) {
        Serial.print("Sự hiện diện của chủ sở hữu: ");
        Serial.println(ownerPresent ? "Đã phát hiện" : "Không phát hiện");
        updateActivityTime(); // Cập nhật thời gian hoạt động

        // Chuyển đổi chế độ MPU dựa trên sự hiện diện của chủ sở hữu
        toggleMPUSleepMode(ownerPresent);

        if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
            Blynk.virtualWrite(VPIN_OWNER, ownerPresent ? 1 : 0);
        }

        // Nếu chủ sở hữu hiện đang có mặt, dừng mọi cảnh báo đang hoạt động
        if (ownerPresent && alarmStage != STAGE_NONE && alarmStage != STAGE_TRACKING) {
            previousAlarmStage = alarmStage; // Lưu trạng thái trước khi thay đổi
            alarmStage = STAGE_NONE;
            beepIndex = 0;
            sequenceIndex = 0;
            inBeepSequence = false;
            sequenceCompleted = false;
            notificationSent = false;
            initialPositionSet = false;
            stage2GpsUpdateCount = 0;
            motionDetectionCount = 0;  // Reset biến đếm số lần phát hiện chuyển động
            firstMotionTime = 0;       // Reset thời gian phát hiện đầu tiên
            initialBeepsDone = false;  // Reset trạng thái phát tiếng bíp ban đầu
            digitalWrite(BUZZER_PIN, LOW);

            // Điều chỉnh timer về trạng thái bình thường
            adjustTimersForAlarm(STAGE_NONE);

            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                Blynk.virtualWrite(VPIN_MOTION, 0);
            }

            // Đặt GPS vào chế độ ngủ nếu đang hoạt động
            if (gpsActive && alarmStage != STAGE_TRACKING) {
                deactivateGPS();
            }
        }
    }
}

// Xử lý báo động ở các giai đoạn - Đã cập nhật theo logic mới
void handleAlarmStages() {
    // Feed watchdog trong quá trình xử lý báo động
    feedWatchdog();

    // Kiểm tra nếu trạng thái báo động thay đổi
    if (alarmStage != previousAlarmStage) {
        adjustTimersForAlarm(alarmStage);
        previousAlarmStage = alarmStage;

        // Khởi tạo biến theo dõi trạng thái cho giai đoạn mới
        if (alarmStage == STAGE_WARNING) {
            beepIndex = 0;
            sequenceIndex = 0;
            inBeepSequence = false;
            sequenceCompleted = false;
            lastBeepEndTime = 0;
            lastMotionTime = millis();  // Cập nhật thời gian phát hiện chuyển động
            digitalWrite(BUZZER_PIN, LOW);  // Đảm bảo còi tắt khi bắt đầu
            Serial.println("Đã vào giai đoạn 1: CẢNH BÁO - Chuỗi bíp");
        }
        else if (alarmStage == STAGE_ALERT) {
            stage2CheckCount = 0;
            stage2PositionSet = false;
            digitalWrite(BUZZER_PIN, HIGH);  // Bật còi liên tục
            Serial.println("Đã vào giai đoạn 2: BÁO ĐỘNG - Còi liên tục & GPS");

            // Kích hoạt GPS và SIM
            if (!gpsActive) activateGPS();
            if (!simActive) activateSim();

            // Kích hoạt mạng nếu chưa kích hoạt
            if (!networkModeActive) {
                activateNetworkMode();
            }
        }
        else if (alarmStage == STAGE_TRACKING) {
            digitalWrite(BUZZER_PIN, LOW);  // Tắt còi khi vào chế độ theo dõi
            Serial.println("Đã vào giai đoạn 3: THEO DÕI - Gửi vị trí liên tục");

            // Vô hiệu hóa RF khi chuyển sang theo dõi
            disableRF433();
        }
    }

    switch(alarmStage) {
        case STAGE_NONE:
            // Không có gì xảy ra
            break;

        case STAGE_WARNING:
            // GIAI ĐOẠN 1: CẢNH BÁO - Phát 3 tiếng bíp ngay lập tức

            // MỚI: Kiểm tra nếu đã vượt quá 2 phút kể từ lần phát hiện đầu tiên
            if (millis() - firstMotionTime > WARNING_INACTIVITY_TIMEOUT) {
                // Không có chuyển động trong 2 phút kể từ lần đầu, quay về trạng thái bình thường
                alarmStage = STAGE_NONE;
                motionDetectionCount = 0;
                initialBeepsDone = false;
                digitalWrite(BUZZER_PIN, LOW);
                Serial.println("Quá 2 phút kể từ lần phát hiện đầu tiên - Quay về trạng thái chờ");
                break;
            }

            // MỚI: Phát 3 tiếng bíp ngay khi vào giai đoạn 1 (chỉ 1 lần)
            if (!initialBeepsDone) {
                Serial.println("Phát 3 tiếng bíp cảnh báo ban đầu");

                // Phát 3 tiếng bíp
                for (int i = 0; i < 3; i++) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(200);
                    digitalWrite(BUZZER_PIN, LOW);
                    delay(200);
                    feedWatchdog(); // Feed watchdog trong quá trình bíp
                }

                initialBeepsDone = true;
            }

            break;

        case STAGE_ALERT:
            // GIAI ĐOẠN 2: BÁO ĐỘNG - Còi liên tục và theo dõi vị trí

            // Bật còi liên tục
            digitalWrite(BUZZER_PIN, HIGH);

            // Kiểm tra nếu chủ sở hữu xuất hiện, quay về trạng thái bình thường
            if (ownerPresent) {
                alarmStage = STAGE_NONE;
                digitalWrite(BUZZER_PIN, LOW);
                Serial.println("Chủ sở hữu đã xuất hiện - Quay về trạng thái bình thường");
                updateActivityTime();
                break;
            }

            // Kiểm tra nếu đã vượt quá thời gian của giai đoạn 2
            if (millis() - alarmStartTime > STAGE2_TIMEOUT) {
                // Nếu không phát hiện dịch chuyển sau tối đa số lần kiểm tra, quay lại giai đoạn 1
                if (stage2CheckCount >= STAGE2_MAX_CHECKS && !isSignificantMovement()) {
                    alarmStage = STAGE_WARNING;
                    sequenceIndex = 0; // Reset lại chuỗi bíp
                    Serial.println("Không có dịch chuyển đáng kể sau 3 lần kiểm tra - Quay lại giai đoạn CẢNH BÁO");
                    updateActivityTime();
                    break;
                }
            }

            // Gửi cảnh báo qua Blynk và SMS (chỉ gửi một lần)
            if (!notificationSent) {
                // Gửi thông báo qua Blynk nếu đang kết nối
                if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                    Blynk.virtualWrite(VPIN_ALARM_STATE, 2);  // Giai đoạn 2
                    sendBlynkNotification("CẢNH BÁO: Phương tiện đang bị xâm nhập!");
                }

                // Gửi cảnh báo qua SMS
                if (!simActive) {
                    activateSim();
                    delay(1000);
                }

                String alertMessage = "CANH BAO: Phuong tien dang bi xam nhap bat hop phap!";
                sendSMSWithCooldown(alertMessage.c_str(), SMS_ALERT_ALARM);

                notificationSent = true;
                Serial.println("Đã gửi thông báo và SMS cảnh báo");
                updateActivityTime();
            }

            // Cập nhật và kiểm tra vị trí GPS
            if (gpsActive && (millis() - lastGpsUpdate > GPS_STAGE2_INTERVAL)) {
                updateGpsPosition();
                lastGpsUpdate = millis();

                // Thiết lập vị trí ban đầu nếu chưa có
                if (!stage2PositionSet && hadValidFix) {
                    initialStage2Lat = currentLat;
                    initialStage2Lng = currentLng;
                    stage2PositionSet = true;
                    Serial.println("Đã thiết lập vị trí ban đầu cho giai đoạn 2");
                }

                // Kiểm tra khoảng cách di chuyển nếu đã có vị trí ban đầu
                if (stage2PositionSet && hadValidFix) {
                    // Tính khoảng cách từ vị trí ban đầu
                    float distance = calculateDistanceFromInitial(initialStage2Lat, initialStage2Lng, currentLat, currentLng);

                    Serial.print("Khoảng cách di chuyển: ");
                    Serial.print(distance);
                    Serial.println(" mét");

                    // Tăng số lần kiểm tra
                    stage2CheckCount++;

                    // Nếu di chuyển quá ngưỡng, chuyển sang giai đoạn 3
                    if (distance > DISTANCE_THRESHOLD) {
                        alarmStage = STAGE_TRACKING;
                        alarmStartTime = millis();
                        notificationSent = false;
                        initialPositionSet = true;
                        lastLat = currentLat;
                        lastLng = currentLng;

                        // Gửi thông báo di chuyển
                        String movingMessage = "CANH BAO: Phuong tien dang di chuyen! Chuyen sang che do theo doi.";
                        sendSMSWithCooldown(movingMessage.c_str(), SMS_ALERT_MOVEMENT);

                        Serial.println("Phát hiện di chuyển quá 10m - Chuyển sang giai đoạn THEO DÕI");
                        updateActivityTime();
                        break;
                    }

                    // Nếu đã kiểm tra đủ số lần mà không thấy di chuyển, quay lại giai đoạn 1
                    if (stage2CheckCount >= STAGE2_MAX_CHECKS) {
                        Serial.println("Đã kiểm tra " + String(STAGE2_MAX_CHECKS) + " lần mà không thấy di chuyển đáng kể");
                    }
                }
            }
            break;

        case STAGE_TRACKING:
            // GIAI ĐOẠN 3: THEO DÕI - Gửi vị trí liên tục

            // Tắt còi để tiết kiệm pin
            digitalWrite(BUZZER_PIN, LOW);

            // Đảm bảo GPS và SIM luôn hoạt động
            if (!gpsActive) activateGPS();
            if (!simActive) activateSim();

            // Đọc GPS liên tục nhưng không in log liên tục
            if (millis() - lastGpsUpdate > 5000) { // Đọc GPS thường xuyên
                // Đọc giới hạn số lượng byte để tránh treo loop
                int bytesRead = 0;
                while (SerialGPS.available() > 0 && bytesRead < 100) {
                    gps.encode(SerialGPS.read());
                    bytesRead++;
                }

                // Vẫn cập nhật vị trí nhưng không in serial log
                updateGpsPosition();
                updateActivityTime();
            }

            // Gửi vị trí định kỳ mỗi 5 phút
            if (millis() - lastGpsUpdate > POSITION_UPDATE_INTERVAL) {
                if (hadValidFix) {
                    // In ra serial CHỈ khi gửi SMS vị trí
                    Serial.print("Gửi SMS vị trí: ");
                    Serial.print(currentLat, 6);
                    Serial.print(", ");
                    Serial.println(currentLng, 6);

                    // Tạo link Google Maps dạng directions từ vị trí người dùng đến vị trí GPS
                    String positionMessage = "Cap nhat vi tri: ";
                    positionMessage += "https://www.google.com/maps/dir/?api=1&destination=";
                    positionMessage += String(currentLat, 6);
                    positionMessage += ",";
                    positionMessage += String(currentLng, 6);
                    positionMessage += "&travelmode=driving";

                    // Thêm thông tin pin
                    positionMessage += " | Pin: ";
                    positionMessage += String(batteryPercentage);
                    positionMessage += "%";

                    // Gửi SMS với cooldown
                    sendSMSWithCooldown(positionMessage.c_str(), SMS_ALERT_POSITION);

                    lastGpsUpdate = millis();
                    Serial.println("Đã gửi cập nhật vị trí định kỳ qua SMS");
                    feedWatchdog();
                } else if (signalLost) {
                    // In ra serial CHỈ khi gửi SMS về mất tín hiệu
                    Serial.println("Gửi SMS cảnh báo mất tín hiệu GPS với vị trí cuối cùng");

                    // Nếu mất tín hiệu GPS, gửi vị trí cuối cùng
                    String lostSignalMessage = "CANH BAO: Mat tin hieu GPS. Vi tri cuoi: ";
                    lostSignalMessage += "https://www.google.com/maps/dir/?api=1&destination=";
                    lostSignalMessage += String(lastLat, 6);
                    lostSignalMessage += ",";
                    lostSignalMessage += String(lastLng, 6);
                    lostSignalMessage += "&travelmode=driving";
                    lostSignalMessage += " | Pin: ";
                    lostSignalMessage += String(batteryPercentage);
                    lostSignalMessage += "%";

                    sendSMSWithCooldown(lostSignalMessage.c_str(), SMS_ALERT_GPS_LOST);

                    lastGpsUpdate = millis();
                    feedWatchdog();
                }
            }
            break;
    }
}

// Xử lý tiếng bíp ban đầu khi khởi động
void handleInitialBeeps() {
    // Phát 3 tiếng bíp để xác nhận hệ thống đã khởi động
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }

    // Kiểm tra các thiết bị cần thiết
    Serial.println("Kiểm tra các thiết bị:");

    // Kiểm tra MPU6050
    Serial.print("- MPU6050: ");
    if (mpu.begin()) {
        Serial.println("OK");
    } else {
        Serial.println("KHÔNG TÌM THẤY");
        // Phát tiếng bíp dài nếu không tìm thấy MPU6050
        digitalWrite(BUZZER_PIN, HIGH);
        delay(1000);
        digitalWrite(BUZZER_PIN, LOW);
    }

    // Kiểm tra module RF
    Serial.print("- Module RF433: ");
    // RF luôn được kích hoạt
    Serial.println("Đã kích hoạt");

    // Feed watchdog trong quá trình kiểm tra thiết bị
    feedWatchdog();

    // Kiểm tra GPS
    Serial.print("- GPS ATGM336H: ");
    // GPS luôn được cấp nguồn, đánh thức từ chế độ ngủ
    wakeGPSFromSleep();

    unsigned long gpsStartTime = millis();
    bool gpsResponded = false;

    while (millis() - gpsStartTime < 1000) { // Giảm thời gian đợi xuống 1s
        if (SerialGPS.available()) {
            gpsResponded = true;
            break;
        }
        delay(10);
    }

    if (gpsResponded) {
        Serial.println("OK");
        // Đặt lại vào chế độ ngủ
        putGPSToSleep();

        // Cập nhật thời gian phản hồi GPS
        lastGpsResponse = millis();
        gpsResponding = true;
    } else {
        Serial.println("KHÔNG PHẢN HỒI");
        // Phát 2 tiếng bíp dài nếu GPS không phản hồi
        for (int i = 0; i < 2; i++) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
    }

    // Chỉ kích hoạt SIM khi cần
    Serial.println("- Module SIM: Chưa kích hoạt (sẽ kích hoạt khi cần)");

    // Feed watchdog trước khi tiếp tục
    feedWatchdog();

    // Phát 2 tiếng bíp để xác nhận hoàn tất kiểm tra
    for (int i = 0; i < 2; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
    }

    // Khởi tạo bộ lọc Kalman
    mpuKalman.reset();
    gpsKalman.reset(0, 0);

    updateActivityTime(); // Cập nhật thời gian hoạt động
}

// Báo cáo trạng thái hệ thống qua Serial
void reportSystemStatus() {
    Serial.println("\n=== TRẠNG THÁI HỆ THỐNG ===");
    Serial.print("Thời gian hoạt động: ");
    Serial.print(millis() / 1000);
    Serial.println(" giây");

    Serial.print("Pin: ");
    Serial.print(batteryPercentage);
    Serial.println("%");

    Serial.print("Chủ sở hữu: ");
    Serial.println(ownerPresent ? "Có mặt" : "Không có mặt");

    Serial.print("Trạng thái báo động: ");
    switch(alarmStage) {
        case STAGE_NONE: Serial.println("Không có"); break;
        case STAGE_WARNING: Serial.println("Cảnh báo"); break;
        case STAGE_ALERT: Serial.println("Báo động"); break;
        case STAGE_TRACKING: Serial.println("Theo dõi"); break;
    }

    Serial.print("Phát hiện chuyển động: ");
    Serial.println(motionDetected ? "Có" : "Không");

    Serial.print("Mã lỗi: ");
    Serial.println(currentError);

    Serial.println("========================\n");
}

// Handler cho nút kiểm tra còi từ Blynk
BLYNK_WRITE(VPIN_CONTROL_TEST) {
        int pinValue = param.asInt();
        if (pinValue) {
            digitalWrite(BUZZER_PIN, HIGH);
            Serial.println("Kiểm tra còi: BẬT");
        } else {
            digitalWrite(BUZZER_PIN, LOW);
            Serial.println("Kiểm tra còi: TẮT");
        }
        updateActivityTime(); // Cập nhật thời gian hoạt động

        // Cập nhật thời gian phản hồi WiFi
        lastWifiResponse = millis();
        wifiResponding = true;
}

void setup() {
    // Khởi tạo Serial
    Serial.begin(115200);
    delay(1000);  // Thêm delay để đảm bảo khởi động ổn định

    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX cho GPS
    SerialSIM.begin(115200, SERIAL_8N1, 27, 26); // RX, TX cho SIM A7682S

    // Đặt chế độ cho các chân
    pinMode(BUZZER_PIN, OUTPUT);
    // LED đã bị vô hiệu hóa, không cần cấu hình
    pinMode(RF_POWER_PIN, OUTPUT);
    pinMode(GPS_POWER_PIN, OUTPUT);
    // Không sử dụng PWRKEY để điều khiển nguồn SIM
    pinMode(MPU_INT_PIN, INPUT_PULLUP);

    // Thiết lập trạng thái ban đầu
    digitalWrite(BUZZER_PIN, LOW);      // Tắt còi
    // Bỏ qua LED (đã vô hiệu hóa)
    digitalWrite(RF_POWER_PIN, HIGH);   // RF luôn BẬT - ĐÃ SỬA ĐỔI
    digitalWrite(GPS_POWER_PIN, HIGH);  // GPS luôn BẬT

    // Cập nhật biến trạng thái
    rfPowerState = true;  // RF luôn BẬT - ĐÃ SỬA ĐỔI

    // Đợi ESP32 ổn định sau khi khởi động
    delay(1000);

    // Khởi tạo Watchdog
    setupWatchdog();

    // Xử lý thức dậy từ sleep
    handleWakeup();

    // Khởi tạo biến đếm chuyển động và biến liên quan
    motionDetectionCount = 0;
    firstMotionTime = 0;
    initialBeepsDone = false;

    // Chỉ khởi tạo đầy đủ nếu là khởi động bình thường hoặc thức dậy từ deep sleep
    if (!isWakingUp || wakeupCause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        // Khởi tạo I2C cho MPU6050
        Wire.begin();

        // Khởi tạo MPU6050
        if (mpu.begin()) {
            // Cấu hình các thông số của MPU6050
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

            // Thiết lập ngắt cho MPU6050
            attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuISR, RISING);

            // Đặt trạng thái sleep ban đầu dựa trên sự hiện diện của chủ sở hữu
            mpuSleepState = false;
            if (ownerPresent) {
                putMPUToSleep();
            }

            Serial.println("MPU6050 đã được khởi tạo thành công");
        } else {
            Serial.println("Không thể khởi tạo MPU6050!");
            currentError = ERROR_MPU;
        }

        // Khởi tạo RF433
        rfReceiver.enableReceive(RF_DATA_PIN);
        Serial.println("RF433 đã được khởi tạo");
        
        // Khởi tạo hệ thống từ điển tiếng Việt
        if (vietnameseDictionary.begin()) {
            Serial.println("Hệ thống từ điển tiếng Việt đã khởi tạo thành công");
        } else {
            Serial.println("Cảnh báo: Không thể khởi tạo từ điển tiếng Việt");
        }

        // Cập nhật trạng thái GPS
        gpsPowerState = true;  // GPS luôn được cấp nguồn

        // Đặt GPS vào chế độ ngủ ban đầu để tiết kiệm pin
        putGPSToSleep();

        // Hiển thị thông tin phiên bản
        Serial.println("\n==============================");
        Serial.println("Hệ thống Chống trộm Xe ESP32");
        Serial.print("Phiên bản: ");
        Serial.println(APP_VERSION);
        Serial.print("Ngày cập nhật: ");
        Serial.println("2025-06-18 19:16:51");  // Cập nhật thời gian hiện tại
        Serial.print("Người phát triển: ");
        Serial.println("nguongthienTieu");
        Serial.println("==============================\n");

        // Feed watchdog trước khi tiếp tục
        feedWatchdog();

        // Chỉ phát tiếng bíp khởi động nếu đây là khởi động đầu tiên hoặc khởi động từ deep sleep
        if (bootCount <= 1) {
            handleInitialBeeps();
        }

        // RF luôn bật nguồn, không cần kích hoạt riêng

        // Khởi tạo các timer
        setupBaseTimers();

        // Cập nhật trạng thái pin ban đầu
        checkBatteryStatus();

        // Đặt chế độ WiFi là tắt ban đầu (chỉ kích hoạt khi cần)
        WiFi.mode(WIFI_OFF);
        wifiConnected = false;

        // Khởi tạo bộ lọc Kalman - chuẩn bị sẵn sàng
        mpuKalman.reset();
        gpsKalman.reset(0, 0);

        Serial.println("Hệ thống đã sẵn sàng và đang chờ tín hiệu...");
    }

    // Khởi tạo thời gian hoạt động
    updateActivityTime();
}

void loop() {
    // Biến thời gian hiện tại
    unsigned long currentMillis = millis();

    // Feed watchdog trong loop chính
    feedWatchdog();

    // Chạy timer của Blynk
    timer.run();

    // Xử lý ngắt MPU6050
    if (mpuInterrupt) {
        mpuInterrupt = false;
        handleMotionDetection();
        updateActivityTime(); // Cập nhật thời gian hoạt động
    }

    // Đọc dữ liệu GPS nếu đang hoạt động
    if (gpsActive) {
        while (SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
            updateActivityTime(); // Cập nhật thời gian hoạt động

            // Cập nhật thời gian phản hồi GPS
            lastGpsResponse = millis();
            gpsResponding = true;
        }
    }

    // Đọc và xử lý dữ liệu SMS từ module SIM
    if (simActive) {
        String smsData = "";
        while (SerialSIM.available()) {
            char c = SerialSIM.read();
            smsData += c;
            updateActivityTime(); // Cập nhật thời gian hoạt động

            // Cập nhật thời gian phản hồi SIM
            lastSimResponse = millis();
            simResponding = true;
        }

        // Xử lý các lệnh SMS
        if (smsData.indexOf("+CMT:") >= 0) {
            Serial.println("Đã nhận được SMS:");
            Serial.println(smsData);
            updateActivityTime(); // Cập nhật thời gian hoạt động

            // Feed watchdog khi nhận được SMS
            feedWatchdog();

            // Xử lý lệnh SOS
            if (smsData.indexOf(SMS_SOS_CODE) >= 0) {
                // Kích hoạt chế độ báo động khẩn cấp
                if (alarmStage == STAGE_NONE) {
                    alarmStage = STAGE_ALERT;
                    alarmStartTime = millis();

                    // Điều chỉnh timer cho trạng thái báo động
                    adjustTimersForAlarm(STAGE_ALERT);
                    previousAlarmStage = STAGE_ALERT;

                    // Kích hoạt GPS và gửi vị trí
                    activateGPS();

                    // Phản hồi rằng đã nhận được lệnh
                    String response = "Da nhan lenh SOS. Dang kich hoat bao dong va gui vi tri.";
                    sendSMSWithCooldown(response.c_str(), SMS_ALERT_ALARM);

                    Serial.println("Đã kích hoạt báo động SOS từ SMS");
                }
            }
                // Xử lý lệnh STATUS
            else if (smsData.indexOf(SMS_STATUS_CODE) >= 0) {
                // Gửi trạng thái hiện tại
                String statusMessage = "Trang thai: ";
                statusMessage += "Pin: " + String(batteryPercentage) + "%, ";
                statusMessage += "Bao dong: " + String((int)alarmStage) + ", ";
                statusMessage += "Chu so huu: " + String(ownerPresent ? "Co mat" : "Vang mat") + ", ";
                statusMessage += "MPU: " + String(mpuSleepState ? "Sleep" : "Active");

                sendSMSWithCooldown(statusMessage.c_str(), SMS_ALERT_SYSTEM_ERROR);

                Serial.println("Đã gửi trạng thái qua SMS");
            }
                // Xử lý lệnh GPS
            else if (smsData.indexOf(SMS_GPS_CODE) >= 0) {
                // Kích hoạt GPS nếu chưa hoạt động
                if (!gpsActive) {
                    activateGPS();
                    delay(3000);  // Giảm thời gian đợi xuống 3s
                }

                // Cập nhật vị trí
                updateGpsPosition();

                // Gửi vị trí qua SMS
                if (hadValidFix) {
                    String positionMessage = "Vi tri hien tai: ";
                    positionMessage += "https://www.google.com/maps/dir/?api=1&destination=";
                    positionMessage += String(currentLat, 6);
                    positionMessage += ",";
                    positionMessage += String(currentLng, 6);
                    positionMessage += "&travelmode=driving";

                    sendSMSWithCooldown(positionMessage.c_str(), SMS_ALERT_POSITION);

                    Serial.println("Đã gửi vị trí GPS qua SMS theo yêu cầu");
                } else {
                    sendSMSWithCooldown("Khong the xac dinh vi tri GPS. Vui long thu lai sau.", SMS_ALERT_GPS_LOST);
                    Serial.println("Không thể xác định vị trí GPS");
                }
            }
                // Xử lý lệnh STOP
            else if (smsData.indexOf(SMS_STOP_CODE) >= 0) {
                // Dừng báo động ở bất kỳ giai đoạn nào, kể cả TRACKING
                previousAlarmStage = alarmStage;
                alarmStage = STAGE_NONE;
                digitalWrite(BUZZER_PIN, LOW);  // Tắt còi

                // Khôi phục lại RF nếu có thể
                rfEnabled = true;
                // RF luôn bật nguồn, không cần kích hoạt riêng

                // Điều chỉnh timer về trạng thái bình thường
                adjustTimersForAlarm(STAGE_NONE);

                // Phản hồi rằng đã dừng báo động
                sendSMSWithCooldown("Da dung bao dong va che do theo doi theo yeu cau.", SMS_ALERT_ALARM);

                Serial.println("Đã dừng tất cả trạng thái báo động theo yêu cầu SMS");
            }
                // Xử lý lệnh SLEEP - Vào chế độ ngủ sâu
            else if (smsData.indexOf(SMS_SLEEP_CODE) >= 0) {
                if (alarmStage == STAGE_NONE) {
                    // Vào chế độ ngủ sâu tự động
                    sendSMSWithCooldown("He thong se vao che do ngu sau trong 10 giay...", SMS_ALERT_SYSTEM_ERROR);
                    delay(10000); // Đợi 10 giây trước khi vào ngủ sâu
                    goToDeepSleep(0); // 0 = sử dụng thời gian mặc định
                } else {
                    sendSMSWithCooldown("Khong the vao che do ngu khi dang trong trang thai bao dong.", SMS_ALERT_SYSTEM_ERROR);
                }
            }
                // Xử lý lệnh WAKE - Cập nhật thời gian hoạt động để ngăn vào sleep
            else if (smsData.indexOf(SMS_WAKE_CODE) >= 0) {
                updateActivityTime(); // Cập nhật thời gian hoạt động để tránh vào sleep ngay lập tức
                sendSMSWithCooldown("He thong dang duoc danh thuc va se khong vao che do ngu trong 10 phut.", SMS_ALERT_SYSTEM_ERROR);
            }
                // Xử lý lệnh RESET - Reset các module bị treo
            else if (smsData.indexOf("RESET") >= 0) {
                sendSMSWithCooldown("Dang thuc hien reset he thong...", SMS_ALERT_MODULE_RESET);
                delay(1000);

                // Reset các biến đếm số lần reset
                wifiResetCount = 0;
                simResetCount = 0;
                gpsResetCount = 0;

                // Reset tất cả timer
                resetAllTimers();

                // Thực hiện reset phần cứng
                performHardwareReset();
            }
                // Xử lý lệnh MPU - Điều khiển chế độ sleep của MPU6050
            else if (smsData.indexOf("MPU SLEEP") >= 0) {
                if (!mpuSleepState) {
                    putMPUToSleep();
                    sendSMSWithCooldown("Da dua MPU6050 vao che do ngu.", SMS_ALERT_SYSTEM_ERROR);
                } else {
                    sendSMSWithCooldown("MPU6050 da o trong che do ngu.", SMS_ALERT_SYSTEM_ERROR);
                }
            }
            else if (smsData.indexOf("MPU WAKE") >= 0) {
                if (mpuSleepState) {
                    wakeMPUFromSleep();
                    sendSMSWithCooldown("Da danh thuc MPU6050 tu che do ngu.", SMS_ALERT_SYSTEM_ERROR);
                } else {
                    sendSMSWithCooldown("MPU6050 da dang hoat dong.", SMS_ALERT_SYSTEM_ERROR);
                }
            }
                // Xử lý lệnh FILTER - Điều chỉnh tham số bộ lọc Kalman
            else if (smsData.indexOf("FILTER") >= 0) {
                // Thiết lập lại bộ lọc Kalman với các tham số mặc định
                mpuKalman = MPUKalmanFilter(0.5, 1.0, 0.05);
                gpsKalman = GPSKalmanFilter(0.1, 1.0, 0.01);

                sendSMSWithCooldown("Da cai dat lai bo loc Kalman ve tham so mac dinh.", SMS_ALERT_SYSTEM_ERROR);
                Serial.println("Đã thiết lập lại bộ lọc Kalman về tham số mặc định");
            }
            // Xử lý các lệnh từ điển tiếng Việt
            else if (smsData.indexOf("DICT STATS") >= 0) {
                // Thống kê từ điển
                String stats = "Tu dien: ";
                stats += String(vietnameseDictionary.getTotalWords()) + " tu, ";
                stats += String(vietnameseDictionary.getTotalCompoundWords()) + " tu ghep, ";
                std::vector<String> deadWords = vietnameseDictionary.getDeadWords();
                stats += String(deadWords.size()) + " tu chet";
                sendSMSWithCooldown(stats.c_str(), SMS_ALERT_SYSTEM_ERROR);
                Serial.println("Đã gửi thống kê từ điển");
            }
            else if (smsData.indexOf("DICT ADD") >= 0) {
                // Thêm từ mới: DICT ADD [từ]:[nghĩa]
                int colonPos = smsData.indexOf(":", smsData.indexOf("DICT ADD"));
                if (colonPos > 0) {
                    String word = smsData.substring(smsData.indexOf("DICT ADD") + 9, colonPos);
                    String meaning = smsData.substring(colonPos + 1);
                    word.trim();
                    meaning.trim();
                    
                    if (vietnameseDictionary.addWord(word, meaning)) {
                        String response = "Da them tu: " + word + " - " + meaning;
                        sendSMSWithCooldown(response.c_str(), SMS_ALERT_SYSTEM_ERROR);
                        Serial.println("Đã thêm từ mới qua SMS: " + word);
                    } else {
                        sendSMSWithCooldown("Loi: Khong the them tu moi.", SMS_ALERT_SYSTEM_ERROR);
                    }
                } else {
                    sendSMSWithCooldown("Cu phap: DICT ADD [tu]:[nghia]", SMS_ALERT_SYSTEM_ERROR);
                }
            }
            else if (smsData.indexOf("DICT COMPOUND") >= 0) {
                // Thêm từ ghép: DICT COMPOUND [từ1] [từ2]:[nghĩa]
                int spacePos = smsData.indexOf(" ", smsData.indexOf("DICT COMPOUND") + 14);
                int colonPos = smsData.indexOf(":", spacePos);
                if (spacePos > 0 && colonPos > 0) {
                    String firstPart = smsData.substring(smsData.indexOf("DICT COMPOUND") + 14, spacePos);
                    String secondPart = smsData.substring(spacePos + 1, colonPos);
                    String meaning = smsData.substring(colonPos + 1);
                    firstPart.trim();
                    secondPart.trim();
                    meaning.trim();
                    
                    if (vietnameseDictionary.addCompoundWord(firstPart, secondPart, meaning)) {
                        String response = "Da them tu ghep: " + firstPart + " " + secondPart + " - " + meaning;
                        sendSMSWithCooldown(response.c_str(), SMS_ALERT_SYSTEM_ERROR);
                        Serial.println("Đã thêm từ ghép mới qua SMS");
                    } else {
                        sendSMSWithCooldown("Loi: Khong the them tu ghep.", SMS_ALERT_SYSTEM_ERROR);
                    }
                } else {
                    sendSMSWithCooldown("Cu phap: DICT COMPOUND [tu1] [tu2]:[nghia]", SMS_ALERT_SYSTEM_ERROR);
                }
            }
            else if (smsData.indexOf("DICT DEAD") >= 0) {
                // Đánh dấu từ chết: DICT DEAD [từ]
                String word = smsData.substring(smsData.indexOf("DICT DEAD") + 10);
                word.trim();
                
                if (word.length() > 0) {
                    if (vietnameseDictionary.markWordAsDead(word, "Danh dau qua SMS")) {
                        String response = "Da danh dau tu chet: " + word;
                        sendSMSWithCooldown(response.c_str(), SMS_ALERT_SYSTEM_ERROR);
                        Serial.println("Đã đánh dấu từ chết qua SMS: " + word);
                    } else {
                        sendSMSWithCooldown("Loi: Khong tim thay tu hoac khong the danh dau.", SMS_ALERT_SYSTEM_ERROR);
                    }
                } else {
                    sendSMSWithCooldown("Cu phap: DICT DEAD [tu]", SMS_ALERT_SYSTEM_ERROR);
                }
            }
            else if (smsData.indexOf("DICT REVIVE") >= 0) {
                // Khôi phục từ chết: DICT REVIVE [từ]
                String word = smsData.substring(smsData.indexOf("DICT REVIVE") + 12);
                word.trim();
                
                if (word.length() > 0) {
                    if (vietnameseDictionary.reviveWord(word)) {
                        String response = "Da khoi phuc tu: " + word;
                        sendSMSWithCooldown(response.c_str(), SMS_ALERT_SYSTEM_ERROR);
                        Serial.println("Đã khôi phục từ qua SMS: " + word);
                    } else {
                        sendSMSWithCooldown("Loi: Khong tim thay tu chet hoac khong the khoi phuc.", SMS_ALERT_SYSTEM_ERROR);
                    }
                } else {
                    sendSMSWithCooldown("Cu phap: DICT REVIVE [tu]", SMS_ALERT_SYSTEM_ERROR);
                }
            }
            else if (smsData.indexOf("DICT SEARCH") >= 0) {
                // Tìm kiếm từ: DICT SEARCH [pattern]
                String pattern = smsData.substring(smsData.indexOf("DICT SEARCH") + 12);
                pattern.trim();
                
                if (pattern.length() > 0) {
                    std::vector<String> results = vietnameseDictionary.searchWords(pattern);
                    String response = "Tim thay " + String(results.size()) + " tu: ";
                    for (size_t i = 0; i < min((size_t)5, results.size()); i++) {
                        if (i > 0) response += ", ";
                        response += results[i];
                    }
                    if (results.size() > 5) response += "...";
                    sendSMSWithCooldown(response.c_str(), SMS_ALERT_SYSTEM_ERROR);
                } else {
                    sendSMSWithCooldown("Cu phap: DICT SEARCH [pattern]", SMS_ALERT_SYSTEM_ERROR);
                }
            }
            else if (smsData.indexOf("DICT CLEANUP") >= 0) {
                // Dọn dẹp từ chết
                vietnameseDictionary.cleanupDeadWords();
                sendSMSWithCooldown("Da don dep cac tu chet khong su dung.", SMS_ALERT_SYSTEM_ERROR);
                Serial.println("Đã dọn dẹp từ chết qua SMS");
            }
        }
    }

    // Xử lý Blynk nếu đang kết nối
    if (Blynk.connected()) {
        Blynk.run();
        updateActivityTime(); // Cập nhật thời gian hoạt động

        // Cập nhật thời gian phản hồi WiFi
        lastWifiResponse = millis();
        wifiResponding = true;
    }

    // Kiểm tra timeout kết nối mạng
    checkNetworkTimeout();

    // Xử lý các giai đoạn báo động
    handleAlarmStages();

    // Nếu đang trong trạng thái báo động, cập nhật thời gian hoạt động
    if (alarmStage != STAGE_NONE) {
        updateActivityTime();
    }

    // Báo cáo trạng thái hệ thống định kỳ
    if (currentMillis % 60000 < 10) {  // Mỗi phút
        reportSystemStatus();
    }

    // Hiển thị mã lỗi qua còi thay vì LED (vì LED đã bị vô hiệu hóa)
    if (currentError != ERROR_NONE) {
        // Phát âm thanh cảnh báo lỗi mỗi 5 giây
        if (currentMillis % 5000 < 100) {  // Chỉ phát âm thanh mỗi 5 giây một lần, trong 100ms
            for (int i = 0; i < currentError; i++) {
                digitalWrite(BUZZER_PIN, HIGH);
                delay(50);
                digitalWrite(BUZZER_PIN, LOW);
                delay(50);
                feedWatchdog();
            }
        }
    }
}