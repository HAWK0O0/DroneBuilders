#include "AHRS/ahrs_manager.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#include "Control/controller_math.h"
#include "INPUT/drone_config.h"

namespace {

// BNO055 IMU sensor instances (primary and secondary addresses)
// نسخ مستشعر IMU BNO055 (العنوان الأساسي والثانوي)
// 中文: BNO055 IMU传感器实例（主地址和备用地址）
Adafruit_BNO055 g_bno_a(55, DroneConfig::kBnoAddressA, &Wire);
Adafruit_BNO055 g_bno_b(55, DroneConfig::kBnoAddressB, &Wire);
Adafruit_BNO055* g_bno = nullptr;  // Active sensor pointer / مؤشر المستشعر النشط / 活动传感器指针

// IMU data snapshot
// لقطة بيانات IMU
// 中文: IMU数据快照
ImuSnapshot g_snapshot{};

// Timing and calibration variables
// متغيرات التوقيت والمعايرة
// 中文: 时间和校准变量
uint32_t g_last_read_us = 0;           // Last sensor read time / وقت آخر قراءة للمستشعر / 上次传感器读取时间
uint32_t g_last_calibration_ms = 0;    // Last calibration check / آخر فحص للمعايرة / 上次校准检查
float g_roll_offset_deg = 0.0f;        // Roll angle offset / إزاحة زاوية الدوران / 横滚角偏移
float g_pitch_offset_deg = 0.0f;       // Pitch angle offset / إزاحة زاوية الميل / 俯仰角偏移
float g_raw_roll_deg = 0.0f;           // Raw roll from sensor / الدوران الخام من المستشعر / 传感器原始横滚
float g_raw_pitch_deg = 0.0f;          // Raw pitch from sensor / الميل الخام من المستشعر / 传感器原始俯仰
float g_raw_yaw_deg = 0.0f;            // Raw yaw from sensor / الانعراج الخام من المستشعر / 传感器原始偏航

constexpr uint8_t kMinCalSystem = 1;
constexpr uint8_t kMinCalGyro = 2;
constexpr uint8_t kMinCalAccel = 2;
constexpr uint32_t kBootCalibrationWaitMs = 7000;
constexpr uint32_t kBootCalibrationPollMs = 30;
constexpr uint8_t kLevelCaptureSamples = 24;
constexpr uint32_t kLevelCaptureDelayMs = 12;
constexpr float kRadToDeg = 57.2957795f;

bool ReadSensorEvents(sensors_event_t* euler, sensors_event_t* gyro) {
  if (g_bno == nullptr || euler == nullptr || gyro == nullptr) return false;

  g_bno->getEvent(euler, Adafruit_BNO055::VECTOR_EULER);
  g_bno->getEvent(gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

  return isfinite(euler->orientation.x) && isfinite(euler->orientation.y) &&
         isfinite(euler->orientation.z) && isfinite(gyro->gyro.x) && isfinite(gyro->gyro.y) &&
         isfinite(gyro->gyro.z);
}

void ApplySensorEvents(const sensors_event_t& euler, const sensors_event_t& gyro) {
  g_raw_yaw_deg = euler.orientation.x * DroneConfig::kYawSign;
  g_raw_roll_deg = euler.orientation.z * DroneConfig::kRollSign;
  g_raw_pitch_deg = euler.orientation.y * DroneConfig::kPitchSign;

  g_snapshot.yaw_deg = ControllerMath::WrapDegrees180(g_raw_yaw_deg);
  g_snapshot.roll_deg = g_raw_roll_deg - g_roll_offset_deg;
  g_snapshot.pitch_deg = g_raw_pitch_deg - g_pitch_offset_deg;
  g_snapshot.roll_rate_dps = gyro.gyro.y * kRadToDeg * DroneConfig::kRollSign;
  g_snapshot.pitch_rate_dps = gyro.gyro.x * kRadToDeg * DroneConfig::kPitchSign;
  g_snapshot.yaw_rate_dps = gyro.gyro.z * kRadToDeg * DroneConfig::kYawSign;
}

// Initialize BNO055 sensor with fallback to secondary address
// تهيئة مستشعر BNO055 مع التراجع للعنوان الثانوي
// 中文: 初始化BNO055传感器，失败时回退到备用地址
bool InitBno() {
  g_bno = &g_bno_a;
  if (!g_bno->begin(OPERATION_MODE_NDOF)) {
    g_bno = &g_bno_b;
    if (!g_bno->begin(OPERATION_MODE_NDOF)) {
      g_bno = nullptr;
      return false;
    }
  }

  // Some MCU-55 / GY-BNO055 boards ship without the external crystal soldered.
  // Forcing external crystal mode in that case causes unstable startup and poor calibration.
  g_bno->setExtCrystalUse(DroneConfig::kBnoUseExternalCrystal);
  return true;
}

// Update calibration status from sensor
// تحديث حالة المعايرة من المستشعر
// 中文: 从传感器更新校准状态
void UpdateCalibrationStatus() {
  if (!g_snapshot.ready || g_bno == nullptr) {
    g_snapshot.calibrated = false;
    g_snapshot.cal_sys = 0;
    g_snapshot.cal_gyro = 0;
    g_snapshot.cal_accel = 0;
    g_snapshot.cal_mag = 0;
    return;
  }

  g_bno->getCalibration(&g_snapshot.cal_sys, &g_snapshot.cal_gyro, &g_snapshot.cal_accel,
                        &g_snapshot.cal_mag);
  g_snapshot.calibrated = g_snapshot.cal_sys >= kMinCalSystem &&
                          g_snapshot.cal_gyro >= kMinCalGyro &&
                          g_snapshot.cal_accel >= kMinCalAccel;
}

}  // namespace

void ImuManager::begin(float roll_offset_deg, float pitch_offset_deg) {
  g_roll_offset_deg = roll_offset_deg;
  g_pitch_offset_deg = pitch_offset_deg;
  Wire.begin(DroneConfig::kI2cSdaPin, DroneConfig::kI2cSclPin, 400000U);

  g_snapshot = {};
  g_snapshot.ready = InitBno();
  g_last_read_us = micros();
  g_last_calibration_ms = millis();

  if (!g_snapshot.ready || g_bno == nullptr) {
    UpdateCalibrationStatus();
    return;
  }

  // Give the internal fusion stack a short startup window and wait for acceptable calibration.
  const uint32_t wait_start_ms = millis();
  while ((millis() - wait_start_ms) < kBootCalibrationWaitMs) {
    UpdateCalibrationStatus();
    if (g_snapshot.calibrated) break;
    delay(kBootCalibrationPollMs);
  }

  sensors_event_t euler{};
  sensors_event_t gyro{};
  if (ReadSensorEvents(&euler, &gyro)) {
    ApplySensorEvents(euler, gyro);
  }
  UpdateCalibrationStatus();
}

void ImuManager::update() {
  if (!g_snapshot.ready || g_bno == nullptr) return;

  const uint32_t now_us = micros();
  if ((now_us - g_last_read_us) < DroneConfig::kImuReadPeriodUs) return;

  g_last_read_us = now_us;

  sensors_event_t euler{};
  sensors_event_t gyro{};
  if (ReadSensorEvents(&euler, &gyro)) {
    ApplySensorEvents(euler, gyro);
  }

  if ((millis() - g_last_calibration_ms) >= 250) {
    g_last_calibration_ms = millis();
    UpdateCalibrationStatus();
  }
}

ImuSnapshot ImuManager::snapshot() {
  return g_snapshot;
}

bool ImuManager::isReady() {
  return g_snapshot.ready;
}

bool ImuManager::captureLevelOffsets(float* roll_offset_deg, float* pitch_offset_deg) {
  if (!g_snapshot.ready || roll_offset_deg == nullptr || pitch_offset_deg == nullptr) {
    return false;
  }

  UpdateCalibrationStatus();
  if (!g_snapshot.calibrated) {
    return false;
  }

  float roll_sum = 0.0f;
  float pitch_sum = 0.0f;
  uint8_t sample_count = 0;
  for (uint8_t i = 0; i < kLevelCaptureSamples; ++i) {
    sensors_event_t euler{};
    sensors_event_t gyro{};
    if (ReadSensorEvents(&euler, &gyro)) {
      roll_sum += euler.orientation.z * DroneConfig::kRollSign;
      pitch_sum += euler.orientation.y * DroneConfig::kPitchSign;
      ++sample_count;
    }
    delay(kLevelCaptureDelayMs);
  }

  if (sample_count == 0) {
    return false;
  }

  *roll_offset_deg = roll_sum / static_cast<float>(sample_count);
  *pitch_offset_deg = pitch_sum / static_cast<float>(sample_count);
  return true;
}

void ImuManager::setLevelOffsets(float roll_offset_deg, float pitch_offset_deg) {
  g_roll_offset_deg = roll_offset_deg;
  g_pitch_offset_deg = pitch_offset_deg;
}
