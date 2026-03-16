#pragma once

// AHRS wrapper for the BNO055 attitude source.
// واجهة AHRS لمستشعر BNO055 الخاص بالاتجاه.
// 中文: 面向 BNO055 姿态传感器的 AHRS 封装。

#include "INPUT/drone_types.h"

namespace ImuManager {

void begin(float roll_offset_deg, float pitch_offset_deg);
void update();
ImuSnapshot snapshot();
bool isReady();
bool captureLevelOffsets(float* roll_offset_deg, float* pitch_offset_deg);
void setLevelOffsets(float roll_offset_deg, float pitch_offset_deg);

}  // namespace ImuManager
