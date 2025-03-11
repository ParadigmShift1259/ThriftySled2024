// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#define LED

constexpr double c_turnGearRatio = 25.0;

constexpr double kMinOut = -1.0;
constexpr double kMaxOut = 1.0;

enum ELevels { L1, L2, L3, L4, algaeRemovalL3_4, algaeRemovalL2_3 };

namespace DriveConstants
{
    inline constexpr double kTurnVoltageToRadians = 2.0 * std::numbers::pi / 4.78;    // Absolute encoder runs 0 to 4.78V
    inline constexpr double KTurnVoltageToDegrees = 360 / 4.78;

    inline constexpr double c_HolomonicTranslateP = 3.5;
    inline constexpr double c_HolomonicRotateP = 1.5;
}

namespace OperatorConstants
{
    inline constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants
