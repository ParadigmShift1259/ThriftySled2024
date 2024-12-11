// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

constexpr double c_turnGearRatio = 25.0;

namespace DriveConstants
{
    inline constexpr double kTurnVoltageToRadians = 2.0 * std::numbers::pi / 4.93;    // Absolute encoder runs 0 to 4.93V
    inline constexpr double KTurnVoltageToDegrees = 360 / 4.93;
}

namespace OperatorConstants
{
    inline constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants