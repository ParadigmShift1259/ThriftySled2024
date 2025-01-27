#pragma once
#include "Constants.h"

constexpr int kFrontLeftDriveCANID    = 1;       //!< Front Left Drive CAN ID (TalonFX)   
constexpr int kFrontLeftTurningCANID  = 2;       //!< Front Left Turn CAN ID (SparkFlex)   

constexpr int kFrontRightDriveCANID   = 3;       //!< Front Right Drive CAN ID (TalonFX)   
constexpr int kFrontRightTurningCANID = 4;       //!< Front Right Turn CAN ID (SparkFlex)

constexpr int kRearRightDriveCANID    = 5;       //!< Rear Right Drive CAN ID (TalonFX)   
constexpr int kRearRightTurningCANID  = 6;       //!< Rear Right Turn CAN ID (SparkFlex)

constexpr int kRearLeftDriveCANID     = 7;       //!< Rear Left Drive CAN ID (TalonFX)   
constexpr int kRearLeftTurningCANID   = 8;       //!< Rear Left Turn CAN ID (SparkFlex)

constexpr int kCoralManipRollerCANID   = 21;       // TalonSRX
constexpr int kCoralManipDeployCANID   = 24;       // SparkMax

constexpr int kIntakeRollerCANID       = 11;       // TalonSRX
constexpr int kIntakeDeployCANID       = 14;       // SparkMax
constexpr int kIntakeDeployFollowCANID = 16;       // SparkMax

// constexpr int kShooterElevationControllerCANID = 15;

constexpr int kDrivePigeonCANID = 1;
// constexpr int kShooterPigeonCANID = 2;

constexpr int kLEDCANID = 1;

constexpr int kClimbMotorCANID = 22;

constexpr int kElevatorLeadMotorCANID = 23;
constexpr int kElevatorFollowMotorCANID = 24;