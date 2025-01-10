// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <string>
#include <wpi/DataLog.h>

#include <frc/Encoder.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/AnalogInput.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>

#include <networktables/NetworkTableEntry.h>

#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

// using namespace ctre::phoenix6::hardware::can;
using namespace ctre::phoenix6::hardware;
using namespace rev::spark;

class SwerveModule
{
public:
    SwerveModule(int driveMotorCanId, int turningMotorCanId, double offset, bool driveMotorReversed);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(frc::SwerveModuleState& state);
    void Periodic();
    void ResyncAbsRelEnc();
    void SetMaxSpeed(units::meters_per_second_t newMaxSpeed) { m_currentMaxSpeed = newMaxSpeed; }
    TalonFX& GetTalon() { return m_driveMotor; };

private:
    units::meters_per_second_t CalcMetersPerSec();
    units::meter_t CalcMeters();
    units::radian_t GetTurnPosition();
    double VoltageToRadians(double Voltage);

    //static constexpr double kWheelRadius = 0.0508;
    //static constexpr int kEncoderResolution = 4096;
    static constexpr int kEncoderResolution = 7168;
    //static constexpr double kTurnMotorRevsPerWheelRev = 150.0 / 7.0; //!< The steering gear ratio of the MK4i is 150/7:1
    static constexpr double kTurnMotorRevsPerWheelRev = 25.0; //!< The steering gear ratio of the ThriftySwere is 25:1
    //static constexpr double kTurnMotorRevsPerWheelRev = 12.8;        //!< The steering gear ratio of the MK4 is 12.8:1

    static constexpr auto kModuleMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

    // https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
    // static constexpr double kEncoderCPR = 2048.0;                  //!< Falcon internal encoder is 2048 CPR
    // static constexpr double kEncoderTicksPerSec = 10.0;            //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    // static constexpr units::angular_velocity::turns_per_second_t kEncoderRevPerSec = units::angular_velocity::turns_per_second_t(kEncoderTicksPerSec / kEncoderCPR);  //!< CPR counts per rev is the same as ticks per rev
    static constexpr units::meter_t kWheelDiameterMeters = 4_in;          //!< 4"
    static constexpr units::meter_t kWheelCircumfMeters = kWheelDiameterMeters * std::numbers::pi;  //!< Distance driven per wheel rev

    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    // The MK4i is available in 3 different drive gear ratios. 
    // The table below shows the drive gear ratios and free speeds with NEO and Falcon 500 motors. 
    // L1 and L2 ratios are the most popular ratios and are suitable for standard full weight competition robots. 
    // The L3 ratio is more aggressive and is recommended for light weight robots.
    static constexpr double kDriveGearRatioL1 = 8.14;    //!< MK4i swerve modules L1 gearing w/Falcon 13.5 ft/sec
    static constexpr double kDriveGearRatioL2 = 6.75;    //!< MK4i swerve modules L2 gearing w/Falcon 16.3 ft/sec
    static constexpr double kDriveGearRatioL3 = 6.12;    //!< MK4i swerve modules L3 gearing w/Falcon 18.0 ft/sec


    static constexpr double kDriveGearRatio2ndStage18P12 = 6.75; //!< Thrifty Swerve modules 18T 12P gearing w/Kraken 15.5 ft/s (4.72 m/s)
    static constexpr double kDriveGearRatio2ndStage18P13 = 6.23; //!< Thrifty Swerve modules 18T 13P gearing w/Kraken 16.8 ft/s (5.12 m/s)
    static constexpr double kDriveGearRatio2ndStage18P14 = 5.79; //!< Thrifty Swerve modules 18T 14P gearing w/Kraken 18.1 ft/s (5.52 m/s)
    static constexpr double kDriveGearRatio2ndStage16P12 = 6.00; //!< Thrifty Swerve modules 16T 12P gearing w/Kraken 17.5 ft/s (5.21 m/s)
    static constexpr double kDriveGearRatio2ndStage16P13 = 5.54; //!< Thrifty Swerve modules 16T 13P gearing w/Kraken 18.9 ft/s (5.76 m/s)
    static constexpr double kDriveGearRatio2ndStage16P14 = 5.14; //!< Thrifty Swerve modules 16T 14P gearing w/Kraken 20.4 ft/s (6.22 m/s)

    // static constexpr double kDriveGearRatio = kDriveGearRatioL3; | Old Gear Ratios

    static constexpr double kDriveGearRatio = kDriveGearRatio2ndStage16P13;
    /// Assumes the encoders are mounted on the motor shaft
    /// ticks / 100 ms -> ticks / s -> motor rev / s -> wheel rev / s -> m / s
    // static constexpr units::meters_per_second_t kDriveEncoderMetersPerSec = kEncoderRevPerSec / kDriveGearRatio * kWheelCircumfMeters;
    // static constexpr double kDriveEncoderMetersPerTurn = kWheelCircumfMeters.to<double>() / (kEncoderCPR * kDriveGearRatio);
    units::meters_per_second_t m_currentMaxSpeed = 1.0_mps;

    TalonFX m_driveMotor;
    SparkFlex m_turningMotor;

    std::string m_id;
    bool m_driveMotorReversed = false;

    // SparkMaxAlternateEncoder m_turningEncoder = m_turningMotor.GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature, kEncoderResolution);
//    rev::SparkRelativeEncoder m_turningEncoder = m_turningMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 7168);
    // Use CPR 42 due to DS error "countsPerRev must be 42 when using the hall sensor"

    SparkRelativeEncoder m_turningEncoder = m_turningMotor.GetEncoder(); 
    SparkFlexConfig m_turningConfig;

    frc::AnalogInput m_absEnc;
    double m_offset = 0.0;
    
    SparkClosedLoopController  m_turningPIDController = m_turningMotor.GetClosedLoopController();
    double m_turnP;
    double m_turnI;
    double m_turnD;

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 3_V / 1_mps};

    /// Timer used to sync absolute and relative encoders on robot turn on
    frc::Timer m_timer;

    // Logging Member Variables
    wpi::log::DoubleLogEntry m_logTurningEncoderPosition;
    wpi::log::DoubleLogEntry m_logAbsoluteEncoderPosition;
    wpi::log::DoubleLogEntry m_logAbsoluteEncoderPositionWithOffset;
    wpi::log::DoubleLogEntry m_logTurningRefSpeed;
    wpi::log::DoubleLogEntry m_logTurningRefAngle;
    wpi::log::DoubleLogEntry m_logTurningNewAngle;
    wpi::log::DoubleLogEntry m_logDriveNewSpeed;
    wpi::log::DoubleLogEntry m_logDriveNormalizedSpeed;

    std::array<std::string, 4> m_nameArray = 
    {
        "FrontLeft"
    ,   "FrontRight"
    ,   "RearRight"
    ,   "RearLeft"
    };
};
