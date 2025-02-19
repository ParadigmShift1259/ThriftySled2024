// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>

#include <cstdlib>

SwerveModule::SwerveModule(const int driveMotorCanId, const int turningMotorCanId, double offset, bool driveMotorReversed)
  : m_driveMotor(driveMotorCanId)
  , m_turningMotor(turningMotorCanId, SparkLowLevel::MotorType::kBrushless)
  , m_id(std::to_string(turningMotorCanId / 2))
  , m_driveMotorReversed(driveMotorReversed)
  , m_absEnc((turningMotorCanId / 2) - 1)
  , m_offset(offset)
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  std::string logHeader = "/swerveModule" + m_id + "/";
  m_logTurningEncoderPosition = wpi::log::DoubleLogEntry(log, logHeader + "turningEncoderPosition");
  m_logAbsoluteEncoderPosition = wpi::log::DoubleLogEntry(log, logHeader + "absoluteEncoderPosition");
  m_logAbsoluteEncoderPositionWithOffset = wpi::log::DoubleLogEntry(log, logHeader + "absoluteEncoderPositionWithOffset");
  m_logTurningRefSpeed = wpi::log::DoubleLogEntry(log, logHeader + "refSpeed");
  m_logTurningRefAngle = wpi::log::DoubleLogEntry(log, logHeader + "refAngle");
  m_logTurningNewAngle = wpi::log::DoubleLogEntry(log, logHeader + "newAngle");
  m_logDriveNewSpeed = wpi::log::DoubleLogEntry(log, logHeader + "newSpeed");
  m_logDriveNormalizedSpeed = wpi::log::DoubleLogEntry(log, logHeader + "normalizedNewSpeed");

  m_turningConfig.encoder.PositionConversionFactor(2.0 * std::numbers::pi); //<! Converts from wheel rotations to radians
  m_turningConfig.Inverted(true);
  // Absolute encoders are on the wheel shaft, convert to the motor shaft via the gear ratio
  // Negative sign is because turning motor rotates opposite wheel
  double initPosition = -c_turnGearRatio * VoltageToRadians(m_absEnc.GetVoltage());
  m_turningEncoder.SetPosition(initPosition);

  ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitConfigs;
  currentLimitConfigs.WithStatorCurrentLimit(60.0_A);
  currentLimitConfigs.WithStatorCurrentLimitEnable(true);
  currentLimitConfigs.WithSupplyCurrentLimit(60.0_A);
  currentLimitConfigs.WithSupplyCurrentLimitEnable(true);
  currentLimitConfigs.WithSupplyCurrentLowerLimit(70.0_A); //Will do nothing right now, larger than SupplyCurrentLimit
  currentLimitConfigs.WithSupplyCurrentLowerTime(0.85_s);

  m_driveMotor.SetPosition(units::angle::turn_t(0.0));

  constexpr double kDriveP = 0.0025; // 0.1;
  constexpr double kDriveI = 0;
  constexpr double kDriveD = 0;
  // constexpr double kDriveFF = 0.055;//0.047619;
  constexpr double m_max = 1.0;
  constexpr double m_min = -1.0;

  ctre::phoenix6::configs::Slot0Configs slot0Configs;
  slot0Configs.WithKP(kDriveP);
  slot0Configs.WithKI(kDriveI);
  slot0Configs.WithKD(kDriveD);
  // m_driveMotor.Config_kF(0, kDriveFF);

  ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs;
  motorOutputConfigs.WithPeakForwardDutyCycle(m_max);
  motorOutputConfigs.WithPeakReverseDutyCycle(m_min);
  motorOutputConfigs.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  m_driveMotor.GetConfigurator().Apply(currentLimitConfigs);
  m_driveMotor.GetConfigurator().Apply(slot0Configs);
  m_driveMotor.GetConfigurator().Apply(motorOutputConfigs);

  m_turningConfig.closedLoop.SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningConfig.closedLoop.OutputRange(-1.0, 1.0); // -1 to 1 means full power
  constexpr double kTurnP = 0.03;
  constexpr double kTurnI = 0.0;
  constexpr double kTurnD = 0.0;
  m_turnP = kTurnP;
  m_turnI = kTurnI;
  m_turnD = kTurnD;
  frc::SmartDashboard::PutNumber("SwrvP", m_turnP);
  frc::SmartDashboard::PutNumber("SwrvI", m_turnI);
  frc::SmartDashboard::PutNumber("SwrvD", m_turnD);
  m_turningConfig.closedLoop.Pid(m_turnP, m_turnI, m_turnD);
  m_turningConfig.SmartCurrentLimit(20).SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
  
  m_turningMotor.Configure(m_turningConfig, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kPersistParameters); // TODO: VERIFY CONFIGURATION
  m_timer.Reset();
  m_timer.Start();

//#define SWERVE_ABS_ENC_TUNE
#ifdef SWERVE_ABS_ENC_TUNE
  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("AbsEncTuning");
  std::string name = m_nameArray[stoi(m_id) - 1];
  std::string nteName = name + " offset";
  const wpi::StringMap<nt::Value> sliderPropMap
  {
    std::make_pair("Min", nt::Value::MakeDouble(0.0))
  , std::make_pair("Max", nt::Value::MakeDouble(2 * std::numbers::pi))
  , std::make_pair("Block increment", nt::Value::MakeDouble(std::numbers::pi / 180))
  };
  tab.Add(nteName, m_offset)
    .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    .WithProperties(sliderPropMap)
    .GetEntry();
  nteName = name + " voltage";
  tab.Add(nteName, m_turningMotor.GetAnalog(rev::SparkAnalogSensor::Mode::kRelative).GetVoltage())
    .WithWidget(frc::BuiltInWidgets::kVoltageView)
    .GetEntry();
#endif
  frc::SmartDashboard::PutNumber("NewRef", 0.0);
  frc::SmartDashboard::PutNumber("Offset" + m_id, 0.0);

  auto pos = m_driveMotor.GetPosition().GetValue().value();
  frc::SmartDashboard::PutNumber("DrvTurns" + m_id, pos);
  frc::SmartDashboard::PutNumber("DrvMtrs" + m_id, kWheelCircumfMeters.value() * pos);
  frc::SmartDashboard::PutNumber("DrvGrRt" + m_id, kWheelCircumfMeters.value() * pos / units::angle::turn_t(kDriveGearRatio).value());
}

void SwerveModule::Periodic()
{
  // auto time = m_timer.Get();
  // if (time < 5.0_s)
  // {
  //   ResyncAbsRelEnc();
  // }

  //static int count = 0;
  // if (count++ % 50)
  // {
  //   ResyncAbsRelEnc();
  // }
  double turnP = frc::SmartDashboard::GetNumber("SwrvP", m_turnP);
  double turnI = frc::SmartDashboard::GetNumber("SwrvI", m_turnI);
  double turnD = frc::SmartDashboard::GetNumber("SwrvD", m_turnD);
  bool updateConfig = false;
  if (turnP != m_turnP)
  {
    m_turnP = turnP;
    m_turningConfig.closedLoop.P(m_turnP);
    updateConfig = true;
  }
  if (turnI != m_turnI)
  {
    m_turnI = turnI;
    m_turningConfig.closedLoop.I(m_turnI);
    updateConfig = true;
  }
  if (turnD != m_turnD)
  {
    m_turnD = turnD;
    m_turningConfig.closedLoop.D(m_turnD);
    updateConfig = true;
  }
  if (updateConfig)
  {
    m_turningMotor.Configure(m_turningConfig, SparkFlex::ResetMode::kNoResetSafeParameters, SparkFlex::PersistMode::kPersistParameters); // TODO: VERIFY CONFIGURATION
  }

  // Log relative encoder and absolute encoder positions (radians)
  double absPos = VoltageToRadians(m_absEnc.GetVoltage());
  frc::SmartDashboard::PutNumber("Abs Pos" + m_id, absPos);
  frc::SmartDashboard::PutNumber("Abs Pos Offset" + m_id, m_offset);  
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, GetTurnPosition().to<double>());

  auto pos = m_driveMotor.GetPosition().GetValue().value();
  frc::SmartDashboard::PutNumber("DrvTurns" + m_id, pos);
  frc::SmartDashboard::PutNumber("DrvMtrs" + m_id, kWheelCircumfMeters.value() * pos);
  frc::SmartDashboard::PutNumber("DrvGrRt" + m_id, kWheelCircumfMeters.value() * pos / units::angle::turn_t(kDriveGearRatio).value());

  m_logTurningEncoderPosition.Append(GetTurnPosition().to<double>());
  m_logAbsoluteEncoderPosition.Append(absPos);
}

void SwerveModule::ResyncAbsRelEnc()
{
  // Absolute encoders are on the wheel shaft, convert to the motor shaft via the gear ratio
  // Negative sign is because turning motor rotates opposite wheel
  auto angleInRad = -c_turnGearRatio * VoltageToRadians(m_absEnc.GetVoltage());         // Returns rotations between 0 and 1
  m_turningEncoder.SetPosition(angleInRad);

//#define PRINT_ABS_RESYNC
#ifdef PRINT_ABS_RESYNC
  auto time = m_timer.Get();
  printf("Module %s %.3f offset %.3f Set abs enc %.3f [rad] to rel enc %.3f [rad] mot pos %.3f [rot]\n"
        , m_id.c_str()
        , time.to<double>()
        , m_offset
        , angleInRad
        , -1.0 * m_turningEncoder.GetPosition()
        , -1.0 * m_turningEncoder.GetPosition() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
#endif
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return { CalcMetersPerSec(), GetTurnPosition() };
}

units::radian_t SwerveModule::GetTurnPosition()
{
  // Negative sign is because turning motor rotates opposite wheel
  return units::radian_t{ m_turningEncoder.GetPosition() / -c_turnGearRatio};
}

units::meters_per_second_t SwerveModule::CalcMetersPerSec()
{
  return kWheelCircumfMeters * m_driveMotor.GetVelocity().GetValue() / units::angle::turn_t(kDriveGearRatio);
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
  return { CalcMeters(), GetTurnPosition() };
}

units::meter_t SwerveModule::CalcMeters()
{
  return kWheelCircumfMeters * m_driveMotor.GetPosition().GetValue() / units::angle::turn_t(kDriveGearRatio);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& referenceState)
{
  // Need to log before referenceState is optimized
  m_logTurningRefSpeed.Append(referenceState.speed.to<double>());
  m_logTurningRefAngle.Append(referenceState.angle.Degrees().to<double>());

  // Optimize the reference state to avoid spinning further than 90 degrees
  double currPosition = GetTurnPosition().to<double>();
  referenceState.Optimize(frc::Rotation2d{ units::radian_t(currPosition) });
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, currPosition);

  if (referenceState.speed != 0_mps)
  {
#ifdef DISABLE_DRIVE
    m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
#else
    auto spd = (referenceState.speed).to<double>();
    spd *= m_driveMotorReversed ? -1.0 : 1.0;
    m_driveMotor.Set(spd);  
#endif
  }
  else
  {
    m_driveMotor.Set(0.0);
  }

  // Calculate the turning motor output from the turning PID controller.
  // Negative sign is because turning motor rotates opposite wheel
  double newRef = -c_turnGearRatio * referenceState.angle.Radians().to<double>();

  m_logTurningNewAngle.Append(referenceState.angle.Degrees().to<double>());
  m_logDriveNewSpeed.Append(referenceState.speed.to<double>());
  m_logDriveNormalizedSpeed.Append((referenceState.speed / m_currentMaxSpeed).to<double>());

  frc::SmartDashboard::PutNumber("Turn Ref Motor" + m_id, newRef);
  m_turningPIDController.SetReference(newRef, SparkBase::ControlType::kPosition);
}

double SwerveModule::VoltageToRadians(double Voltage)
{
    frc::SmartDashboard::PutNumber("Voltage" + m_id, Voltage);
    double angle = Voltage * DriveConstants::kTurnVoltageToRadians;
    angle -= m_offset;
    angle = fmod(angle + 2 * std::numbers::pi, 2 * std::numbers::pi);

    return angle;
}