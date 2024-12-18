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
  , m_turningMotor(turningMotorCanId, rev::CANSparkLowLevel::MotorType::kBrushless)
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

  m_turningEncoder.SetPositionConversionFactor(2.0 * std::numbers::pi); //<! Converts from wheel rotations to radians
  double initPosition = VoltageToRadians(m_absEnc.GetVoltage());
  m_turningEncoder.SetPosition(initPosition);

  ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitConfigs;
  currentLimitConfigs.WithStatorCurrentLimit(60);
  currentLimitConfigs.WithStatorCurrentLimitEnable(true);
  currentLimitConfigs.WithSupplyCurrentLimit(60);
  currentLimitConfigs.WithSupplyCurrentLimitEnable(true);
  currentLimitConfigs.WithSupplyCurrentThreshold(70);
  currentLimitConfigs.WithSupplyTimeThreshold(0.85);
//  m_driveMotor.SetInverted(driveMotorReversed);
  // m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
  m_driveMotor.SetPosition(units::angle::turn_t(0.0));
  m_driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

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

  m_driveMotor.GetConfigurator().Apply(currentLimitConfigs);
  m_driveMotor.GetConfigurator().Apply(slot0Configs);
  m_driveMotor.GetConfigurator().Apply(motorOutputConfigs);

  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.SetOutputRange(-1.0, 1.0); // -1 to 1 means full power
  constexpr double kTurnP = 0.03;
  //constexpr double kTurnI = 0.000001;
  constexpr double kTurnI = 0.0;
  constexpr double kTurnD = 0.0;
  m_turnP = kTurnP;
  m_turnI = kTurnI;
  m_turnD = kTurnD;
  frc::SmartDashboard::PutNumber("SwrvP", m_turnP);
  frc::SmartDashboard::PutNumber("SwrvI", m_turnI);
  frc::SmartDashboard::PutNumber("SwrvD", m_turnD);
  m_turningPIDController.SetP(m_turnP);
  m_turningPIDController.SetI(m_turnI);
  m_turningPIDController.SetD(m_turnD);
  // frc::SmartDashboard::PutBoolean("Load Turn PID", false);
  // frc::SmartDashboard::PutNumber("Turn P", kTurnP);
  // frc::SmartDashboard::PutNumber("Turn I", kTurnI);
  // frc::SmartDashboard::PutNumber("Turn D", kTurnD);
  m_turningMotor.SetSmartCurrentLimit(20);
  // m_turningMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
  m_turningMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
  
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
}

void SwerveModule::Periodic()
{
  // auto time = m_timer.Get();
  // if (time < 2.0_s)
  // {
  //   ResyncAbsRelEnc();
  // }

  static int count = 0;
  // if (count++ % 50)
  // {
  //   ResyncAbsRelEnc();
  // }
  double turnP = frc::SmartDashboard::GetNumber("SwrvP", m_turnP);
  double turnI = frc::SmartDashboard::GetNumber("SwrvI", m_turnI);
  double turnD = frc::SmartDashboard::GetNumber("SwrvD", m_turnD);
  if (turnP != m_turnP)
  {
    m_turnP = turnP;
    m_turningPIDController.SetP(m_turnP);
  }
  if (turnI != m_turnI)
  {
    m_turnI = turnI;
    m_turningPIDController.SetI(m_turnI);
  }
  if (turnD != m_turnD)
  {
    m_turnD = turnD;
    m_turningPIDController.SetD(m_turnD);
  }

  // Log relative encoder and absolute encoder positions (radians)
  double absPos = VoltageToRadians(m_absEnc.GetVoltage());
  frc::SmartDashboard::PutNumber("Abs Pos" + m_id, absPos);
  frc::SmartDashboard::PutNumber("Abs Pos Offset" + m_id, m_offset);  
  //auto angle = fmod(1 + m_offset - absPos, 1.0);
  // frc::SmartDashboard::PutNumber("Abs Pos plus Offset" + m_id, angle);
  // frc::SmartDashboard::PutNumber("Offset" + m_id, m_offset);
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, GetTurnPosition().to<double>());
  // frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, -1.0 * m_turningEncoder.GetPosition() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));

  m_logTurningEncoderPosition.Append(GetTurnPosition().to<double>());
  //m_logAbsoluteEncoderPosition.Append(absPos * 2 * std::numbers::pi);
  m_logAbsoluteEncoderPosition.Append(((1 + m_offset - absPos) * 2 * std::numbers::pi) - (GetTurnPosition().to<double>()));
  m_logAbsoluteEncoderPositionWithOffset.Append((1 + m_offset - absPos) * 2 * std::numbers::pi);

  // bool bLoadPID = frc::SmartDashboard::GetBoolean("Load Turn PID", false);
  // if (bLoadPID)
  // {
  //   double kTurnP = frc::SmartDashboard::GetNumber("Turn P", 0.5);
  //   double kTurnI = frc::SmartDashboard::GetNumber("Turn I", 0.00001);
  //   double kTurnD = frc::SmartDashboard::GetNumber("Turn D", 0.05);
  //   m_turningPIDController.SetP(kTurnP);
  //   m_turningPIDController.SetI(kTurnI);
  //   m_turningPIDController.SetD(kTurnD);
  //   frc::SmartDashboard::PutNumber("Turn P echo", kTurnP);
  //   frc::SmartDashboard::PutNumber("Turn I echo", kTurnI);
  //   frc::SmartDashboard::PutNumber("Turn D echo", kTurnD);
  //   printf("Loaded Turn PID values P %.3f I %.3f D %.3f\n", kTurnP, kTurnI, kTurnD);
  // }
}

void SwerveModule::ResyncAbsRelEnc()
{
  auto angleInRad = VoltageToRadians(m_absEnc.GetVoltage());         // Returns rotations between 0 and 1
  // if (angleInRad > std::numbers::pi)                    // If angle is between pi and 2pi, put it between -pi and pi
  //     angleInRad -= 2 * std::numbers::pi;

  m_turningEncoder.SetPosition(angleInRad);
//#define PRINT_ABS_RESYNC
#ifdef PRINT_ABS_RESYNC
  auto time = m_timer.Get();
  printf("Module %s %.3f AbsPos %.3f offset %.3f Set abs enc %.3f [rot] %.3f [rad] to rel enc %.3f [rad] mot pos %.3f [rot]\n"
        , m_id.c_str()
        , time.to<double>()
        , absPos
        , m_offset
        , angleInRot
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
  return units::radian_t{ m_turningEncoder.GetPosition() / -c_turnGearRatio};
       //+ units::radian_t{frc::SmartDashboard::GetNumber("Offset" + m_id, 0.0) };
}

units::meters_per_second_t SwerveModule::CalcMetersPerSec()
{
  return kWheelCircumfMeters * m_driveMotor.GetVelocity().GetValue() / units::angle::turn_t(1.0);
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
  return {CalcMeters(), GetTurnPosition() };
}

units::meter_t SwerveModule::CalcMeters()
{
  return kWheelCircumfMeters * m_driveMotor.GetPosition().GetValue() / units::angle::turn_t(1.0);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  double currPosition = GetTurnPosition().to<double>();
  const auto state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d{ units::radian_t(currPosition) });
  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, currPosition);

  if (state.speed != 0_mps)
  {
#ifdef DISABLE_DRIVE
    m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
#else
    // TODO Change how the speed to the drive motor is calculated
    // m_driveMotor.Set(TalonFXControlMode::Velocity, CalcTicksPer100Ms(state.speed));
    // if (m_id == "1")
    // {
    //   m_driveMotor.Set(1.0);
    // }
    // else
    {
      auto spd = (state.speed / m_currentMaxSpeed).to<double>();
      spd *= m_driveMotorReversed ? -1.0 : 1.0;
      //frc::SmartDashboard::PutNumber("Drive Speed" + m_id, spd);
      m_driveMotor.Set(spd);
      //m_driveMotor.Set((state.speed / m_currentMaxSpeed).to<double>());
    }
#endif
  }
  else
  {
    m_driveMotor.Set(0.0);
  }

  // Calculate the turning motor output from the turning PID controller.
  //frc::SmartDashboard::PutNumber("Turn Ref Opt" + m_id, state.angle.Radians().to<double>());
  //frc::SmartDashboard::PutNumber("Turn Ref" + m_id, referenceState.angle.Radians().to<double>());
  double newRef = -c_turnGearRatio * state.angle.Radians().to<double>();
  // double newRef = 25.0 * state.angle.Radians().to<double>();
  // newRef = frc::SmartDashboard::GetNumber("NewRef", 0.0);

  m_logTurningRefSpeed.Append(referenceState.speed.to<double>());
  m_logTurningRefAngle.Append(referenceState.angle.Degrees().to<double>());
  m_logTurningNewAngle.Append(state.angle.Degrees().to<double>());
  m_logDriveNewSpeed.Append(state.speed.to<double>());
  m_logDriveNormalizedSpeed.Append((state.speed / m_currentMaxSpeed).to<double>());

  frc::SmartDashboard::PutNumber("Turn Ref Motor" + m_id, newRef);
  // m_turningPIDController.SetReference(-1.0 * newRef, CANSparkBase::ControlType::kPosition);
  m_turningPIDController.SetReference(newRef, CANSparkBase::ControlType::kPosition);
}

double SwerveModule::VoltageToRadians(double Voltage)
{
    frc::SmartDashboard::PutNumber("Voltage" + m_id, Voltage);
    double angle = Voltage * DriveConstants::kTurnVoltageToRadians;
    angle -= m_offset;
    angle = fmod(angle + 2 * std::numbers::pi, 2 * std::numbers::pi);
// #ifndef ZERO_OFFSETS
//     // angle ranges from 0 to 2pi
//     // This reverses it from 2pi to 0 
//     angle = 2 * std::numbers::pi - angle;
// #endif

    return angle;
}