#include "Constants.h"
#include "subsystems/ClimberSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/Preferences.h>

constexpr ClosedLoopSlot c_defaultClimbDownPIDSlot = rev::spark::ClosedLoopSlot::kSlot0;
constexpr ClosedLoopSlot c_defaultClimbUpPIDSlot = rev::spark::ClosedLoopSlot::kSlot1;

constexpr double c_defaultClimbDownP = 0.006;
constexpr double c_defaultClimbUpP = 0.1;
constexpr double c_defaultClimbI = 0.0;
constexpr double c_defaultClimbD = 0.0;
constexpr double c_defaultClimbFF = 0.00000;

ClimberSubsystem::ClimberSubsystem()
    : m_motor(kClimbMotorCANID, rev::spark::SparkLowLevel::MotorType::kBrushless)
{
    SparkBaseConfig config{};
    config
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(false);
    config.ClosedLoopRampRate(0.0);
    config.closedLoop.OutputRange(kMinOut, kMaxOut);

    m_motor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    m_relativeEnc.SetPosition(0.0);

    frc::SmartDashboard::PutNumber("ClimbPosEcho", 1.0);
    frc::SmartDashboard::PutNumber("ClimbRel", c_defaultClimbDeployRelTurns);

    frc::Preferences::InitDouble("kClimbPosDownP", c_defaultClimbDownP);
    frc::Preferences::InitDouble("kClimbPosUpP", c_defaultClimbUpP);
    frc::Preferences::InitDouble("kClimbPosI", c_defaultClimbI);
    frc::Preferences::InitDouble("kClimbPosD", c_defaultClimbD);
    frc::Preferences::InitDouble("kClimbPosFF", c_defaultClimbFF);
}

void ClimberSubsystem::Periodic()
{
  static int count = 0;
  if (count++ % 20 == 0)
  {
    static double lastDownP = 0.0;
    static double lastUpP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;
    static double lastFF = 0.0;

    auto pDown = frc::Preferences::GetDouble("kClimbPosDownP", c_defaultClimbDownP); //originally .07
    auto pUp = frc::Preferences::GetDouble("kClimbPosUpP", c_defaultClimbUpP);
    auto i = frc::Preferences::GetDouble("kClimbPosI", c_defaultClimbI);
    auto d = frc::Preferences::GetDouble("kClimbPosD", c_defaultClimbD);
    auto ff = frc::Preferences::GetDouble("kClimbPosFF", c_defaultClimbFF);
    if (pDown != lastDownP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(pDown, c_defaultClimbDownPIDSlot);
        m_motor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (pUp != lastUpP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(pUp, c_defaultClimbUpPIDSlot);
        m_motor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (i != lastI)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .I(i, c_defaultClimbDownPIDSlot)
            .I(i, c_defaultClimbUpPIDSlot);
        m_motor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (d != lastD)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .D(d, c_defaultClimbDownPIDSlot)
            .D(d, c_defaultClimbUpPIDSlot);
        m_motor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (ff != lastFF)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .VelocityFF(ff, c_defaultClimbDownPIDSlot)
            .VelocityFF(ff, c_defaultClimbUpPIDSlot);
        m_motor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    lastDownP = pDown;
    lastUpP = pUp;
    lastI = i;
    lastD = d;
    lastFF = ff;

    frc::SmartDashboard::PutNumber("ClimbPosEcho", m_relativeEnc.GetPosition());
  }
}

void ClimberSubsystem::GoToPositionRel(double relPos)
{
    relPos = frc::SmartDashboard::GetNumber("ClimbRel", c_defaultClimbDeployRelTurns);
    GoToPosition(m_relativeEnc.GetPosition() + relPos);
}

void ClimberSubsystem::GoToPosition(double position)
{
    ClosedLoopSlot slot = c_defaultClimbUpPIDSlot;
    m_closedLoopController.SetReference(position, SparkBase::ControlType::kPosition, slot);
}

void ClimberSubsystem::Set(double speed) 
{
    m_motor.Set(speed);
}

double ClimberSubsystem::GetPosition() 
{
    return m_relativeEnc.GetPosition();
}