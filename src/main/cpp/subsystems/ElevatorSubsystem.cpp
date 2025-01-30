#include "Constants.h"

#include "subsystems/ElevatorSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

double c_defaultLeadDirection;
double c_defaultFollowDirection;

constexpr ClosedLoopSlot c_defaultElevatorDownPIDSlot = ClosedLoopSlot::kSlot0;
constexpr ClosedLoopSlot c_defaultElevatorUpPIDSlot = ClosedLoopSlot::kSlot1;

constexpr double c_defaultElevatorDownP = 0.01;
constexpr double c_defaultElevatorUpP = 0.1;
constexpr double c_defaultElevatorI = 0.00005;
constexpr double c_defaultElevatorD = 0.0;
constexpr double c_defaultElevatorFF = 0.00000;

ElevatorSubsystem::ElevatorSubsystem()
    : m_leadMotor(kElevatorLeadMotorCANID, SparkLowLevel::MotorType::kBrushless)
    , m_followMotor(kElevatorFollowMotorCANID, SparkLowLevel::MotorType::kBrushless)
{
    m_leadConfig
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(true);
    m_leadConfig.ClosedLoopRampRate(0.0);
    m_leadConfig.closedLoop.OutputRange(kMinOut, kMaxOut);
    m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    m_followConfig
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(false);
    m_followConfig.ClosedLoopRampRate(0.0);
    m_followConfig.closedLoop.OutputRange(kMinOut, kMaxOut);
    m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    m_leadRelativeEnc.SetPosition(0.0);
    m_followRelativeEnc.SetPosition(0.0);

    c_defaultLeadDirection = m_leadDirection;
    c_defaultFollowDirection = m_followDirection;

    //m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10.0, 1.0);
    //m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10.0, 1.0);

    // m_motor.ConfigNeutralDeadband(0);
    // m_motor.ConfigNominalOutputForward(kNominal);
    // m_motor.ConfigNominalOutputReverse(kNominal * -1.0);
    // m_motor.ConfigPeakOutputForward(kMaxOut);
    // m_motor.ConfigPeakOutputReverse(kMaxOut * -1.0);
    frc::SmartDashboard::PutNumber("ElevatorLeadMotorDirection", c_defaultLeadDirection);
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorDirection", c_defaultFollowDirection);

    //frc::SmartDashboard::PutNumber("ElevatorHiTurns", c_defaultHighTurns);
    frc::SmartDashboard::PutNumber("ElevatorParkTurns", c_defaultParkTurns);
    frc::SmartDashboard::PutNumber("ElevatorResetTurns", c_defaultResetTurns);

    frc::Preferences::InitDouble("ElevatorPosDownP", c_defaultElevatorDownP);
    frc::Preferences::InitDouble("ElevatorPosUpP", c_defaultElevatorUpP);
    frc::Preferences::InitDouble("ElevatorPosI", c_defaultElevatorI);
    frc::Preferences::InitDouble("ElevatorPosD", c_defaultElevatorD);
    frc::Preferences::InitDouble("ElevatorPosFF", c_defaultElevatorFF);

    frc::SmartDashboard::PutNumber("ElevatorLeadMotorPos", 1.0);
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorPos", 1.0);

    frc::SmartDashboard::PutNumber("ElevatorGoToRel", 0.0);
}

void ElevatorSubsystem::Periodic()
{
  static int count = 0;
  if (count++ % 20 == 0)
  {
    static double lastDownP = 0.0;
    static double lastUpP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;
    static double lastFF = 0.0;

    auto pDown = frc::Preferences::GetDouble("ElevatorPosDownP", c_defaultElevatorDownP);
    auto pUp = frc::Preferences::GetDouble("ElevatorPosUpP", c_defaultElevatorUpP);
    auto i = frc::Preferences::GetDouble("ElevatorPosI", c_defaultElevatorI);
    auto d = frc::Preferences::GetDouble("ElevatorPosD", c_defaultElevatorD);
    auto ff = frc::Preferences::GetDouble("ElevatorPosFF", c_defaultElevatorFF);

    if (pDown != lastDownP)
    {
        m_leadConfig.closedLoop.P(pDown, c_defaultElevatorDownPIDSlot);
        m_followConfig.closedLoop.P(pDown, c_defaultElevatorDownPIDSlot);
        m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (pUp != lastUpP)
    {
        m_leadConfig.closedLoop.P(pUp, c_defaultElevatorUpPIDSlot);
        m_followConfig.closedLoop.P(pUp, c_defaultElevatorUpPIDSlot);
        m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (i != lastI)
    {
        m_leadConfig.closedLoop.I(i, c_defaultElevatorDownPIDSlot).I(i, c_defaultElevatorUpPIDSlot);
        m_followConfig.closedLoop.I(i, c_defaultElevatorDownPIDSlot).I(i, c_defaultElevatorUpPIDSlot);
        m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (d != lastD)
    {
        m_leadConfig.closedLoop.D(d, c_defaultElevatorDownPIDSlot).D(d, c_defaultElevatorUpPIDSlot);
        m_followConfig.closedLoop.D(d, c_defaultElevatorDownPIDSlot).D(d, c_defaultElevatorUpPIDSlot);
        m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (ff != lastFF)
    {
        m_leadConfig.closedLoop.VelocityFF(ff, c_defaultElevatorDownPIDSlot).VelocityFF(ff, c_defaultElevatorUpPIDSlot);
        m_followConfig.closedLoop.VelocityFF(ff, c_defaultElevatorDownPIDSlot).VelocityFF(ff, c_defaultElevatorUpPIDSlot);
        m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    lastDownP = pDown;
    lastUpP = pUp;
    lastI = i;
    lastD = d;
    lastFF = ff;

    double currentPos = m_leadRelativeEnc.GetPosition();
    bool bGoingToBottom = currentPos < 10.0 && m_position < 1.0;
    bool bGoingToTop = currentPos > 30.0 && m_position > 39.0;
    if (( bGoingToBottom || bGoingToTop ) && m_slot == c_defaultElevatorUpPIDSlot)
    {
        m_slot = c_defaultElevatorDownPIDSlot;
        m_leadPIDController.SetReference(m_position, SparkBase::ControlType::kPosition, m_slot);
        m_followPIDController.SetReference(m_position, SparkBase::ControlType::kPosition, m_slot);
    }

    if (fabs(m_position - currentPos) < 0.25) 
    {
        Stop();
    }

    frc::SmartDashboard::PutNumber("ElevatorLeadMotorPos Echo", m_leadRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorPos Echo", m_followRelativeEnc.GetPosition());
  }
}

void ElevatorSubsystem::GoToPosition(double position)
{
    std::clamp(position, 0.0, 40.0);
    m_position = position;
    
    frc::SmartDashboard::PutNumber("ElevatorLeadMotorPos", position);
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorPos", position);

    m_slot = c_defaultElevatorUpPIDSlot;
    if(position < 1.5)
    {
        m_slot = c_defaultElevatorDownPIDSlot;
    }
    m_leadPIDController.SetReference(position, SparkBase::ControlType::kPosition, m_slot);
    m_followPIDController.SetReference(position, SparkBase::ControlType::kPosition, m_slot);

}

void ElevatorSubsystem::GotoPositionRel(double relPos)
{
    bool bDown = relPos < 0.0;
    relPos = frc::SmartDashboard::GetNumber("ElevatorGoToRel", 0.0);
    std::clamp(relPos, -10.0, 10.0);
    if(bDown)
    {
        relPos *= -1.0;
    }
    // printf("GotoPositionRel enc %.3f relPos %.3f newPos %.3f\n", m_leadRelativeEnc.GetPosition(), relPos, m_leadRelativeEnc.GetPosition() + relPos);
    GoToPosition(m_leadRelativeEnc.GetPosition() + relPos);
}

bool ElevatorSubsystem::IsAtPosition(double level)
{
    double difference = fabs(m_leadRelativeEnc.GetPosition() - level);
    return difference > -0.53 && difference < 0.5;
}

