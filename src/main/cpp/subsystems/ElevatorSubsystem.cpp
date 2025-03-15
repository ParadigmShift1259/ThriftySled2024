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
constexpr double c_defaultElevatorI = 0.00003;
constexpr double c_defaultElevatorD = 0.0;
constexpr double c_defaultElevatorFF = 0.00000;

// constexpr double c_maxVelocity = 4000.0;        // RPM
// constexpr double c_maxAcceleration = 8000.0;   // RPM/s
// constexpr double c_allowedError = 1.0;          // Turns

ElevatorSubsystem::ElevatorSubsystem()
    : m_leadMotor(kElevatorLeadMotorCANID, SparkLowLevel::MotorType::kBrushless)
    , m_followMotor(kElevatorFollowMotorCANID, SparkLowLevel::MotorType::kBrushless)
{
    m_leadConfig
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(true);
    m_leadConfig.ClosedLoopRampRate(0.0);
    m_leadConfig.closedLoop.OutputRange(kMinOut, kMaxOut);
    // m_leadConfig.closedLoop.maxMotion.MaxVelocity(c_maxVelocity);
    // m_leadConfig.closedLoop.maxMotion.MaxAcceleration(c_maxAcceleration);
    // m_leadConfig.closedLoop.maxMotion.AllowedClosedLoopError(c_allowedError);
    m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    m_followConfig
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(false);
    m_followConfig.ClosedLoopRampRate(0.0);
    m_followConfig.closedLoop.OutputRange(kMinOut, kMaxOut);
    // m_followConfig.closedLoop.maxMotion.MaxVelocity(c_maxVelocity);
    // m_followConfig.closedLoop.maxMotion.MaxAcceleration(c_maxAcceleration);
    // m_followConfig.closedLoop.maxMotion.AllowedClosedLoopError(c_allowedError);
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
    frc::Preferences::InitDouble("ElevatorL4", c_defaultL4Turns);

    frc::Preferences::InitDouble("ElevatorPosRR", 0.0);

    // frc::Preferences::InitDouble("ElevatorL3", c_defaultL3Turns);
    // frc::Preferences::InitDouble("ElevatorL2", c_defaultL2Turns);
    // frc::Preferences::InitDouble("ElevatorL1", c_defaultL1Turns);

    frc::SmartDashboard::PutNumber("ElevatorLeadMotorPos", 1.0);
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorPos", 1.0);

    frc::SmartDashboard::PutNumber("ElevatorGoToRel", 1.0);

    frc::SmartDashboard::PutBoolean("L1", false);
    frc::SmartDashboard::PutBoolean("L2", false);
    frc::SmartDashboard::PutBoolean("L3", false);
    frc::SmartDashboard::PutBoolean("L4", false);
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
    //static double lastRampRate = 0.0;

    auto pDown = frc::Preferences::GetDouble("ElevatorPosDownP", c_defaultElevatorDownP);
    auto pUp = frc::Preferences::GetDouble("ElevatorPosUpP", c_defaultElevatorUpP);
    auto i = frc::Preferences::GetDouble("ElevatorPosI", c_defaultElevatorI);
    auto d = frc::Preferences::GetDouble("ElevatorPosD", c_defaultElevatorD);
    auto ff = frc::Preferences::GetDouble("ElevatorPosFF", c_defaultElevatorFF);

    //auto rampRate = frc::Preferences::GetDouble("ElevatorPosRR", 0.0);

    // if (rampRate != lastRampRate)
    // {
    //     m_leadConfig.ClosedLoopRampRate(rampRate);
    //     m_followConfig.ClosedLoopRampRate(rampRate);;
    //     m_leadMotor.Configure(m_leadConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    //     m_followMotor.Configure(m_followConfig, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    // }

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
        m_leadConfig.closedLoop.I(i, c_defaultElevatorDownPIDSlot); // Only apply I to down motion .I(i, c_defaultElevatorUpPIDSlot);
        m_followConfig.closedLoop.I(i, c_defaultElevatorDownPIDSlot); // Only apply I to down motion .I(i, c_defaultElevatorUpPIDSlot);
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
    //lastRampRate = rampRate;

    //m_dbvVelocity.Put(m_leadRelativeEnc.GetVelocity());

    double currentPos = m_leadRelativeEnc.GetPosition();
    bool bGoingToBottom = currentPos < 10.0 && m_position < 1.0;
    bool bGoingToTop = currentPos > 30.0 && m_position > 39.0;
    if (( bGoingToBottom || bGoingToTop ) && m_slot == c_defaultElevatorUpPIDSlot)
    {
       m_slot = c_defaultElevatorDownPIDSlot;
       m_leadPIDController.SetReference(m_position, SparkBase::ControlType::kPosition, m_slot);
       m_followPIDController.SetReference(m_position, SparkBase::ControlType::kPosition, m_slot);
   }

    // if (fabs(m_position - currentPos) < 0.25) 
    // {
    //     Stop();
    // }

    frc::SmartDashboard::PutNumber("ElevatorLeadMotorPos Echo", m_leadRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorPos Echo", m_followRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("ElevatorUpperLimit", m_upperLimit.Get());
    frc::SmartDashboard::PutBoolean("ElevatorLowerLimit", m_lowerLimit.Get());
  }
}

double ElevatorSubsystem::GetPositionForLevel(ELevels eLevel)
{
    static double positions[] = { c_defaultL1Turns, c_defaultL2Turns, c_defaultL3Turns, c_defaultL4Turns, c_algaeRemovalL3_4, c_algaeRemovalL2_3 };
    double position;
    if (eLevel == L4)
    {
        position = frc::Preferences::GetDouble("ElevatorL4", c_defaultL4Turns);
    }
    else
    {
        position = positions[eLevel]; 
    }
     return position;
}

void ElevatorSubsystem::GoToPosition(ELevels eLevel)
{
    //if (eLevel != algaeRemovalL3_4)
    {
        SetPresetLevel(eLevel);
    }
    GoToPosition(GetPositionForLevel(eLevel));  
}

void ElevatorSubsystem::GoToPosition(double position)
{
    m_position = std::clamp(position, 0.0, c_defaultL4Turns);
    
    frc::SmartDashboard::PutNumber("ElevatorLeadMotorPos", position);
    frc::SmartDashboard::PutNumber("ElevatorFollowMotorPos", position);

    m_slot = c_defaultElevatorUpPIDSlot;
    if (m_position < 1.5)
    {
        m_slot = c_defaultElevatorDownPIDSlot;
    }
    m_leadPIDController.SetReference(position, SparkBase::ControlType::kPosition, m_slot);
    m_followPIDController.SetReference(position, SparkBase::ControlType::kPosition, m_slot);
    // m_leadPIDController.SetReference(m_position, SparkBase::ControlType::kMAXMotionPositionControl, m_slot);
    // m_followPIDController.SetReference(m_position, SparkBase::ControlType::kMAXMotionPositionControl, m_slot);
}

void ElevatorSubsystem::GotoPositionRel(double relPos)
{
    bool bDown = relPos < 0.0;
    relPos = frc::SmartDashboard::GetNumber("ElevatorGoToRel", 1.0);
    relPos = std::clamp(relPos, -10.0, 10.0);
    if(bDown)
    {
        relPos *= -1.0;
    }
    // printf("GotoPositionRel enc %.3f relPos %.3f newPos %.3f\n", m_leadRelativeEnc.GetPosition(), relPos, m_leadRelativeEnc.GetPosition() + relPos);
    GoToPosition(m_leadRelativeEnc.GetPosition() + relPos);
}

bool ElevatorSubsystem::IsAtPosition(ELevels level)
{
    double levelPos = GetPositionForLevel(level);
    double difference = fabs(m_leadRelativeEnc.GetPosition() - levelPos);
    // return difference > -0.53 && difference < 0.5;
    return difference > -6.0 && difference < 6.0;
}

void ElevatorSubsystem::SetPresetLevel(ELevels level)
{
    m_level = level;
    frc::SmartDashboard::PutBoolean("L1", (m_level == L1));
    frc::SmartDashboard::PutBoolean("L2", (m_level == L2));
    frc::SmartDashboard::PutBoolean("L3", (m_level == L3));
    frc::SmartDashboard::PutBoolean("L4", (m_level == L4));
}

void ElevatorSubsystem::GoToPresetLevel()
{
    GoToPosition(m_level);
}