#include "ConstantsDigitalInputs.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultIntakeP = 0.03;
constexpr double c_defaultIntakeI = 0.0;
constexpr double c_defaultIntakeD = 0.0;

constexpr ClosedLoopSlot c_intakeGeneralPIDSlot = ClosedLoopSlot::kSlot0;

IntakeSubsystem::IntakeSubsystem() 
    // : m_motor(kIntakeRollerCANID)
    : m_deployMotor(kIntakeChuteCANID, SparkLowLevel::MotorType::kBrushless)
{
    // m_motor.SetNeutralMode(NeutralMode::Coast);

    SparkBaseConfig config{};
    config
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(false);
    config.ClosedLoopRampRate(0.0);
    config.closedLoop.OutputRange(kMinOut, kMaxOut);

    m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    
    m_deployRelativeEnc.SetPosition(0.0);

    frc::Preferences::InitDouble("kIntakeDeployP", c_defaultIntakeP);
    frc::Preferences::InitDouble("kIntakeDeployI", c_defaultIntakeI);
    frc::Preferences::InitDouble("kIntakeDeployD", c_defaultIntakeD);

    frc::SmartDashboard::PutNumber("IntakePosTurns", c_LoadCoralPosition);
    frc::SmartDashboard::PutNumber("IntakeParkTurns", c_ParkForClimbPosition);
}

void IntakeSubsystem::Periodic()
{
    LoadDeployPid();
    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
}

void IntakeSubsystem::LoadDeployPid()
{
    static double lastP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;

    auto p = frc::Preferences::GetDouble("kIntakeDeployP", c_defaultIntakeP);
    auto i = frc::Preferences::GetDouble("kIntakeDeployI", c_defaultIntakeI);
    auto d = frc::Preferences::GetDouble("kIntakeDeployD", c_defaultIntakeD);

    if (p != lastP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(p, c_intakeGeneralPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (i != lastI)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .I(i, c_intakeGeneralPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (d != lastD)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .D(d, c_intakeGeneralPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    lastP = p;
    lastI = i;
    lastD = d;
}

void IntakeSubsystem::Set(double speed)
{
    // m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::AlignIntake()
{
    double turns = frc::SmartDashboard::GetNumber("IntakePosTurns", c_LoadCoralPosition);
    //printf("dep extend turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
}

void IntakeSubsystem::ParkIntakeForClimb()
{
    double turns = frc::SmartDashboard::GetNumber("IntakeParkTurns", c_ParkForClimbPosition);
    //printf("dep retract turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
}

void IntakeSubsystem::GoToPosition(double turns)
{
    m_deployPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
}