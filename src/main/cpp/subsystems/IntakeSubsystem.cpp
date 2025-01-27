
#include "ConstantsDigitalInputs.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultIntakeP = 0.03;
constexpr double c_defaultIntakeExtendP = 0.03;
constexpr double c_defaultIntakeI = 0.0;
constexpr double c_defaultIntakeD = 0.0;

constexpr ClosedLoopSlot c_intakeGeneralPIDSlot = ClosedLoopSlot::kSlot0;
constexpr ClosedLoopSlot c_intakeExtendPIDSlot = ClosedLoopSlot::kSlot1;

constexpr double c_defaultIntakeMin = -0.5;
constexpr double c_defaultIntakeMax = 0.5;


IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeRollerCANID)
    , m_photoEye(kIntakePhotoeye)
    , m_deployMotor(kIntakeDeployCANID, SparkLowLevel::MotorType::kBrushless)
    , m_deployFollowMotor(kIntakeDeployFollowCANID, SparkLowLevel::MotorType::kBrushless)
{
    m_motor.SetNeutralMode(NeutralMode::Coast);

    SparkBaseConfig config{};
    config
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(false);
    config.ClosedLoopRampRate(0.0);
    config.closedLoop.OutputRange(kMinOut, kMaxOut);

    m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    
    config.Inverted(true);
    m_deployFollowMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    m_deployRelativeEnc.SetPosition(0.0);
    m_deployFollowRelativeEnc.SetPosition(0.0);

    frc::Preferences::InitDouble("kIntakeDeployP", c_defaultIntakeP);
    frc::Preferences::InitDouble("kIntakeDeployExtendP", c_defaultIntakeExtendP);
    frc::Preferences::InitDouble("kIntakeDeployI", c_defaultIntakeI);
    frc::Preferences::InitDouble("kIntakeDeployD", c_defaultIntakeD);

    frc::Preferences::InitDouble("kIntakeDeployMin", c_defaultIntakeMin);
    frc::Preferences::InitDouble("kIntakeDeployMax", c_defaultIntakeMax);

    frc::SmartDashboard::PutNumber("DepRtctTurns", c_defaultRetractTurns);
    frc::SmartDashboard::PutNumber("DepExtTurns", c_defaultExtendTurns);
    frc::SmartDashboard::PutNumber("DepOffsetTurns", c_defaultOffsetTurns);
}

void IntakeSubsystem::Periodic()
{
    LoadDeployPid();

    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("Deploy Follow echo", m_deployFollowRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("Intake PhotoEye", m_photoEye.Get());
}

void IntakeSubsystem::LoadDeployPid()
{
    static double lastP = 0.0;
    static double lastExtendP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;

    auto p = frc::Preferences::GetDouble("kIntakeDeployP", c_defaultIntakeP);
    auto pExtend = frc::Preferences::GetDouble("kIntakeDeployExtendP", c_defaultIntakeExtendP);
    auto i = frc::Preferences::GetDouble("kIntakeDeployI", c_defaultIntakeI);
    auto d = frc::Preferences::GetDouble("kIntakeDeployD", c_defaultIntakeD);

    if (p != lastP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(p, c_intakeGeneralPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        config.Inverted(true);
        m_deployFollowMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (pExtend != lastExtendP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(pExtend, c_intakeExtendPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        config.Inverted(true);
        m_deployFollowMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (i != lastI)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .I(i, c_intakeExtendPIDSlot)
            .I(i, c_intakeGeneralPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        config.Inverted(true);
        m_deployFollowMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (d != lastD)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .D(d, c_intakeExtendPIDSlot)
            .D(d, c_intakeGeneralPIDSlot);
        config.Inverted(false);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
        config.Inverted(true);
        m_deployFollowMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    lastP = p;
    lastExtendP = pExtend;
    lastI = i;
    lastD = d;

}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{
    double turns = frc::SmartDashboard::GetNumber("DepExtTurns", c_defaultExtendTurns);
    double offsetTurns = frc::SmartDashboard::GetNumber("DepOffsetTurns", c_defaultOffsetTurns);
    //printf("dep extend turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns + offsetTurns, SparkBase::ControlType::kPosition, c_intakeExtendPIDSlot);

    m_deployFollowPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeExtendPIDSlot);
    // frc::SmartDashboard::PutNumber("DepApplOut", m_deployMotor.GetAppliedOutput()); 
    // frc::SmartDashboard::PutNumber("DepBusV", m_deployMotor.GetBusVoltage());
    // frc::SmartDashboard::PutNumber("DepTemp", m_deployMotor.GetMotorTemperature());
    // frc::SmartDashboard::PutNumber("DepOutAmps", m_deployMotor.GetOutputCurrent());    
}

void IntakeSubsystem::ExtendIntake(double turns)
{
    double offsetTurns = frc::SmartDashboard::GetNumber("DepOffsetTurns", c_defaultOffsetTurns);
    m_deployPIDController.SetReference(turns + offsetTurns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
    m_deployFollowPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
}


void IntakeSubsystem::RetractIntake()
{
    double turns = frc::SmartDashboard::GetNumber("DepRtctTurns", c_defaultRetractTurns);
    //printf("dep retract turns %.3f\n", turns);
    double offsetTurns = frc::SmartDashboard::GetNumber("DepOffsetTurns", c_defaultOffsetTurns);
    m_deployPIDController.SetReference(turns + offsetTurns, SparkBase::ControlType::kPosition, c_intakeExtendPIDSlot);
    m_deployFollowPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeExtendPIDSlot);
}

void IntakeSubsystem::GoToPosition(double turns)
{
    double offsetTurns = frc::SmartDashboard::GetNumber("DepOffsetTurns", c_defaultOffsetTurns);
    m_deployPIDController.SetReference(turns + offsetTurns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
    m_deployFollowPIDController.SetReference(turns, SparkBase::ControlType::kPosition, c_intakeGeneralPIDSlot);
}
