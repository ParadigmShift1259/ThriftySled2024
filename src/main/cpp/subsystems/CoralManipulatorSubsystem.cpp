
#include "ConstantsDigitalInputs.h"
#include "subsystems/CoralManipulatorSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace rev::spark;

constexpr double c_defaultIntakeP = 0.03;
constexpr double c_defaultIntakeExtendP = 0.03;
constexpr double c_defaultIntakeI = 0.0;
constexpr double c_defaultIntakeD = 0.0;

constexpr ClosedLoopSlot c_intakeGeneralPIDSlot = ClosedLoopSlot::kSlot0;
constexpr ClosedLoopSlot c_intakeExtendPIDSlot = ClosedLoopSlot::kSlot1;

CoralManipulatorSubsystem::CoralManipulatorSubsystem() 
    : m_motor(kCoralManipRollerCANID)
    , m_photoEye(kCoralManipPhotoeye)
    , m_deployMotor(kCoralManipDeployCANID, SparkLowLevel::MotorType::kBrushless)
{
    m_motor.SetNeutralMode(NeutralMode::Coast);

    SparkBaseConfig config{};
    config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    config.ClosedLoopRampRate(0.0);
    config.closedLoop.OutputRange(kMinOut, kMaxOut);

    m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    m_deployRelativeEnc.SetPosition(0.0);

    frc::Preferences::InitDouble("kCoralManipDeployP", c_defaultIntakeP);
    frc::Preferences::InitDouble("kCoralManipDeployExtendP", c_defaultIntakeExtendP);
    frc::Preferences::InitDouble("kCoralManipDeployI", c_defaultIntakeI);
    frc::Preferences::InitDouble("kCoralManipDeployD", c_defaultIntakeD);
}

void CoralManipulatorSubsystem::Periodic()
{
    LoadDeployPid();

    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("CoralManipPhotoEye", m_photoEye.Get());
}

void CoralManipulatorSubsystem::LoadDeployPid()
{
    static double lastP = 0.0;
    static double lastExtendP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;

    auto p = frc::Preferences::GetDouble("kCoralManipDeployP", c_defaultIntakeP);
    auto pExtend = frc::Preferences::GetDouble("kCoralManipDeployExtendP", c_defaultIntakeExtendP);
    auto i = frc::Preferences::GetDouble("kCoralManipDeployI", c_defaultIntakeI);
    auto d = frc::Preferences::GetDouble("kCoralManipDeployD", c_defaultIntakeD);
    
    if (p != lastP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(p, c_intakeGeneralPIDSlot);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (pExtend != lastExtendP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(pExtend, c_intakeExtendPIDSlot);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (i != lastI)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .I(i, c_intakeExtendPIDSlot)
            .I(i, c_intakeGeneralPIDSlot);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (d != lastD)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .D(d, c_intakeExtendPIDSlot)
            .D(d, c_intakeGeneralPIDSlot);
        m_deployMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    lastP = p;
    lastExtendP = pExtend;
    lastI = i;
    lastD = d;

}

void CoralManipulatorSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}
