
#include "ConstantsDigitalInputs.h"
#include "subsystems/CoralManipulatorSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace rev::spark;

constexpr double c_defaultCoralManipP = 0.03;
constexpr double c_defaultCoralManipI = 0.0;
constexpr double c_defaultCoralManipD = 0.0;

constexpr ClosedLoopSlot c_intakeGeneralPIDSlot = ClosedLoopSlot::kSlot0;
constexpr ClosedLoopSlot c_intakeExtendPIDSlot = ClosedLoopSlot::kSlot1;

constexpr double c_defaultIntakeMin = -0.5;
constexpr double c_defaultIntakeMax = 0.5;

CoralManipulatorSubsystem::CoralManipulatorSubsystem() 
    : m_coralMotor(kCoralManipulatorCANID,SparkLowLevel::MotorType::kBrushless)
    , m_photoEyeIn(kCoralManipPhotoeyeIn)
    , m_photoEyeOut(kCoralManipPhotoeyeOut)
    , m_deployServo(kCoralManipDeployChannel)
{
    

    SparkBaseConfig config{};
    config
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    config.ClosedLoopRampRate(0.0);
    config.closedLoop.OutputRange(kDeployMinOut, kDeployMaxOut);

    m_coralMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    m_coralRelativeEnc.SetPosition(0.0);

    frc::Preferences::InitDouble("kCoralManipP", c_defaultCoralManipP);
    frc::Preferences::InitDouble("kCoralManipI", c_defaultCoralManipI);
    frc::Preferences::InitDouble("kCoralManipD", c_defaultCoralManipD);
}

void CoralManipulatorSubsystem::Periodic()
{
    LoadDeployPid();

    frc::SmartDashboard::PutNumber("Coral echo", m_coralRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("CoralManipPhotoEyeIn", m_photoEyeIn.Get());
    frc::SmartDashboard::PutBoolean("CoralManipPhotoEyeOut", m_photoEyeOut.Get());
}

void CoralManipulatorSubsystem::LoadDeployPid()
{
    static double lastP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;

    auto p = frc::Preferences::GetDouble("kCoralManipP", c_defaultCoralManipP);
    auto i = frc::Preferences::GetDouble("kCoralManipI", c_defaultCoralManipI);
    auto d = frc::Preferences::GetDouble("kCoralManipD", c_defaultCoralManipD);
    
    if (p != lastP)
    {
        SparkBaseConfig config{};
        config.closedLoop.P(p, c_intakeGeneralPIDSlot);
        m_coralMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (i != lastI)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .I(i, c_intakeExtendPIDSlot)
            .I(i, c_intakeGeneralPIDSlot);
        m_coralMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    if (d != lastD)
    {
        SparkBaseConfig config{};
        config.closedLoop
            .D(d, c_intakeExtendPIDSlot)
            .D(d, c_intakeGeneralPIDSlot);
        m_coralMotor.Configure(config, SparkBase::ResetMode::kNoResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    }
    lastP = p;
    lastI = i;
    lastD = d;

}

void CoralManipulatorSubsystem::Set(double speed)
{
    //m_motor.Set(ControlMode::PercentOutput, speed);
}
