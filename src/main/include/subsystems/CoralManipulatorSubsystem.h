#pragma once

#include "Constants.h"
#include "ConstantsDigitalInputs.h"
#include "ConstantsCANIDs.h"

#include <frc/DigitalInput.h>
#include <frc/Servo.h>

#include <frc/SmartDashboard/SmartDashboard.h>

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
WPI_UNIGNORE_DEPRECATED

#include <rev/SparkMax.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace rev::spark;

constexpr double c_sevroDeployDefault = 0.85;
constexpr double c_sevroRetractDefault = 0.3;

class CoralManipulatorSubsystem : public frc2::SubsystemBase
{
public:
    CoralManipulatorSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the intake at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void SetFeeder(double speed);
    /// Deploys the manipulator out of the robot
    void DeployManipulator() {double sp = frc::SmartDashboard::GetNumber("ServoDeploy", c_sevroDeployDefault); m_deployServo.Set(sp); }
    // Retracts the manipulator into the robot
    void RetractManipulator() {double sp = frc::SmartDashboard::GetNumber("ServoRetract", c_sevroRetractDefault);m_deployServo.Set(sp); }
    void GoToPosition(double turns);
    double GetPosition() { return m_coralRelativeEnc.GetPosition(); }
    bool IsCoralPresentInput() { return m_photoEyeIn.Get(); }
    bool IsCoralPresentOutput() { return m_photoEyeOut.Get(); }
    void EjectCoral(bool slow) { m_coralMotor.SetVoltage(slow ? -5.0_V : -9.0_V); }
    void SetManipulator(double speed);
    void RetractCoral(ELevels eLevel);

    void Stop() { m_coralMotor.SetVoltage(0.0_V); }

private:
    void LoadDeployPid();

    // 775 that runs intake
    // TalonSRX m_motor;
    frc::Timer m_timer;
    frc::DigitalInput m_photoEyeIn;
    frc::DigitalInput m_photoEyeOut;
    frc::Servo m_deployServo;

    SparkMax m_coralMotor;
    SparkRelativeEncoder m_coralRelativeEnc = m_coralMotor.GetEncoder();    
    SparkClosedLoopController& m_coralPIDController = m_coralMotor.GetClosedLoopController();
};
