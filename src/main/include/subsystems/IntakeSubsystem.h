#pragma once

#include "Constants.h"
#include "ConstantsDigitalInputs.h"
#include "ConstantsCANIDs.h"

#include <frc/DigitalInput.h>

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
WPI_UNIGNORE_DEPRECATED

#include <rev/SparkMax.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace rev::spark;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;

constexpr double kIngestSpeed = 1.0;
constexpr double kReleaseSpeed = -1.0;

class IntakeSubsystem : public frc2::SubsystemBase
{
public:
    IntakeSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the intake at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);
    /// Extends the intake out of the robot
    void ExtendIntake();
    void ExtendIntake(double turns);
    // Retracts the intake into the robot
    void RetractIntake();
    void GoToPosition(double turns);
    double GetPosition() { return m_deployRelativeEnc.GetPosition(); }
    bool IsNotePresent() { return m_photoEye.Get(); }
    void EjectNote() { Set(kIngestSpeed); }
    void Stop() { Set(0.0); }

private:
    void LoadDeployPid();

    /// 775 that runs intake
    TalonSRX m_motor;
    frc::Timer m_timer;
    frc::DigitalInput m_photoEye;

    SparkMax m_deployMotor;
    SparkRelativeEncoder m_deployRelativeEnc = m_deployMotor.GetEncoder();    
    SparkClosedLoopController m_deployPIDController = m_deployMotor.GetClosedLoopController();
    SparkMax m_deployFollowMotor;
    SparkRelativeEncoder m_deployFollowRelativeEnc = m_deployFollowMotor.GetEncoder();    
    SparkClosedLoopController m_deployFollowPIDController = m_deployFollowMotor.GetClosedLoopController();

    static constexpr bool kIntakeExtend = true;
    static constexpr bool kIntakeRetract = false;
};
