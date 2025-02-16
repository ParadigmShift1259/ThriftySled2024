#pragma once

#include "Constants.h"
#include "ConstantsCANIDs.h"

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

using namespace rev::spark;
using namespace frc;

constexpr double kIngestSpeed = 1.0;
constexpr double kReleaseSpeed = -1.0;

constexpr double c_LoadCoralPosition = 10.0;
constexpr double c_ParkForClimbPosition = 25.0;

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
    void AlignIntake();
    // Retracts the intake into the robot
    void ParkIntakeForClimb();
    void ParkIntakeAtZero();
    void GoToPosition(double turns);
    double GetPosition() { return m_deployRelativeEnc.GetPosition(); }
    void Stop() { Set(0.0); }

private:
    void LoadDeployPid();

    /// 775 that runs intake
    // TalonSRX m_motor;
    frc::Timer m_timer;
    // frc::DigitalInput m_photoEye;

    SparkMax m_deployMotor;
    SparkRelativeEncoder m_deployRelativeEnc = m_deployMotor.GetEncoder();    
    SparkClosedLoopController m_deployPIDController = m_deployMotor.GetClosedLoopController();

    // static constexpr bool kIntakeExtend = true;
    // static constexpr bool kIntakeRetract = false;
};
