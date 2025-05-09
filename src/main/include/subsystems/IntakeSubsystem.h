#pragma once

#include "Constants.h"
#include "ConstantsCANIDs.h"
#include "DashBoardValue.h"

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

using namespace rev::spark;
using namespace frc;

constexpr double kIngestSpeed = 1.0;
constexpr double kReleaseSpeed = -1.0;

constexpr double c_InitalDeployPosition = 15.0;
constexpr double c_LoadCoralPosition = 9.238;
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
    /// Extends the intake past the pin pop
    void AlignIntake();
    /// Sets the intake position for loading coral
    void ParkIntakeForLoad();
    /// Sets the intake out of the way for climb
    void ParkIntakeForClimb();
    void GoToPosition(double turns);
    void GoToPositionRel(double turns);
    double GetPosition() { return m_deployRelativeEnc.GetPosition(); }
    void Stop() { Set(0.0); }

private:
    void LoadDeployPid();

    frc::Timer m_timer;

    SparkMax m_deployMotor;
    SparkRelativeEncoder m_deployRelativeEnc = m_deployMotor.GetEncoder();    
    SparkClosedLoopController m_deployPIDController = m_deployMotor.GetClosedLoopController();

    //DashBoardValue<double> m_dbvParkForClimb{"Intake", "ParkForClimb", c_ParkForClimbPosition};
};
