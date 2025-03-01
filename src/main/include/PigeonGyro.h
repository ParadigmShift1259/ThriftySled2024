#pragma once

#include <units/angular_velocity.h>

#define USE_PIGEON_2
#ifdef USE_PIGEON_2
#include <ctre/phoenix6/Pigeon2.hpp>
#endif

class PigeonGyro
{
public:
    PigeonGyro(int CANId);

    frc::Rotation2d GetRotation2d();
    units::degree_t GetYaw();
    units::degree_t GetRoll() { return m_gyro.GetRoll().GetValue(); }
    units::degree_t GetPitch() { return m_gyro.GetPitch().GetValue(); }
    //void Reset();
    void Set(units::degree_t yaw);
    units::degrees_per_second_t GetTurnRate();
         
#ifdef USE_PIGEON_2
    ctre::phoenix6::hardware::Pigeon2 m_gyro;
#else
    PigeonIMU m_gyro;
#endif
};