#include "PigeonGyro.h"

PigeonGyro::PigeonGyro(int CANId) 
    : m_gyro(CANId)
{

}

frc::Rotation2d PigeonGyro::GetRotation2d()
{
#ifdef USE_PIGEON_2
    auto retVal = std::remainder(m_gyro.GetYaw().GetValueAsDouble(), 360.0);
#else
    auto retVal = std::remainder(m_gyro.GetFusedHeading(), 360.0);
#endif
    if (retVal > 180.0)
        retVal -= 360.0;
    return frc::Rotation2d(units::degree_t(retVal));
}

units::degree_t PigeonGyro::GetYaw()
{
#ifdef USE_PIGEON_2
    return m_gyro.GetYaw().GetValue();
#else
    return m_gyro.GetFusedHeading();
#endif
}


void PigeonGyro::Reset()
{
#ifdef USE_PIGEON_2
    m_gyro.SetYaw(units::degree_t(0.0));
#else
    m_gyro.SetFusedHeading(0.0);
#endif
}

void PigeonGyro::Set(units::degree_t yaw)
{
    m_gyro.SetYaw(yaw);
}

units::degrees_per_second_t PigeonGyro::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    turnRates[0] = m_gyro.GetAngularVelocityXDevice().GetValueAsDouble();
    turnRates[1] = m_gyro.GetAngularVelocityYDevice().GetValueAsDouble();
    turnRates[2] = m_gyro.GetAngularVelocityZDevice().GetValueAsDouble();
    // m_gyro.GetRawGyro(turnRates);
    return units::degrees_per_second_t(turnRates[2]); 
}