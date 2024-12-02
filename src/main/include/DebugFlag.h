#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

class DebugFlag
{
public:
    DebugFlag(const char* name, bool defaultVal);

    operator bool() { return m_netTableEntry.GetBoolean(m_defaultVal); }

private:
    nt::NetworkTableEntry   m_netTableEntry;
    bool                    m_defaultVal;

    static int              m_flagCount;
};