#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
//#include <format>

using namespace nt;

using NetTableSPtr = std::shared_ptr<nt::NetworkTable>; 

template <typename T> // T can be bool, double, std::string
class DashBoardValue
{
 public:
    DashBoardValue(std::string_view tableName, std::string_view valueName, const T& defaultVal)
        : m_valueName(valueName)
        , m_defaultValue(defaultVal)
    {
        if (!m_spNetTable)
        {
            m_spNetTable = NetworkTableInstance::GetDefault().GetTable(tableName);
        }

        Put(m_defaultValue);    // Establish the value on the dashboard
    }

    const T& Get()
    {
        if (m_spNetTable)
        {
            if constexpr (std::is_integral_v<T>)
            {
                 return m_spNetTable->GetBoolean(m_valueName, m_defaultValue);
            }
            else if constexpr (std::is_floating_point_v<T>)
            {
                 return m_spNetTable->GetNumber(m_valueName, m_defaultValue);
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                 return m_spNetTable->GetString(m_valueName, m_defaultValue);
            }

            static T t;
            return t;
        }
    }
    
    void Put(const T& newValue)
    {
        if (m_spNetTable)
        {
            if constexpr (std::is_integral_v<T>)
            {
                m_spNetTable->PutBoolean(m_valueName, newValue);
            }
            else if constexpr (std::is_floating_point_v<T>)
            {
                m_spNetTable->PutNumber(m_valueName, newValue);
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                m_spNetTable->PutString(m_valueName, newValue);
            }
        }
    }
 
    const std::string c_slash{"/"};
    const std::string Path() const 
    {
         std::string path = std::string{m_spNetTable ? m_spNetTable->GetPath() : "/unkTable"};
         path += c_slash + m_valueName; 
         return path;
    }

 private:
    std::string  m_valueName;
    T            m_defaultValue;
    NetTableSPtr m_spNetTable;
};