using namespace ViconDataStreamSDK::CPP;

namespace
{
    std::string Adapt(const bool i_Value) { return i_Value ? "True" : "False"; }

    // Set time standard
    std::string Adapt(const TimecodeStandard::Enum i_Standard)
    {
        switch (i_Standard)
        {
        default:
        case TimecodeStandard::None:
            return "0";
        case TimecodeStandard::PAL:
            return "1";
        case TimecodeStandard::NTSC:
            return "2";
        case TimecodeStandard::NTSCDrop:
            return "3";
        case TimecodeStandard::Film:
            return "4";
        case TimecodeStandard::NTSCFilm:
            return "5";
        case TimecodeStandard::ATSC:
            return "6";
        }
    }

    // Doubt - Set the direction for i axis
    std::string Adapt(const Direction::Enum i_Direction)
    {
        switch (i_Direction)
        {
        case Direction::Forward:
            return "Forward";
        case Direction::Backward:
            return "Backward";
        case Direction::Left:
            return "Left";
        case Direction::Right:
            return "Right";
        case Direction::Up:
            return "Up";
        case Direction::Down:
            return "Down";
        default:
            return "Unknown";
        }
    }

    // Enable forceplate input device (not relevant)
    std::string Adapt(const DeviceType::Enum i_DeviceType)
    {
        switch (i_DeviceType)
        {
        case DeviceType::ForcePlate:
            return "ForcePlate";
        case DeviceType::Unknown:
        default:
            return "Unknown";
        }
    }

    // Set the unit for state variable
    std::string Adapt(const Unit::Enum i_Unit)
    {
        switch (i_Unit)
        {
        case Unit::Meter:
            return "Meter";
        case Unit::Volt:
            return "Volt";
        case Unit::NewtonMeter:
            return "NewtonMeter";
        case Unit::Newton:
            return "Newton";
        case Unit::Kilogram:
            return "Kilogram";
        case Unit::Second:
            return "Second";
        case Unit::Ampere:
            return "Ampere";
        case Unit::Kelvin:
            return "Kelvin";
        case Unit::Mole:
            return "Mole";
        case Unit::Candela:
            return "Candela";
        case Unit::Radian:
            return "Radian";
        case Unit::Steradian:
            return "Steradian";
        case Unit::MeterSquared:
            return "MeterSquared";
        case Unit::MeterCubed:
            return "MeterCubed";
        case Unit::MeterPerSecond:
            return "MeterPerSecond";
        case Unit::MeterPerSecondSquared:
            return "MeterPerSecondSquared";
        case Unit::RadianPerSecond:
            return "RadianPerSecond";
        case Unit::RadianPerSecondSquared:
            return "RadianPerSecondSquared";
        case Unit::Hertz:
            return "Hertz";
        case Unit::Joule:
            return "Joule";
        case Unit::Watt:
            return "Watt";
        case Unit::Pascal:
            return "Pascal";
        case Unit::Lumen:
            return "Lumen";
        case Unit::Lux:
            return "Lux";
        case Unit::Coulomb:
            return "Coulomb";
        case Unit::Ohm:
            return "Ohm";
        case Unit::Farad:
            return "Farad";
        case Unit::Weber:
            return "Weber";
        case Unit::Tesla:
            return "Tesla";
        case Unit::Henry:
            return "Henry";
        case Unit::Siemens:
            return "Siemens";
        case Unit::Becquerel:
            return "Becquerel";
        case Unit::Gray:
            return "Gray";
        case Unit::Sievert:
            return "Sievert";
        case Unit::Katal:
            return "Katal";

        case Unit::Unknown:
        default:
            return "Unknown";
        }
    }
#ifdef WIN32
    bool Hit()
    {
        bool hit = false;
        while (_kbhit())
        {
            getchar();
            hit = true;
        }
        return hit;
    }
#endif

    class NullBuffer : public std::streambuf
    {
    public:
        int overflow(int c) { return c; }
    };

    NullBuffer Null;
    std::ostream NullStream(&Null);

} // namespace