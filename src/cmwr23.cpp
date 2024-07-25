#include "cmwr23.h"

/// @brief default constructor
CMWR23::CMWR23()
{
    _sensorProperties.clear();
    _sensorProperties.push_back(_IMEI);
    _sensorProperties.push_back(_MODL);
    _sensorProperties.push_back(_MFDT);
    _sensorProperties.push_back(_HWVN);
    _sensorProperties.push_back(_BTVN);
    _sensorProperties.push_back(_APVN);
    _sensorProperties.push_back(_PMVN);
    _sensorProperties.push_back(_ANGL);
    _sensorProperties.push_back(_CMST);
    _sensorProperties.push_back(_TLIV);
    _sensorProperties.push_back(_STST);
    _sensorProperties.push_back(_STTS);
};

/// @brief default constructor
CMWR23::~CMWR23() {};

/// @brief set the content for a specific property
/// @param parameter property before
/// @param value property content
void CMWR23::SetProperty(CMWR_Parameter parameter, std::string value)
{
    // process the query command
    switch (parameter)
    {
    case CMWR_Parameter::angl:

        break;
    case CMWR_Parameter::apvn:

        break;
    case CMWR_Parameter::btvn:

        break;
    case CMWR_Parameter::cmst:

        break;
    case CMWR_Parameter::hwvn:

        break;
    case CMWR_Parameter::imei:

        break;
    case CMWR_Parameter::mfdt:

        break;
    case CMWR_Parameter::modl:

        break;
    case CMWR_Parameter::pmvn:

        break;
    case CMWR_Parameter::stst:

        break;
    case CMWR_Parameter::stts:

        break;
    case CMWR_Parameter::tliv:

        break;
    }
}

std::string GetSensorProperty(CMWR_Parameter parameter)
{
    return "";
}
