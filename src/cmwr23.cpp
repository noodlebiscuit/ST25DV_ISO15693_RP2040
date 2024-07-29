#include "cmwr23.h"

/// @brief default constructor
CMWR23::CMWR23()
{
    _sensorProperties.clear();
    _sensorProperties.push_back(_IMEI_DEFAULT);
    _sensorProperties.push_back(_MODL_DEFAULT);
    _sensorProperties.push_back(_MFDT_DEFAULT);
    _sensorProperties.push_back(_HWVN_DEFAULT);
    _sensorProperties.push_back(_BTVN_DEFAULT);
    _sensorProperties.push_back(_APVN_DEFAULT);
    _sensorProperties.push_back(_PMVN_DEFAULT);
    _sensorProperties.push_back(_ANGL_DEFAULT);
    _sensorProperties.push_back(_CMST_DEFAULT);
    _sensorProperties.push_back(_TLIV_DEFAULT);
    _sensorProperties.push_back(_STST_DEFAULT);
    _sensorProperties.push_back(_STTS_DEFAULT);
};

/// @brief default destructor
CMWR23::~CMWR23() {};

/// @brief set the content for a specific property
/// @param parameter property before
/// @param value property content
void CMWR23::SetProperty(CMWR_Parameter parameter, const char *value)
{
    std::string v;
    // process the query command
    switch (parameter)
    {
    case CMWR_Parameter::angl:
        v.append(_ANGL);
        break;
    case CMWR_Parameter::apvn:
        v.append(_APVN);
        break;
    case CMWR_Parameter::btvn:
        v.append(_BTVN);
        break;
    case CMWR_Parameter::cmst:
        v.append(_CMST);
        break;
    case CMWR_Parameter::hwvn:
        v.append(_HWVN);
        break;
    case CMWR_Parameter::imei:
        v.append(_IMEI);
        break;
    case CMWR_Parameter::mfdt:
        v.append(_MFDT);
        break;
    case CMWR_Parameter::modl:
        v.append(_MODL);
        break;
    case CMWR_Parameter::pmvn:
        v.append(_PMVN);
        break;
    case CMWR_Parameter::stst:
        v.append(_STST);
        break;
    case CMWR_Parameter::stts:
        v.append(_STTS);
        break;
    case CMWR_Parameter::tliv:
        v.append(_TLIV);
        break;
    }
    v.append(":");
    v.append(value);
    _sensorProperties[(byte)parameter - 0x01] = v;
}

/// @brief returns the formatted string for a specic nfc sensor property
/// @param parameter property to search against
std::string CMWR23::GetSensorProperty(CMWR_Parameter parameter)
{
    return _sensorProperties[(byte)parameter - 0x01];
}
