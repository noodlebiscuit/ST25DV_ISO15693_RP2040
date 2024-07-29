#ifndef _CMWR23_
#define _CMWR23_

#include <Arduino.h>
#include <sstream>
#include <vector>
#include <string>

///
/// @brief  Describes each of the commands that this reader supports
///
enum CMWR_Parameter : uint8_t
{
    none = 0x00,
    imei = 0x01,
    modl = 0x02,
    mfdt = 0x03,
    hwvn = 0x04,
    btvn = 0x05,
    apvn = 0x06,
    pmvn = 0x07,
    angl = 0x08,
    cmst = 0x09,
    tliv = 0x0a,
    stst = 0x0b,
    stts = 0x0c
};

class CMWR23
{

#define _IMEI "imei"
#define _MODL "modl"
#define _MFDT "mfdt"
#define _HWVN "hwvn"
#define _BTVN "btvn"
#define _APVN "apvn"
#define _PMVN "pmvn"
#define _ANGL "angl"
#define _CMST "cmst"
#define _TLIV "tliv"
#define _STST "stst"
#define _STTS "stts"

#define _IMEI_DEFAULT "imei:753000080000000"
#define _MODL_DEFAULT "modl:CMWR 23"
#define _MFDT_DEFAULT "mfdt:010170"
#define _HWVN_DEFAULT "hwvn:13"
#define _BTVN_DEFAULT "btvn:1.13.0"
#define _APVN_DEFAULT "apvn:1.13.0"
#define _PMVN_DEFAULT "pmvn:0.8.0"
#define _ANGL_DEFAULT "angl:?"
#define _CMST_DEFAULT "cmst:ship"
#define _TLIV_DEFAULT "tliv:3.47 2312041113"
#define _STST_DEFAULT "stst:OK 20"
#define _STTS_DEFAULT "stts:2401100506"

private:
    std::vector<std::string> _sensorProperties;

public:
    /// @brief default constructor
    CMWR23();

    /// @brief default destructor
    ~CMWR23();

    void SetProperty(CMWR_Parameter, const char*);
    std::string GetSensorProperty(CMWR_Parameter);
};

#endif