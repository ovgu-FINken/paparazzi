//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2008-2015 Basler AG
//  http://www.baslerweb.com
//  Author:  Italo Anzioso
//-----------------------------------------------------------------------------
/*!
\file
\brief    Definition of CBaslerCamEmuDeviceInfo
*/

#ifndef BASLERCAMDEMUEVICEINFO_H__
#define BASLERCAMDEMUEVICEINFO_H__

#if _MSC_VER > 1000
#pragma once
#endif //_MSC_VER > 1000

#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/PylonBase.h>
#include <pylon/DeviceInfo.h>

namespace Pylon
{
    /*!
    \brief PylonCamEmu specific Device Info object.
    */
    class PYLONBASE_API CBaslerCamEmuDeviceInfo : public Pylon::CDeviceInfo
    {
    public:
        CBaslerCamEmuDeviceInfo();
        CBaslerCamEmuDeviceInfo(const CDeviceInfo& di);
        virtual ~CBaslerCamEmuDeviceInfo();
    };
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif // BASLERCAMDEMUEVICEINFO_H__
