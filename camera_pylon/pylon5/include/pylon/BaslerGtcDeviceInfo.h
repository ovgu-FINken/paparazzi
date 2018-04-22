//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2015 Basler AG
//  http://www.baslerweb.com
//  Author:  AH
//-----------------------------------------------------------------------------
/*!
\file
\brief    Definition of CBaslerGenTlDeviceInfo
*/

#ifndef BASLERGENTLDEVICEINFO_H__
#define BASLERGENTLDEVICEINFO_H__

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

    namespace Key
    {
        const char* const TlTypeKey = "TLType";
        const char* const InterfaceIDKey = "GTLInterfaceID";
        const char* const GTLDeviceIDKey = "GTLDeviceID";
        const char* const AccessStatusKey = "AccessStatus";
    }

    /*!
    \brief PylonGenTl specific Device Info object.
    */
    class PYLONBASE_API CBaslerGenTlDeviceInfo : public Pylon::CDeviceInfo
    {
    public:
        CBaslerGenTlDeviceInfo();
        CBaslerGenTlDeviceInfo(const CDeviceInfo& di);
        virtual ~CBaslerGenTlDeviceInfo();

        ///Retrieves the transport layer type.
        ///This property is identified by Key::TLTypeKey.
        String_t GetTlType() const;
        ///Sets the above property.
        CBaslerGenTlDeviceInfo& SetTlType( const String_t& );
        ///Returns true if the above property is available.
        bool IsTlTypeAvailable() const;

        ///Retrieves the ID of the interface the device is connected to.
        ///This property is identified by Key::InterfaceIDKey.
        String_t GetInterfaceID() const;
        ///Sets the above property.
        CBaslerGenTlDeviceInfo& SetInterfaceID( const String_t& );
        ///Returns true if the above property is available.
        bool IsInterfaceIDAvailable() const;

        ///Retrieves the device ID of the device.
        ///This property is identified by Key::GTLDeviceIDKey.
        String_t GetGTLDeviceID() const;
        ///Sets the above property.
        CBaslerGenTlDeviceInfo& SetGTLDeviceID( const String_t& );
        ///Returns true if the above property is available.
        bool IsGTLDeviceIDAvailable() const;

        ///Retrieves the access status of the device.
        ///This property is identified by Key::AccessStatusKey.
        String_t GetAccessStatus() const;
        ///Sets the above property.
        CBaslerGenTlDeviceInfo& SetAccessStatus( const String_t& );
        ///Returns true if the above property is available.
        bool IsAccessStatusAvailable() const;
    };
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif // BASLERGENTLDEVICEINFO_H__
