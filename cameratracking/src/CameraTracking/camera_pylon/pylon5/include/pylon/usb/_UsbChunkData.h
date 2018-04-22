

//-----------------------------------------------------------------------------
//  (c) 2004-2008 by Basler Vision Technologies
//  Section: Vision Components
//  Project: GenApi
//-----------------------------------------------------------------------------
/*!
\file
\brief Basler USB3Vision camera interface
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------



#ifndef Basler_UsbChunkData_PARAMS_H
#define Basler_UsbChunkData_PARAMS_H

#include <GenApi/IEnumerationT.h>
#include <GenApi/NodeMapRef.h>
#include <GenApi/DLLLoad.h>

// common node types
#include <GenApi/IBoolean.h>
#include <GenApi/ICategory.h>
#include <GenApi/ICommand.h>
#include <GenApi/IEnumeration.h>
#include <GenApi/IEnumEntry.h>
#include <GenApi/IFloat.h>
#include <GenApi/IInteger.h>
#include <GenApi/IString.h>
#include <GenApi/IRegister.h>

#ifdef __GNUC__
#   undef GCC_VERSION
#   define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#   undef GCC_DIAGNOSTIC_AWARE
#   define GCC_DIAGNOSTIC_AWARE          (GCC_VERSION >= 40200)
#   undef GCC_DIAGNOSTIC_PUSH_POP_AWARE
#   define GCC_DIAGNOSTIC_PUSH_POP_AWARE (GCC_VERSION >= 40600)
#else
#   define GCC_DIAGNOSTIC_AWARE 0
#endif

#ifdef __GNUC__
    // GCC_DIAGNOSTIC_AWARE ensures that the internal deprecated warnings can be ignored by gcc.
    // As a result older gcc will not generate warnings about really used deprecated features.
#   if GCC_DIAGNOSTIC_AWARE
#       define GENAPI_DEPRECATED_FEATURE __attribute__((deprecated))
#   else
#       define GENAPI_DEPRECATED_FEATURE
#   endif
#elif defined(_MSC_VER)
#   define GENAPI_DEPRECATED_FEATURE __declspec(deprecated)
#else
#   define GENAPI_DEPRECATED_FEATURE
#endif

#if GCC_DIAGNOSTIC_AWARE
#   if GCC_DIAGNOSTIC_PUSH_POP_AWARE
#       pragma GCC diagnostic push
#   endif
#   pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif


//! The namespace containing the device's control interface and related enumeration types
namespace Basler_UsbChunkData
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************
    
    //! Valid values for ChunkGainSelector
    enum ChunkGainSelectorEnums
    {
        ChunkGainSelector_All   //!<Gain will be applied to all channels or taps
        
    };

    //! Valid values for ChunkCounterSelector
    enum ChunkCounterSelectorEnums
    {
        ChunkCounterSelector_Counter1,  //!<Select counter 1
        ChunkCounterSelector_Counter2   //!<Select counter 2
        
    };


    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! Basler USB3Vision camera interface
    class CUsbChunkData_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CUsbChunkData_Params(void);

            //! Destructor
            ~CUsbChunkData_Params(void);

            //! Initializes the references
            void _Initialize(GenApi::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Selects which Gain to retrieve data from. Applies to: ace

        Selects which Gain to retrieve data from.
    
        \b Visibility = Expert
    
    */
    GenApi::IEnumerationT<ChunkGainSelectorEnums > &ChunkGainSelector;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Returns the gain used to capture the image. Applies to: ace

        Returns the gain used to capture the image.
    
        \b Visibility = Beginner
        
    
        \b Selected by : ChunkGainSelector
    
    */
    GenApi::IFloat &ChunkGain;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Returns the exposure time used to capture the image. Applies to: ace

        Returns the exposure time used to capture the image.
    
        \b Visibility = Beginner
        
    
    */
    GenApi::IFloat &ChunkExposureTime;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Indicates the value of the timestamp when the image was acquired. Applies to: ace

        This integer indicates the value of the timestamp when the image was acquired.
    
        \b Visibility = Beginner
        
    
    */
    GenApi::IInteger &ChunkTimestamp;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief A bit field that indicates the status of all of the camera's input and output lines when the image was acquired. Applies to: ace

        This value is a bit field that indicates the status of all of the camera's input and output lines when the image was acquired.
    
        \b Visibility = Beginner
        
    
    */
    GenApi::IInteger &ChunkLineStatusAll;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Selects which counter to retrieve data from. Applies to: ace

        Selects which counter to retrieve data from.
    
        \b Visibility = Expert
    
    */
    GenApi::IEnumerationT<ChunkCounterSelectorEnums > &ChunkCounterSelector;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Indicates the value of the selected Chunk counter. Applies to: ace

        Indicates the value of the selected Chunk counter.
    
        \b Visibility = Beginner
        
    
        \b Selected by : ChunkCounterSelector
    
    */
    GenApi::IInteger &ChunkCounterValue;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Return the index of the active set of the running sequencer included in the payload. Applies to: ace

        Return the index of the active set of the running sequencer included in the payload
    
        \b Visibility = Beginner
        
    
    */
    GenApi::IInteger &ChunkSequencerSetActive;
    
    //@}
    

    //! \name ChunkData - This category includes items related to the chunk data that can be appended to the image data
    //@{
    /*!
        \brief Indicates the value of CRC checksum. Applies to: ace

        This integer indicates the value of CRC checksum.
    
        \b Visibility = Beginner
        
    
    */
    GenApi::IInteger &ChunkPayloadCRC16;
    
    //@}
    


    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CUsbChunkData_Params(CUsbChunkData_Params&);

            //! not implemented assignment operator
            CUsbChunkData_Params& operator=(CUsbChunkData_Params&);

        //! \endcond
    };


    //**************************************************************************************************
    // Parameter class implementation
    //**************************************************************************************************

    //! \cond HIDE_CLASS_METHODS

    inline CUsbChunkData_Params::CUsbChunkData_Params(void)
        : ChunkGainSelector( *new GenApi::CEnumerationTRef<ChunkGainSelectorEnums>() )
        , ChunkGain( *new GenApi::CFloatRef() )
        , ChunkExposureTime( *new GenApi::CFloatRef() )
        , ChunkTimestamp( *new GenApi::CIntegerRef() )
        , ChunkLineStatusAll( *new GenApi::CIntegerRef() )
        , ChunkCounterSelector( *new GenApi::CEnumerationTRef<ChunkCounterSelectorEnums>() )
        , ChunkCounterValue( *new GenApi::CIntegerRef() )
        , ChunkSequencerSetActive( *new GenApi::CIntegerRef() )
        , ChunkPayloadCRC16( *new GenApi::CIntegerRef() )
        
    {
    }

    inline CUsbChunkData_Params::~CUsbChunkData_Params(void)
    {
        delete static_cast < GenApi::CEnumerationTRef<ChunkGainSelectorEnums> *> (&ChunkGainSelector );
        delete static_cast < GenApi::CFloatRef*> (&ChunkGain );
        delete static_cast < GenApi::CFloatRef*> (&ChunkExposureTime );
        delete static_cast < GenApi::CIntegerRef*> (&ChunkTimestamp );
        delete static_cast < GenApi::CIntegerRef*> (&ChunkLineStatusAll );
        delete static_cast < GenApi::CEnumerationTRef<ChunkCounterSelectorEnums> *> (&ChunkCounterSelector );
        delete static_cast < GenApi::CIntegerRef*> (&ChunkCounterValue );
        delete static_cast < GenApi::CIntegerRef*> (&ChunkSequencerSetActive );
        delete static_cast < GenApi::CIntegerRef*> (&ChunkPayloadCRC16 );
        
    }

    inline void CUsbChunkData_Params::_Initialize(GenApi::INodeMap* _Ptr)
    {
        static_cast<GenApi::CEnumerationTRef<ChunkGainSelectorEnums> *> (&ChunkGainSelector )->SetReference(_Ptr->GetNode("ChunkGainSelector"));
    static_cast<GenApi::CEnumerationTRef<ChunkGainSelectorEnums> *> (&ChunkGainSelector )->SetNumEnums(1);
    static_cast<GenApi::CEnumerationTRef<ChunkGainSelectorEnums> *> (&ChunkGainSelector )->SetEnumReference( ChunkGainSelector_All, "All" );        static_cast<GenApi::CFloatRef*> (&ChunkGain )->SetReference(_Ptr->GetNode("ChunkGain"));
    static_cast<GenApi::CFloatRef*> (&ChunkExposureTime )->SetReference(_Ptr->GetNode("ChunkExposureTime"));
    static_cast<GenApi::CIntegerRef*> (&ChunkTimestamp )->SetReference(_Ptr->GetNode("ChunkTimestamp"));
    static_cast<GenApi::CIntegerRef*> (&ChunkLineStatusAll )->SetReference(_Ptr->GetNode("ChunkLineStatusAll"));
    static_cast<GenApi::CEnumerationTRef<ChunkCounterSelectorEnums> *> (&ChunkCounterSelector )->SetReference(_Ptr->GetNode("ChunkCounterSelector"));
    static_cast<GenApi::CEnumerationTRef<ChunkCounterSelectorEnums> *> (&ChunkCounterSelector )->SetNumEnums(2);
    static_cast<GenApi::CEnumerationTRef<ChunkCounterSelectorEnums> *> (&ChunkCounterSelector )->SetEnumReference( ChunkCounterSelector_Counter1, "Counter1" );        static_cast<GenApi::CEnumerationTRef<ChunkCounterSelectorEnums> *> (&ChunkCounterSelector )->SetEnumReference( ChunkCounterSelector_Counter2, "Counter2" );        static_cast<GenApi::CIntegerRef*> (&ChunkCounterValue )->SetReference(_Ptr->GetNode("ChunkCounterValue"));
    static_cast<GenApi::CIntegerRef*> (&ChunkSequencerSetActive )->SetReference(_Ptr->GetNode("ChunkSequencerSetActive"));
    static_cast<GenApi::CIntegerRef*> (&ChunkPayloadCRC16 )->SetReference(_Ptr->GetNode("ChunkPayloadCRC16"));
    
    }

    inline const char* CUsbChunkData_Params::_GetVendorName(void)
    {
        return "Basler";
    }

    inline const char* CUsbChunkData_Params::_GetModelName(void)
    {
        return "UsbChunkData";
    }

    //! \endcond

} // namespace Basler_UsbChunkData

#if GCC_DIAGNOSTIC_AWARE
#   if GCC_DIAGNOSTIC_PUSH_POP_AWARE
#       pragma GCC diagnostic pop
#   else
#       pragma GCC diagnostic warning "-Wdeprecated-declarations"
#   endif
#endif

#undef GENAPI_DEPRECATED_FEATURE
#endif // Basler_UsbChunkData_PARAMS_H
