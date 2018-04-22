//-----------------------------------------------------------------------------
//  Copyright (c) 2008-2015 Basler AG
//  Section: Basler Components
//  Project: PYLON
//  Author:  Thomas Koeller
//  $Header:  $
//  This is a generated file - do not edit!
//-----------------------------------------------------------------------------

#if !defined (API_AUTOCONF_H)
#define API_AUTOCONF_H

#   ifdef PYLON_STATIC
#       define APIEXPORT
#       define APIIMPORT
#   else
#       define APIEXPORT    __attribute__((visibility("default")))
#       define APIIMPORT
#endif

#define CDECL        
#define PYLON_DEPRECATED(x) __attribute__((deprecated))

#define interface struct

#endif /* !defined(API_AUTOCONF_H) */
