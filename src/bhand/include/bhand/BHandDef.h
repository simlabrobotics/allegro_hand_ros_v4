/* Allegro, Copyright 2012 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */

/**
 * @file BHandDef.h
 * @author SimLab
 * @brief Definitions.
 */
#ifndef __BHANDDEF_H__
#define __BHANDDEF_H__

/* DLL export */
#if defined(WIN32) || defined(WINCE)
#    if defined(BHAND_EXPORTS)
#        define BHANDEXPORT __declspec(dllexport)
#    elif defined(BHAND_IMPORTS)
#        define BHANDEXPORT __declspec(dllimport)
#    else
#        define BHANDEXPORT
#    endif
#else
#    define BHANDEXPORT
#endif


#ifdef __cplusplus
#    define BHAND_EXTERN_C_BEGIN    extern "C" {
#    define BHAND_EXTERN_C_END    }
#else
#    define BHAND_EXTERN_C_BEGIN
#    define BHAND_EXTERN_C_END
#endif

#ifndef DEG2RAD
#define DEG2RAD (3.141592f/180.0f)
#endif

#ifndef RAD2DEG
#define RAD2DEG (180.0f/3.141592f)
#endif

#endif // __BHANDDEF_H__
