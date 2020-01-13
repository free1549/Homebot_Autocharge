/*****************************************************************************
 * Code related to the platform
 * Author: Dong Xiaoguang
 * Created on 2017/06/15
 *****************************************************************************/
#ifndef PLATFORM_H_INCLUDED
#define PLATFORM_H_INCLUDED

//-----------------------------choose platform---------------------------------
//#define MY_WINDOWS
//#define MY_ANDROID
#define MY_EVB

//---------------------------enable log or printf------------------------------
#define ALGO_DEBUG                      1
#if ALGO_DEBUG
	#define myprintf(format, ...) OutputDebugLog(format, ##__VA_ARGS__)
#else
    #define myprintf(format, ...)
#endif // ALGO_DEBUG

//-----------------------------------------------------------------------------
//Algorithm API
#ifdef MY_WINDOWS
	void OutputDebugLog(const char *format,...);
	#pragma once
	#define DLL_IMPLEMENT
	#ifdef DLL_IMPLEMENT
		#define DLL_API extern "C" __declspec(dllexport)
	#else
		#define DLL_API extern "C" __declspec(dllimport)
	#endif
#endif

#ifdef MY_ANDROID
	void OutputDebugPrintf(const char *format,...);
	void OutputDebugLog(const char *format,...);
	#define DLL_API
#endif
		
#ifdef MY_EVB
	void OutputDebugPrintf(const char *format,...);
	void OutputDebugLog(const char *format,...);
	#define DLL_API
#endif		


//----------------------------------------------------------------------------
//Platform Detect
#if (defined MY_WINDOWS) && (defined MY_ANDROID) && (defined MY_EVB)
	#error Three platforms are defined, choose ONLY one platform in platform.h
#endif
#if (!defined MY_WINDOWS) && (!defined MY_ANDROID) && (!defined MY_EVB)
	#error No platform is defined, choose one platform in platform.h
#endif

#endif

//PLATFORM_H_INCLUDED
