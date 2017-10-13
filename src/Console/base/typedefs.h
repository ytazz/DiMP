#pragma once

// disable Visual C++ decprecation warnings
#define	_SCL_SECURE_NO_WARNINGS
#define	_CRT_SECURE_NO_WARNINGS

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// Scenebuilder
//#define SB_UINT_DEFINED
#include <sbtypes.h>
#include <sbxml.h>
#include <sbmessage.h>
#include <sbcriticalsection.h>
#include <sbevent.h>
#include <sbthread.h>
#include <sbmutex.h>
using namespace Scenebuilder;

// Springhead
#include <Springhead.h>
using namespace Spr;

// DiMP
#include <DiMP/DiMP.h>

// boost
#include <boost/array.hpp>
