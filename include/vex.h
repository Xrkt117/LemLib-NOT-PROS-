#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#if defined(__has_include)
#if __has_include("vex_rotation.h") && __has_include("vex_triport.h") && __has_include("v5_api.h")
#define LEMLIB_FULL_VEX_SDK 1
#else
#define LEMLIB_FULL_VEX_SDK 0
#endif
#else
#define LEMLIB_FULL_VEX_SDK 1
#endif

// The VEX convenience macro collides with Chassis::waitUntil.
#ifdef waitUntil
#undef waitUntil
#endif
