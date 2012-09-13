/*
 * Copyright (c) 2011, Kevin LeBlanc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




//=============================================================================
#ifndef INCLUDE_TELEOP_COMMON_HPP
#define INCLUDE_TELEOP_COMMON_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <vector>
#include <string>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/** Axis types (can be used as indices in an array) */
typedef enum {
  TELEOP_AXIS_TYPE_FIRST = 0,
  TELEOP_AXIS_TYPE_LIN_X = TELEOP_AXIS_TYPE_FIRST,
  TELEOP_AXIS_TYPE_LIN_Y,
  TELEOP_AXIS_TYPE_LIN_Z,
  TELEOP_AXIS_TYPE_ROT_X,
  TELEOP_AXIS_TYPE_ROT_Y,
  TELEOP_AXIS_TYPE_ROT_Z,
  TELEOP_AXIS_TYPE_HAT0_X,
  TELEOP_AXIS_TYPE_HAT0_Y,
  TELEOP_AXIS_TYPE_HAT1_X,
  TELEOP_AXIS_TYPE_HAT1_Y,
  TELEOP_AXIS_TYPE_HAT2_X,
  TELEOP_AXIS_TYPE_HAT2_Y,
  TELEOP_AXIS_TYPE_HAT3_X,
  TELEOP_AXIS_TYPE_HAT3_Y,
  TELEOP_AXIS_TYPE_THROTTLE,
  TELEOP_AXIS_TYPE_UNKNOWN,
  TELEOP_AXIS_TYPE_LAST = TELEOP_AXIS_TYPE_UNKNOWN,
  TELEOP_AXIS_TYPE_COUNT
} TeleopAxisType;

/** Axis value (typedef makes it easier to change width) */
typedef double TeleopAxisValue;

/** Button types (can be used as indices in an array) */
typedef enum {
  TELEOP_BUTTON_TYPE_FIRST = 0,
  TELEOP_BUTTON_TYPE_0 = TELEOP_BUTTON_TYPE_FIRST,
  TELEOP_BUTTON_TYPE_1,
  TELEOP_BUTTON_TYPE_2,
  TELEOP_BUTTON_TYPE_3,
  TELEOP_BUTTON_TYPE_4,
  TELEOP_BUTTON_TYPE_5,
  TELEOP_BUTTON_TYPE_6,
  TELEOP_BUTTON_TYPE_7,
  TELEOP_BUTTON_TYPE_8,
  TELEOP_BUTTON_TYPE_9,
  TELEOP_BUTTON_TYPE_A,
  TELEOP_BUTTON_TYPE_B,
  TELEOP_BUTTON_TYPE_C,
  TELEOP_BUTTON_TYPE_X,
  TELEOP_BUTTON_TYPE_Y,
  TELEOP_BUTTON_TYPE_Z,
  TELEOP_BUTTON_TYPE_RIGHT,
  TELEOP_BUTTON_TYPE_LEFT,
  TELEOP_BUTTON_TYPE_SELECT,
  TELEOP_BUTTON_TYPE_START,
  TELEOP_BUTTON_TYPE_STOP,
  TELEOP_BUTTON_TYPE_TRIGGER,
  TELEOP_BUTTON_TYPE_UNKNOWN,
  TELEOP_BUTTON_TYPE_LAST = TELEOP_BUTTON_TYPE_UNKNOWN,
  TELEOP_BUTTON_TYPE_COUNT
} TeleopButtonType;

/** Button value type (typedef makes it easier to change signed/width) */
typedef int TeleopButtonValue;

/** Teleop device axis (value is in [TELEOP_AXIS_MIN, TELEOP_AXIS_MAX]) */
typedef struct {
  TeleopAxisType type;
  TeleopAxisValue value;
} TeleopAxis;

/** Teleop device button (value of 0 means off) */
typedef struct {
  TeleopButtonType type;
  TeleopButtonValue value;
} TeleopButton;

/** Teleop device state */
typedef struct {
  std::vector<TeleopAxis> axes;
  std::vector<TeleopButton> buttons;
} TeleopState;




//=============================================================================
//Globals
//=============================================================================

/** Axis value min (0=off) */
static const TeleopAxisValue TELEOP_AXIS_MIN = -1.0;

/** Axis value max (0=off) */
static const TeleopAxisValue TELEOP_AXIS_MAX = 1.0;




//=============================================================================
//Function prototypes
//=============================================================================

/**
 * Function to determine axis type name.
 *
 *   @param axisType [in] - axis type
 *
 *   @return axis type name
 */
std::string teleopAxisName(TeleopAxisType axisType);

/**
 * Function to determine button type name.
 *
 *   @param buttonType [in] - button type
 *
 *   @return button type name
 */
std::string teleopButtonName(TeleopButtonType buttonType);




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_COMMON_HPP
//=============================================================================
