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
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <string>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Function definitions
//=============================================================================
std::string teleopAxisName(TeleopAxisType axisType) {
  switch (axisType) {
    case TELEOP_AXIS_TYPE_LIN_X:        return std::string("TELEOP_AXIS_TYPE_LIN_X");
    case TELEOP_AXIS_TYPE_LIN_Y:        return std::string("TELEOP_AXIS_TYPE_LIN_Y");
    case TELEOP_AXIS_TYPE_LIN_Z:        return std::string("TELEOP_AXIS_TYPE_LIN_Z");
    case TELEOP_AXIS_TYPE_ROT_X:        return std::string("TELEOP_AXIS_TYPE_ROT_X");
    case TELEOP_AXIS_TYPE_ROT_Y:        return std::string("TELEOP_AXIS_TYPE_ROT_Y");
    case TELEOP_AXIS_TYPE_ROT_Z:        return std::string("TELEOP_AXIS_TYPE_ROT_Z");
    case TELEOP_AXIS_TYPE_HAT0_X:       return std::string("TELEOP_AXIS_TYPE_HAT0_X");
    case TELEOP_AXIS_TYPE_HAT0_Y:       return std::string("TELEOP_AXIS_TYPE_HAT0_Y");
    case TELEOP_AXIS_TYPE_HAT1_X:       return std::string("TELEOP_AXIS_TYPE_HAT1_X");
    case TELEOP_AXIS_TYPE_HAT1_Y:       return std::string("TELEOP_AXIS_TYPE_HAT1_Y");
    case TELEOP_AXIS_TYPE_HAT2_X:       return std::string("TELEOP_AXIS_TYPE_HAT2_X");
    case TELEOP_AXIS_TYPE_HAT2_Y:       return std::string("TELEOP_AXIS_TYPE_HAT2_Y");
    case TELEOP_AXIS_TYPE_HAT3_X:       return std::string("TELEOP_AXIS_TYPE_HAT3_X");
    case TELEOP_AXIS_TYPE_HAT3_Y:       return std::string("TELEOP_AXIS_TYPE_HAT3_Y");
    case TELEOP_AXIS_TYPE_THROTTLE:     return std::string("TELEOP_AXIS_TYPE_THROTTLE");
    case TELEOP_AXIS_TYPE_UNKNOWN:
    default:
      return "TELEOP_AXIS_TYPE_UNKNOWN";
  }
}
//=============================================================================
std::string teleopButtonName(TeleopButtonType buttonType) {
  switch (buttonType) {
    case TELEOP_BUTTON_TYPE_0:          return std::string("TELEOP_BUTTON_TYPE_0");
    case TELEOP_BUTTON_TYPE_1:          return std::string("TELEOP_BUTTON_TYPE_1");
    case TELEOP_BUTTON_TYPE_2:          return std::string("TELEOP_BUTTON_TYPE_2");
    case TELEOP_BUTTON_TYPE_3:          return std::string("TELEOP_BUTTON_TYPE_3");
    case TELEOP_BUTTON_TYPE_4:          return std::string("TELEOP_BUTTON_TYPE_4");
    case TELEOP_BUTTON_TYPE_5:          return std::string("TELEOP_BUTTON_TYPE_5");
    case TELEOP_BUTTON_TYPE_6:          return std::string("TELEOP_BUTTON_TYPE_6");
    case TELEOP_BUTTON_TYPE_7:          return std::string("TELEOP_BUTTON_TYPE_7");
    case TELEOP_BUTTON_TYPE_8:          return std::string("TELEOP_BUTTON_TYPE_8");
    case TELEOP_BUTTON_TYPE_9:          return std::string("TELEOP_BUTTON_TYPE_9");
    case TELEOP_BUTTON_TYPE_A:          return std::string("TELEOP_BUTTON_TYPE_A");
    case TELEOP_BUTTON_TYPE_B:          return std::string("TELEOP_BUTTON_TYPE_B");
    case TELEOP_BUTTON_TYPE_C:          return std::string("TELEOP_BUTTON_TYPE_C");
    case TELEOP_BUTTON_TYPE_X:          return std::string("TELEOP_BUTTON_TYPE_X");
    case TELEOP_BUTTON_TYPE_Y:          return std::string("TELEOP_BUTTON_TYPE_Y");
    case TELEOP_BUTTON_TYPE_Z:          return std::string("TELEOP_BUTTON_TYPE_Z");
    case TELEOP_BUTTON_TYPE_RIGHT:      return std::string("TELEOP_BUTTON_TYPE_RIGHT");
    case TELEOP_BUTTON_TYPE_LEFT:       return std::string("TELEOP_BUTTON_TYPE_LEFT");
    case TELEOP_BUTTON_TYPE_SELECT:     return std::string("TELEOP_BUTTON_TYPE_SELECT");
    case TELEOP_BUTTON_TYPE_START:      return std::string("TELEOP_BUTTON_TYPE_START");
    case TELEOP_BUTTON_TYPE_STOP:       return std::string("TELEOP_BUTTON_TYPE_STOP");
    case TELEOP_BUTTON_TYPE_TRIGGER:    return std::string("TELEOP_BUTTON_TYPE_TRIGGER");
    case TELEOP_BUTTON_TYPE_UNKNOWN:
    default:
      return std::string("TELEOP_BUTTON_TYPE_UNKNOWN");
  }
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
