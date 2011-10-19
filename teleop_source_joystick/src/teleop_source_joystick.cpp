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
#include <teleop_source.hpp>
#include <teleop_source_joystick.hpp>
#include <linux/joystick.h>
#include <linux/input.h>
#include <linux/types.h>
#include <sys/select.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <cstdio>




//=============================================================================
//Defines
//=============================================================================

//Ensure we're using a valid joystick driver
#ifndef JS_VERSION
#error "JS_VERSION undefined"
#else
#if (JS_VERSION < 0x010000)
#error "JS_VERSION too low"
#endif
#endif




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Static member definitions for non-integral types
//=============================================================================
const std::string TeleopSourceJoystick::DEFAULT_DEVICE = std::string("/dev/input/js0");




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceJoystick::TeleopSourceJoystick(TeleopSourceCallback* callback,
                                           std::string device)
  : TeleopSource(callback), mDevice(device), mJoystickName(std::string("none")),
    mFileDescriptor(-1), mNumAxes(0), mNumButtons(0) {
  //Initialise array members
  for (int i = 0; i < ABS_CNT; i++) {
    mAxisMap[i] = ABS_MISC;
  }
  for (int i = 0; i < KEY_MAX - BTN_MISC + 1; i++) {
    mButtonMap[i] = BTN_MISC;
  }
}
//=============================================================================
TeleopSourceJoystick::~TeleopSourceJoystick() {
  //Stop this source
  stop();
}
//=============================================================================
std::string TeleopSourceJoystick::getJoystickName() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Return joystick name
  return mJoystickName;
}
//=============================================================================
bool TeleopSourceJoystick::prepareToListen() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //If device already open do nothing.  This means this method can be called
  //multiple times without problems.
  if (-1 != mFileDescriptor) {
    return true;
  }

  //Open device in non-blocking mode
  mFileDescriptor = open(mDevice.c_str(), O_RDONLY | O_NONBLOCK);
  if (-1 == mFileDescriptor) {
    std::fprintf(stderr, "TeleopSourceJoystick::prepareToListen: error opening device\n");
    return false;
  }

  //Get number of axes and buttons and corresponding maps
  if (0 > ioctl(mFileDescriptor, JSIOCGAXES, &mNumAxes)) {
    std::fprintf(stderr, "TeleopSourceJoystick::prepareToListen: error reading number of axes\n");
    return false;
  }
  if (0 > ioctl(mFileDescriptor, JSIOCGAXMAP, mAxisMap)) {
    std::fprintf(stderr, "TeleopSourceJoystick::prepareToListen: error reading axis map\n");
    return false;
  }
  if (0 > ioctl(mFileDescriptor, JSIOCGBUTTONS, &mNumButtons)) {
    std::fprintf(stderr, "TeleopSourceJoystick::prepareToListen: error reading number of buttons\n");
    return false;
  }
  if (0 > ioctl(mFileDescriptor, JSIOCGBTNMAP, mButtonMap)) {
    std::fprintf(stderr, "TeleopSourceJoystick::prepareToListen: error reading button map\n");
    return false;
  }

  //Get joystick name
  char name[128];
  if (0 > ioctl(mFileDescriptor, JSIOCGNAME(sizeof(name)), name)) {
    std::fprintf(stderr, "TeleopSourceJoystick::prepareToListen: error reading name\n");
    return false;
  }
  mJoystickName = std::string(name);

  //Print welcome message
  std::fprintf(stdout, "Device file:       %s\n", mDevice.c_str());
  std::fprintf(stdout, "Joystick name:     %s\n", mJoystickName.c_str());
  std::fprintf(stdout, "Num axes:          %u\n", mNumAxes);
  for (int i = 0; i < mNumAxes; i++) {
    std::fprintf(stdout, "  axis[%2d]   = 0x%04x -> %04d -> %s\n",
            i,
            mAxisMap[i],
            axisDriverTypeToTeleopType(mAxisMap[i]),
            teleopAxisName(axisDriverTypeToTeleopType(mAxisMap[i])).c_str());
  }
  std::fprintf(stdout, "Num buttons:       %u\n", mNumButtons);
  for (int i = 0; i < mNumButtons; i++) {
    std::fprintf(stdout, "  button[%2d] = 0x%04x -> %04d -> %s\n",
            i,
            mButtonMap[i],
            buttonDriverTypeToTeleopType(mButtonMap[i]),
            teleopButtonName(buttonDriverTypeToTeleopType(mButtonMap[i])).c_str());
  }

  //Print welcome message
  std::fprintf(stdout, "\n\nJoystick is ready.  Press CTRL-C to quit.\n");

  //Return success
  return true;
}
//=============================================================================
TeleopSource::ListenResult TeleopSourceJoystick::listen(int timeoutMilliseconds,
                                                        TeleopState* const teleopState) {
  //Sanity check
  if (NULL == teleopState) {
    std::fprintf(stderr, "TeleopSourceJoystick::listen: NULL teleop state\n");
    return LISTEN_RESULT_ERROR;
  }

  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Ensure state has correct number and types of axes and buttons
  if (mNumAxes != teleopState->axes.size()) {
    teleopState->axes.resize(mNumAxes);
    for (size_t i = 0; i < mNumAxes; i++) {
      teleopState->axes[i].type = axisDriverTypeToTeleopType(mAxisMap[i]);
      teleopState->axes[i].value = 0.0;
    }
  }

  if (mNumButtons != teleopState->buttons.size()) {
    teleopState->buttons.resize(mNumButtons);
    for (size_t i = 0; i < mNumButtons; i++) {
      teleopState->buttons[i].type = buttonDriverTypeToTeleopType(mButtonMap[i]);
      teleopState->buttons[i].value = 0;
    }
  }

  //Initialise a file descriptor set for select
  fd_set fileDescriptorSet;
  FD_ZERO (&fileDescriptorSet);
  FD_SET (mFileDescriptor, &fileDescriptorSet);

  //Initialise timeout value for select
  struct timeval timeout;
  timeout.tv_sec = timeoutMilliseconds / 1000;
  timeout.tv_usec = (timeoutMilliseconds % 1000) * 1000;

  //Use select to see if anything shows up before timeout
  int result = select(FD_SETSIZE, &fileDescriptorSet, NULL, NULL, &timeout);
  if (0 == result) {
    //Timeout
    return LISTEN_RESULT_UNCHANGED;
  } else if (-1 == result) {
    //Error
    std::fprintf(stderr, "TeleopSourceJoystick::listen: error in select() (%d)\n", errno);
    return LISTEN_RESULT_ERROR;
  }

  //Data available, read and process all events until queue is empty
  bool stateChanged = false;
  js_event event;
  while (true) {

    //Read one event (recall that file was opened in non-blocking mode)
    ssize_t numBytes = read(mFileDescriptor, &event, sizeof(js_event));
    if ((-1 == numBytes) && (EAGAIN == errno)) {
      //Queue is empty
      if (stateChanged) {
        return LISTEN_RESULT_CHANGED;
      } else {
        return LISTEN_RESULT_UNCHANGED;
      }
    } else if (sizeof(js_event) == numBytes) {
      //Handle this event
      switch(handleEvent(&event, teleopState)) {
        case LISTEN_RESULT_UNCHANGED:
          break;
        case LISTEN_RESULT_CHANGED:
          stateChanged = true;
          break;
        case LISTEN_RESULT_ERROR:
          std::fprintf(stderr, "TeleopSourceJoystick::listen: error in handleEvent()\n");
          return LISTEN_RESULT_ERROR;
        default:
          std::fprintf(stderr, "TeleopSourceJoystick::listen: invalid result from handleEvent()\n");
          return LISTEN_RESULT_ERROR;
      }
    } else {
      //Error
      std::fprintf(stderr, "TeleopSourceJoystick::listen: error in read()\n");
      return LISTEN_RESULT_ERROR;
    }
  }
}
//=============================================================================
bool TeleopSourceJoystick::doneListening() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Close device only if it was open.  This means this method can be called
  //multiple times without problems.
  if ((-1 != mFileDescriptor) && (0 != close(mFileDescriptor))) {
    std::fprintf(stderr, "TeleopSourceJoystick::doneListening: error closing device\n");
    return false;
  } else {
    mFileDescriptor = -1;
  }

  //Return success
  return true;
}
//=============================================================================
TeleopSource::ListenResult TeleopSourceJoystick::handleEvent(const js_event* const event,
                                                             TeleopState* const teleopState) {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Handle known events
  switch(event->type)
  {
    case JS_EVENT_AXIS | JS_EVENT_INIT:
      //Ignore init events, they're often wrong
      break;
    case JS_EVENT_AXIS:
      //Event number shouldn't be bigger than the vector
      if(event->number >= teleopState->axes.size()) {
        std::fprintf(stderr, "TeleopSourceJoystick::handleEvent: invalid axis event number\n");
        return LISTEN_RESULT_ERROR;
      }

      //Set value for this event and signal update
      teleopState->axes[event->number].value = axisDriverValueToTeleopValue(event->value);

      //By default X and Y axes are positive right and down, respectively.  We
      //would like them to be positive up and left.  The axes are switched in
      //the axisDriverTypeToTeleopType() method, and the directions are
      //inverted internally here.  Other axes which have counter-intuitive
      //directions are inverted here as well.
      //
      //Note that these inversions are separate from the axis inversion which
      //is handled by the TeleopSource class.  This is just an internal thing
      //done by this sub-class to make most joysticks act correctly by default.
      //This should save most users a bit of work.
      switch (teleopState->axes[event->number].type) {
        case TELEOP_AXIS_TYPE_LIN_X:
        case TELEOP_AXIS_TYPE_LIN_Y:
        case TELEOP_AXIS_TYPE_HAT0_X:
        case TELEOP_AXIS_TYPE_HAT0_Y:
        case TELEOP_AXIS_TYPE_HAT1_X:
        case TELEOP_AXIS_TYPE_HAT1_Y:
        case TELEOP_AXIS_TYPE_HAT2_X:
        case TELEOP_AXIS_TYPE_HAT2_Y:
        case TELEOP_AXIS_TYPE_HAT3_X:
        case TELEOP_AXIS_TYPE_HAT3_Y:
        case TELEOP_AXIS_TYPE_THROTTLE:
          teleopState->axes[event->number].value *= -1.0;
          break;
        default:
          break;
      }

      return LISTEN_RESULT_CHANGED;

    case JS_EVENT_BUTTON | JS_EVENT_INIT:
      //Ignore init events, they're often wrong
      break;
    case JS_EVENT_BUTTON:
      //Event number shouldn't be bigger than the vector
      if(event->number >= teleopState->buttons.size()) {
        std::fprintf(stderr, "TeleopSourceJoystick::handleEvent: invalid button event number\n");
        return LISTEN_RESULT_ERROR;
      }

      //Set value for this event and signal update
      teleopState->buttons[event->number].value = buttonDriverValueToTeleopValue(event->value);

      return LISTEN_RESULT_CHANGED;
  }

  //If we get here return no change
  return LISTEN_RESULT_UNCHANGED;
}
//=============================================================================
TeleopAxisValue TeleopSourceJoystick::axisDriverValueToTeleopValue(__s16 axisValue) {
  return (TeleopAxisValue)(axisValue)/32767.0;
}
//=============================================================================
TeleopButtonValue TeleopSourceJoystick::buttonDriverValueToTeleopValue(__s16 buttonValue) {
  return (TeleopButtonValue)buttonValue;
}
//=============================================================================
TeleopAxisType TeleopSourceJoystick::axisDriverTypeToTeleopType(__u8 axisType) {
  switch (axisType) {
    //By default X and Y axes are positive right and down, respectively.  We
    //would like them to be positive up and left.  Relevant axes are therefore
    //switched here.  The directions are switched in the handleEvent() method.
    case ABS_X:         return TELEOP_AXIS_TYPE_LIN_Y;
    case ABS_Y:         return TELEOP_AXIS_TYPE_LIN_X;
    case ABS_Z:         return TELEOP_AXIS_TYPE_LIN_Z;
    case ABS_RX:        return TELEOP_AXIS_TYPE_ROT_X;
    case ABS_RY:        return TELEOP_AXIS_TYPE_ROT_Y;
    case ABS_RZ:        return TELEOP_AXIS_TYPE_ROT_Z;
    case ABS_HAT0X:     return TELEOP_AXIS_TYPE_HAT0_Y;
    case ABS_HAT0Y:     return TELEOP_AXIS_TYPE_HAT0_X;
    case ABS_HAT1X:     return TELEOP_AXIS_TYPE_HAT1_Y;
    case ABS_HAT1Y:     return TELEOP_AXIS_TYPE_HAT1_X;
    case ABS_HAT2X:     return TELEOP_AXIS_TYPE_HAT2_Y;
    case ABS_HAT2Y:     return TELEOP_AXIS_TYPE_HAT2_X;
    case ABS_HAT3X:     return TELEOP_AXIS_TYPE_HAT3_Y;
    case ABS_HAT3Y:     return TELEOP_AXIS_TYPE_HAT3_X;
    case ABS_TILT_X:    return TELEOP_AXIS_TYPE_ROT_X;
    case ABS_TILT_Y:    return TELEOP_AXIS_TYPE_ROT_Y;
    case ABS_GAS:
    case ABS_THROTTLE:  return TELEOP_AXIS_TYPE_THROTTLE;
  }
  return TELEOP_AXIS_TYPE_UNKNOWN;
}
//=============================================================================
TeleopButtonType TeleopSourceJoystick::buttonDriverTypeToTeleopType(__u16 buttonType) {
  switch (buttonType) {
    case BTN_0:         return TELEOP_BUTTON_TYPE_0;
    case BTN_1:         return TELEOP_BUTTON_TYPE_1;
    case BTN_2:         return TELEOP_BUTTON_TYPE_2;
    case BTN_3:         return TELEOP_BUTTON_TYPE_3;
    case BTN_4:         return TELEOP_BUTTON_TYPE_4;
    case BTN_5:         return TELEOP_BUTTON_TYPE_5;
    case BTN_6:         return TELEOP_BUTTON_TYPE_6;
    case BTN_7:         return TELEOP_BUTTON_TYPE_7;
    case BTN_8:         return TELEOP_BUTTON_TYPE_8;
    case BTN_9:         return TELEOP_BUTTON_TYPE_9;
    case BTN_A:         return TELEOP_BUTTON_TYPE_A;
    case BTN_B:         return TELEOP_BUTTON_TYPE_B;
    case BTN_C:         return TELEOP_BUTTON_TYPE_C;
    case BTN_X:         return TELEOP_BUTTON_TYPE_X;
    case BTN_Y:         return TELEOP_BUTTON_TYPE_Y;
    case BTN_Z:         return TELEOP_BUTTON_TYPE_Z;
    case BTN_RIGHT:     return TELEOP_BUTTON_TYPE_RIGHT;
    case BTN_LEFT:      return TELEOP_BUTTON_TYPE_LEFT;
    case BTN_SELECT:    return TELEOP_BUTTON_TYPE_SELECT;
    case BTN_START:     return TELEOP_BUTTON_TYPE_START;
    case BTN_TRIGGER:   return TELEOP_BUTTON_TYPE_TRIGGER;
  }
  return TELEOP_BUTTON_TYPE_UNKNOWN;
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
