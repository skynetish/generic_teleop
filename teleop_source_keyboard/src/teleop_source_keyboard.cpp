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
#include <teleop_source_keyboard.hpp>
#include <boost/thread.hpp>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Static member definitions
//=============================================================================
const unsigned int TeleopSourceKeyboard::STEPS_DEFAULT = 3;
const unsigned int TeleopSourceKeyboard::STEPS_MIN     = 1;
const unsigned int TeleopSourceKeyboard::STEPS_MAX     = 5;

const unsigned int TeleopSourceKeyboard::KEYCODE_SPACE = 0x20;
const unsigned int TeleopSourceKeyboard::KEYCODE_UP    = 0x41;
const unsigned int TeleopSourceKeyboard::KEYCODE_DOWN  = 0x42;
const unsigned int TeleopSourceKeyboard::KEYCODE_RIGHT = 0x43;
const unsigned int TeleopSourceKeyboard::KEYCODE_LEFT  = 0x44;




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceKeyboard::TeleopSourceKeyboard() :
  mIsInitialised(false) {
  //Set step size using setter in order to update both steps and step size
  setSteps(STEPS_DEFAULT);
}
//=============================================================================
TeleopSourceKeyboard::~TeleopSourceKeyboard() {
  //Shutdown (should always succeed)
  if (!shutdown()) {
    fprintf(stderr, "TeleopSourceKeyboard::~TeleopSourceKeyboard: warning: ignoring error in shutdown\n");
  }
}
//=============================================================================
bool TeleopSourceKeyboard::setSteps(unsigned int steps) {
  //Sanity check
  if (STEPS_MIN > steps || STEPS_MAX < steps) {
    fprintf(stderr, "TeleopSourceKeyboard::setSteps: error: invalid steps (%u)\n", steps);
    return false;
  }

  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Set steps and step size
  mSteps = steps;
  mStepSize = (TeleopAxisValue)(TELEOP_AXIS_MAX - TELEOP_AXIS_MIN)/(2*mSteps);

  //Return result
  return true;
}
//=============================================================================
unsigned int TeleopSourceKeyboard::getSteps() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Return steps
  return mSteps;
}
//=============================================================================
bool TeleopSourceKeyboard::init() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //If already initialised shutdown first (shutdown should always succeed)
  if (mIsInitialised && !shutdown()) {
    fprintf(stderr, "TeleopSourceKeyboard::init: warning: ignoring error in shutdown\n");
  }

  //Raw termios settings
  struct termios rawTermios;

  //Remember old termios settings for stdin
  tcgetattr(STDIN_FILENO, &mOldTermios);
  memcpy(&rawTermios, &mOldTermios, sizeof(struct termios));

  //Update stdin to use raw mode
  rawTermios.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &rawTermios);

  //Print welcome message
  fprintf(stdout, "\n\nUse arrow keys to move and space to stop.  Press CTRL-C to quit.\n");

  //Note that we're prepared
  mIsInitialised = true;

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceKeyboard::listen(unsigned int listenTimeout, TeleopState* const teleopState, bool* updated) {
  //Set default value
  *updated = false;

  //Sanity check
  if (NULL == teleopState) {
    fprintf(stderr, "TeleopSourceKeyboard::listen: error: NULL teleop state\n");
    return false;
  }

  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Ensure we're initialised
  if (!mIsInitialised) {
    fprintf(stderr, "TeleopSourceKeyboard::listen: error: not initialised\n");
    return false;
  }

  //Ensure state has correct number and types of axes and buttons
  if (2 != teleopState->axes.size()) {
    teleopState->axes.resize(2);
    teleopState->axes[0].type  = TELEOP_AXIS_TYPE_LIN_X;
    teleopState->axes[0].value = 0.0;
    teleopState->axes[1].type  = TELEOP_AXIS_TYPE_LIN_Y;
    teleopState->axes[1].value = 0.0;
  }
  if (0 != teleopState->buttons.size()) {
    teleopState->buttons.clear();
  }

  //Initialise a file descriptor set for select
  fd_set fileDescriptorSet;
  FD_ZERO (&fileDescriptorSet);
  FD_SET (STDIN_FILENO, &fileDescriptorSet);

  //Initialise timeout value for select
  struct timeval timeout;
  timeout.tv_sec = listenTimeout / 1000;
  timeout.tv_usec = (listenTimeout % 1000) * 1000;

  //Use select to see if anything shows up before timeout
  int result = select(FD_SETSIZE, &fileDescriptorSet, NULL, NULL, &timeout);
  if (0 == result) {
    //Timeout
    return true;
  } else if (-1 == result) {
    //Error
    fprintf(stderr, "TeleopSourceKeyboard::listen: error: select() failed (%d)\n", errno);
    return false;
  }

  //Data available, read one event
  char c;
  if(0 >= read(STDIN_FILENO, &c, 1)) {
    fprintf(stderr, "TeleopSourceKeyboard::listen: error: read() failed\n");
    return false;
  }

  //Process event and return result
  return handleEvent(c, teleopState, updated);
}
//=============================================================================
bool TeleopSourceKeyboard::shutdown() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //If we're not prepared we're done.  This means this method can be called
  //multiple times without problems.
  if (!mIsInitialised) {
    return true;
  }

  //Restore termios settings
  tcsetattr(STDIN_FILENO, TCSANOW, &mOldTermios);

  //Note that we're not prepared
  mIsInitialised = false;

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceKeyboard::handleEvent(char c, TeleopState* const teleopState, bool* updated) {
  //Handle known events
  switch(c) {
    case KEYCODE_UP:
      if (teleopState->axes[0].value >= TELEOP_AXIS_MAX) {
        return true;
      }
      teleopState->axes[0].value += mStepSize;
      if (teleopState->axes[0].value > TELEOP_AXIS_MAX) {
        teleopState->axes[0].value = TELEOP_AXIS_MAX;
      }
      *updated = true;
      return true;
    case KEYCODE_DOWN:
      if (teleopState->axes[0].value <= TELEOP_AXIS_MIN) {
        return true;
      }
      teleopState->axes[0].value -= mStepSize;
      if (teleopState->axes[0].value < TELEOP_AXIS_MIN) {
        teleopState->axes[0].value = TELEOP_AXIS_MIN;
      }
      *updated = true;
      return true;
    case KEYCODE_RIGHT:
      if (teleopState->axes[1].value <= TELEOP_AXIS_MIN) {
        return true;
      }
      teleopState->axes[1].value -= mStepSize;
      if (teleopState->axes[1].value < TELEOP_AXIS_MIN) {
        teleopState->axes[1].value = TELEOP_AXIS_MIN;
      }
      *updated = true;
      return true;
    case KEYCODE_LEFT:
      if (teleopState->axes[1].value >= TELEOP_AXIS_MAX) {
        return true;
      }
      teleopState->axes[1].value += mStepSize;
      if (teleopState->axes[1].value > TELEOP_AXIS_MAX) {
        teleopState->axes[1].value = TELEOP_AXIS_MAX;
      }
      *updated = true;
      return true;
    case KEYCODE_SPACE:
      if (0.0 != teleopState->axes[0].value || 0.0 != teleopState->axes[1].value) {
        teleopState->axes[0].value = 0.0;
        teleopState->axes[1].value = 0.0;
        *updated = true;
      }
      return true;
    default:
      //Unknown key, return no change
      return true;
  }
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
