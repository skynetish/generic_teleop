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
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <cstdio>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceKeyboard::TeleopSourceKeyboard(TeleopSourceCallback* callback)
  : TeleopSource(callback), mOldTermiosSet(false) {
  //Set step size using setter in order to update both steps and step size
  setSteps(STEPS_DEFAULT);
}
//=============================================================================
TeleopSourceKeyboard::~TeleopSourceKeyboard() {
  //Stop this source
  stop();
}
//=============================================================================
bool TeleopSourceKeyboard::prepareToListen() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Save termios settings only if they were not already saved.  This means this
  //method can be called multiple times without problems.
  if (!mOldTermiosSet) {
    //Raw termios settings
    struct termios rawTermios;

    //Remember old termios settings for stdin
    tcgetattr(STDIN_FILENO, &mOldTermios);
    memcpy(&rawTermios, &mOldTermios, sizeof(struct termios));

    //Update stdin to use raw mode
    rawTermios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &rawTermios);

    //Note that old termios is set and should not be overwritten
    mOldTermiosSet = true;
  }

  //Print welcome message
  std::fprintf(stdout, "\n\nUse arrow keys to move and space to stop.  Press CTRL-C to quit.\n");

  //Return result
  return true;
}
//=============================================================================
TeleopSource::ListenResult TeleopSourceKeyboard::listen(int timeoutMilliseconds,
                                                        TeleopState* const teleopState) {
  //Sanity check
  if (NULL == teleopState) {
    std::fprintf(stderr, "TeleopSourceKeyboard::listen: NULL teleop state\n");
    return LISTEN_RESULT_ERROR;
  }

  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

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
  timeout.tv_sec = timeoutMilliseconds / 1000;
  timeout.tv_usec = (timeoutMilliseconds % 1000) * 1000;

  //Use select to see if anything shows up before timeout
  int result = select(FD_SETSIZE, &fileDescriptorSet, NULL, NULL, &timeout);
  if (0 == result) {
    //Timeout
    return LISTEN_RESULT_UNCHANGED;
  } else if (-1 == result) {
    //Error
    std::fprintf(stderr, "TeleopSourceKeyboard::listen: error in select() (%d)\n", errno);
    return LISTEN_RESULT_ERROR;
  }

  //Data available, read one event
  char c;
  if(0 >= read(STDIN_FILENO, &c, 1)) {
    std::fprintf(stderr, "TeleopSourceKeyboard::listen: error in read()\n");
    return LISTEN_RESULT_ERROR;
  }

  //Process event and return result
  return handleEvent(c, teleopState);
}
//=============================================================================
bool TeleopSourceKeyboard::doneListening() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Restore termios settings only if they were saved.  This means this method
  //can be called multiple times without problems.
  if (mOldTermiosSet) {
    tcsetattr(STDIN_FILENO, TCSANOW, &mOldTermios);
    mOldTermiosSet = false;
  }

  //Return result
  return true;
}
//=============================================================================
TeleopSource::ListenResult TeleopSourceKeyboard::handleEvent(char c,
                                                             TeleopState* const teleopState) {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Handle known events
  switch(c) {
    case KEYCODE_UP:
      if (teleopState->axes[0].value >= TELEOP_AXIS_MAX) {
        return LISTEN_RESULT_UNCHANGED;
      }
      teleopState->axes[0].value += mStepSize;
      if (teleopState->axes[0].value > TELEOP_AXIS_MAX) {
        teleopState->axes[0].value = TELEOP_AXIS_MAX;
      }
      return LISTEN_RESULT_CHANGED;
    case KEYCODE_DOWN:
      if (teleopState->axes[0].value <= TELEOP_AXIS_MIN) {
        return LISTEN_RESULT_UNCHANGED;
      }
      teleopState->axes[0].value -= mStepSize;
      if (teleopState->axes[0].value < TELEOP_AXIS_MIN) {
        teleopState->axes[0].value = TELEOP_AXIS_MIN;
      }
      return LISTEN_RESULT_CHANGED;
    case KEYCODE_RIGHT:
      if (teleopState->axes[1].value <= TELEOP_AXIS_MIN) {
        return LISTEN_RESULT_UNCHANGED;
      }
      teleopState->axes[1].value -= mStepSize;
      if (teleopState->axes[1].value < TELEOP_AXIS_MIN) {
        teleopState->axes[1].value = TELEOP_AXIS_MIN;
      }
      return LISTEN_RESULT_CHANGED;
    case KEYCODE_LEFT:
      if (teleopState->axes[1].value >= TELEOP_AXIS_MAX) {
        return LISTEN_RESULT_UNCHANGED;
      }
      teleopState->axes[1].value += mStepSize;
      if (teleopState->axes[1].value > TELEOP_AXIS_MAX) {
        teleopState->axes[1].value = TELEOP_AXIS_MAX;
      }
      return LISTEN_RESULT_CHANGED;
    case KEYCODE_SPACE:
      teleopState->axes[0].value = 0.0;
      teleopState->axes[1].value = 0.0;
      return LISTEN_RESULT_CHANGED;
  }

  //If we get here return no change
  return LISTEN_RESULT_UNCHANGED;
}
//=============================================================================
bool TeleopSourceKeyboard::setSteps(int steps) {
  //Sanity check
  if (STEPS_MIN > steps || STEPS_MAX < steps) {
    std::fprintf(stderr, "TeleopSourceKeyboard::setSteps: invalid steps (%d)\n", steps);
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
int TeleopSourceKeyboard::getSteps() {
  //Lock access to members
  boost::lock_guard<boost::recursive_mutex> memberLock(mMemberMutex);

  //Return steps
  return mSteps;
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
