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
#ifndef INCLUDE_TELEOP_SOURCE_KEYBOARD_HPP
#define INCLUDE_TELEOP_SOURCE_KEYBOARD_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <termios.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/**
 * This class implements a keyboard teleop source.  The arrow keys are used as
 * linear X and linear Y axes, and the space bar acts as a stop button.  The
 * key presses are detected by reading raw standard input using termios.  If
 * this class is used, other uses of standard input from within the same
 * process should be handled carefully.
 *
 * The default axis dead zone should normally work fine.  Note that the
 * keyboard source increments and decrements the previous teleop source output.
 * For this reason, care should be taken to ensure that the axis dead zone
 * (which is set via the TeleopSource class) is not greater than the step size.
 * Otherwise this source will always output zero axis values.
 *
 * Currently raw standard input is read and processed to identify key presses.
 * An alternative approach could be to detect low-level key press and release
 * events, but this would probably require either access to the X server (which
 * we shouldn't need for a keyboard teleop device), or access to linux inputs
 * in the /dev/input/event* files (which normally requires elevated
 * privileges).
 */
class TeleopSourceKeyboard : public TeleopSource {

public:

  /**@{ Number of steps to reach the maximum level for each axis */
  static const int STEPS_DEFAULT = 5;
  static const int STEPS_MIN = 1;
  static const int STEPS_MAX = 10;
  /**@}*/

  /**
   * Constructor.
   *
   *   @param callback [in] - callback to use to report status
   */
  TeleopSourceKeyboard(TeleopSourceCallback* callback);

  /**
   * Destructor.
   */
  ~TeleopSourceKeyboard();

  /**
   * Set the number of steps required to reach the maximum value for all axes.
   *
   *   @param steps [in] - number of steps
   *
   *   @return true on success
   */
  bool setSteps(int steps);

  /**
   * Get steps.
   *
   *   @return steps
   */
  int getSteps();

private:

  /**@{ Keycodes */
  static const int KEYCODE_SPACE = 0x20;
  static const int KEYCODE_UP    = 0x41;
  static const int KEYCODE_DOWN  = 0x42;
  static const int KEYCODE_RIGHT = 0x43;
  static const int KEYCODE_LEFT  = 0x44;
  /**@}*/

  /** Mutex for protecting all members from multi-threaded access */
  boost::recursive_mutex mMemberMutex;

  /** Number of steps needed to reach max value for each axis */
  int mSteps;

  /** Size of each step for each axis */
  TeleopAxisValue mStepSize;

  /** Old termios settings */
  struct termios mOldTermios;

  /** Old termios settings are set */
  bool mOldTermiosSet;

  /**
   * Handle a given event.
   *
   *   @param c [in] - character to handle
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return LISTEN_ERROR on error, LISTEN_STATE_UNCHANGED on timeout or no
   *           change to state, LISTEN_STATE_CHANGED if state updated
   */
  ListenResult handleEvent(char c, TeleopState* const teleopState);

  /**
   * Override virtual method from parent.
   */
  bool prepareToListen();

  /**
   * Override virtual method from parent.
   */
  ListenResult listen(int timeoutMilliseconds, TeleopState* const teleop);

  /**
   * Override virtual method from parent.
   */
  bool doneListening();

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_KEYBOARD_HPP
//=============================================================================
