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
#include <boost/thread.hpp>
#include <termios.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/**
 * This class implements a keyboard teleop source.  The arrow keys are used to
 * increment and decrement the linear X and Y axes relative to the previous
 * teleop source state, which is passed into the listen method.  The space bar
 * acts as a stop button.
 *
 * Key presses are detected by reading raw standard input using termios.  If
 * this class is used, other uses of standard input from within the same
 * process should be handled carefully.  This approach should be fairly
 * portable, and it doesn't require a graphical environment or low-level
 * operating system inputs.
 */
class TeleopSourceKeyboard : public TeleopSource {

public:

  /** Default number of steps to reach the maximum level for each axis */
  static const unsigned int STEPS_DEFAULT;
  /** Min number of steps to reach the maximum level for each axis */
  static const unsigned int STEPS_MIN;
  /** Max number of steps to reach the maximum level for each axis */
  static const unsigned int STEPS_MAX;

  /**
   * Constructor.
   */
  TeleopSourceKeyboard();

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
  bool setSteps(unsigned int steps);

  /**
   * Get the number of steps required to reach the maximum value for all axes.
   *
   *   @return steps
   */
  unsigned int getSteps();

  /**
   * Override virtual method from parent.
   */
  virtual bool init();

  /**
   * Override virtual method from parent.
   */
  virtual bool listen(unsigned int listenTimeout, TeleopState* const teleopState, bool* updated);

  /**
   * Override virtual method from parent.
   */
  virtual bool shutdown();

private:

  //Keycodes
  static const unsigned int KEYCODE_SPACE;
  static const unsigned int KEYCODE_UP;
  static const unsigned int KEYCODE_DOWN;
  static const unsigned int KEYCODE_RIGHT;
  static const unsigned int KEYCODE_LEFT;

  /** Mutex for protecting all members from multi-threaded access */
  boost::recursive_mutex mMemberMutex;

  /** Flag indicating if we are prepared to listen */
  bool mIsInitialised;

  /** Number of steps needed to reach max value for each axis */
  unsigned int mSteps;

  /** Size of each step for each axis */
  TeleopAxisValue mStepSize;

  /** Old termios settings */
  struct termios mOldTermios;

  /**
   * Handle a given event.
   *
   *   @param c [in] - character to handle
   *   @param teleopState [in/out] - the current teleop state, to be updated
   *   @param updated [in/out] - true if teleop state was updated
   *
   *   @return true on success
   */
  bool handleEvent(char c, TeleopState* const teleopState, bool* updated);

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_KEYBOARD_HPP
//=============================================================================
