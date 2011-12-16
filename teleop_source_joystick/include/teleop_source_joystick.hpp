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
#ifndef INCLUDE_TELEOP_SOURCE_JOYSTICK_HPP
#define INCLUDE_TELEOP_SOURCE_JOYSTICK_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <boost/thread.hpp>
#include <linux/types.h>
#include <linux/joystick.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/**
 * This class implements a joystick teleop source using the Linux kernel
 * joystick driver.
 */
class TeleopSourceJoystick : public TeleopSource {

public:

  /**
   * Constructor.
   */
  TeleopSourceJoystick();

  /**
   * Destructor.
   */
  ~TeleopSourceJoystick();

  /**
   * Set device.  Should only be called before init() or after shutdown().
   *
   *   @param device - device to set
   *
   *   @return true on success
   */
  bool setDevice(std::string device);

  /**
   * Get device.
   *
   *   @return device
   */
  std::string getDevice();

  /**
   * Get joystick name.
   *
   *   @return joystick name
   */
  std::string getJoystickName();

  /**
   * Get default device.
   *
   *   @return default device
   */
  static std::string getDefaultDevice();

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

  /** Mutex for protecting all members from multi-threaded access */
  boost::recursive_mutex mMemberMutex;

  /** Flag indicating if we are prepared to listen */
  bool mIsInitialised;

  /** Device */
  std::string mDevice;

  /** Joystick name */
  std::string mJoystickName;

  /** File descriptor*/
  int mFileDescriptor;

  /** Number of axes from driver */
  __u8 mNumAxes;

  /** Axis type map from driver */
  __u8 mAxisMap[ABS_CNT];

  /** Number of buttons from driver */
  __u8 mNumButtons;

  /** Button type map from driver */
  __u16 mButtonMap[KEY_MAX - BTN_MISC + 1];

  /**
   * Handle a given event.
   *
   *   @param event [in] - event to handle
   *   @param teleopState [in/out] - the current teleop state, to be updated
   *   @param updated [in/out] - true if teleop state was updated
   *
   *   @return true on success
   */
  bool handleEvent(const js_event* const event, TeleopState* const teleopState, bool* updated);

  /**
   * Convert driver axis value to teleop axis value.
   *
   *   @param axisValue [in] - driver axis value to convert
   *
   *   @return teleop axis value
   */
  static TeleopAxisValue axisDriverValueToTeleopValue(__s16 axisValue);

  /**
   * Convert driver button value to teleop button value.
   *
   *   @param buttonValue [in] - driver button value to convert
   *
   *   @return teleop button value
   */
  static TeleopButtonValue buttonDriverValueToTeleopValue(__s16 buttonValue);

  /**
   * Convert driver axis type to teleop axis type.
   *
   *   @param axisType [in] - driver axis type to convert
   *
   *   @return teleop axis type
   */
  static TeleopAxisType axisDriverTypeToTeleopType(__u8 axisType);

  /**
   * Convert driver button type to teleop button type.
   *
   *   @param buttonType [in] - driver button type to convert
   *
   *   @return teleop button type
   */
  static TeleopButtonType buttonDriverTypeToTeleopType(__u16 buttonType);

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_JOYSTICK_HPP
//=============================================================================
