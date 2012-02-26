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
#ifndef INCLUDE_TELEOP_SOURCE_NODE_HPP
#define INCLUDE_TELEOP_SOURCE_NODE_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <teleop_source_adapter.hpp>
#include <teleop_msgs/State.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdint.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================
/**
 * This class creates a teleop source ROS node which reads and publishes the
 * state of a teleop source device.
 *
 * A teleop source adapter object is used to control a teleop source object.
 * This class inherits the teleop source adapter callback interface in order to
 * receive updates about the teleop source state.
 *
 * The init() and shutdown() methods control the life cycle of the object.
 * Note that shutdown() is always called on destruction.
 *
 * Optional parameters for the node are the following:
 *
 *   topic:           topic to which to publish the teleop state
 *   teleop_type:     teleop source type ("auto", "keyboard", or "joystick")
 *   listen_timeout:  teleop source listen timeout in milliseconds
 *   axis_dead_zone:  teleop source initial axis dead zone for all axes
 *   keyboard_steps:  resolution in steps (for keyboard teleop source)
 *   joystick_device: device (for joystick teleop source)
 */
class TeleopSourceNode : public TeleopSourceAdapter::TeleopSourceAdapterCallback {

public:

  //Teleop types
  static const char TELEOP_TYPE_AUTO[];
  static const char TELEOP_TYPE_KEYBOARD[];
  static const char TELEOP_TYPE_JOYSTICK[];

  //Parameter keys
  static const char PARAM_KEY_TELEOP_TOPIC[];
  static const char PARAM_KEY_TELEOP_TYPE[];
  static const char PARAM_KEY_LISTEN_TIMEOUT[];
  static const char PARAM_KEY_AXIS_DEAD_ZONE[];
  static const char PARAM_KEY_KEYBOARD_STEPS[];
  static const char PARAM_KEY_JOYSTICK_DEVICE[];

  //Parameter default values
  static const char   PARAM_DEFAULT_TELEOP_TOPIC[];
  static const char*  PARAM_DEFAULT_TELEOP_TYPE;
  static const int    PARAM_DEFAULT_LISTEN_TIMEOUT;
  static const double PARAM_DEFAULT_AXIS_DEAD_ZONE;
  static const int    PARAM_DEFAULT_KEYBOARD_STEPS;
  static const char   PARAM_DEFAULT_JOYSTICK_DEVICE[];

  /**
   * Constructor.
   */
  TeleopSourceNode();

  /**
   * Destructor.
   */
  ~TeleopSourceNode();

  /**
   * Initialise object.  If object is already initialised it is shutdown and
   * reinitialised.
   *
   *   @param argc - number of command line arguments to process
   *   @param argv - command line arguments
   *   @param nodeName - node name
   *   @param initOptions - init options
   *
   *   @return true on success
   */
  bool init(int argc, char** argv, std::string nodeName, uint32_t initOptions);

  /**
   * Shutdown object.  If object is already shutdown this has no effect.  This
   * method always cleans up as much as possible, even if there are errors.
   * This method is always called on destruction.
   *
   *   @return true on success
   */
  bool shutdown();

private:

  //Parameters
  std::string mTeleopTopic;
  std::string mTeleopType;
  int mListenTimeout;
  double mAxisDeadZone;
  int mKeyboardSteps;
  std::string mJoystickDevice;

  /** Teleop source (dynamically allocated) */
  TeleopSource* mTeleopSource;

  /** Teleop source adapter */
  TeleopSourceAdapter mTeleopSourceAdapter;

  /** Publisher for teleop messages */
  ros::Publisher mPublisher;

  /** Latest message */
  teleop_msgs::State mTeleopStateMsg;

  /** Is initialised flag */
  bool mIsInitialised;

  /** Mutex to protect is initialised flag */
  boost::recursive_mutex mIsInitialisedMutex;

  /**
   * Utility method for creating a teleop source, given the currently set
   * parameters.  The produced teleop source is dynamically allocated.
   *
   *   @return teleop source on success and NULL on error
   */
  TeleopSource* teleopSourceFactory();

  /**
   * Override virtual method from parent.
   */
  virtual void updated(const TeleopState* const teleopState, bool stopping, bool error);

  /**
   * Override virtual method from parent.
   */
  virtual void stopping(bool error);

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_NODE_HPP
//=============================================================================
