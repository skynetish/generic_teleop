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
#include <teleop_source_node.hpp>
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <teleop_source_adapter.hpp>
#include <teleop_source_keyboard.hpp>
#include <teleop_source_joystick.hpp>
#include <teleop_msgs/State.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdint.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Static member definitions
//=============================================================================
const char TeleopSourceNode::TELEOP_TYPE_AUTO[] =               "auto";
const char TeleopSourceNode::TELEOP_TYPE_KEYBOARD[] =           "keyboard";
const char TeleopSourceNode::TELEOP_TYPE_JOYSTICK[] =           "joystick";

const char TeleopSourceNode::PARAM_KEY_TELEOP_TOPIC[] =         "teleop_topic";
const char TeleopSourceNode::PARAM_KEY_TELEOP_TYPE[] =          "teleop_type";
const char TeleopSourceNode::PARAM_KEY_LISTEN_TIMEOUT[] =       "listen_timeout";
const char TeleopSourceNode::PARAM_KEY_AXIS_DEAD_ZONE[] =       "axis_dead_zone";
const char TeleopSourceNode::PARAM_KEY_KEYBOARD_STEPS[] =       "keyboard_steps";
const char TeleopSourceNode::PARAM_KEY_JOYSTICK_DEVICE[] =      "joystick_device";

const char TeleopSourceNode::PARAM_DEFAULT_TELEOP_TOPIC[] =     "teleop";
const char* TeleopSourceNode::PARAM_DEFAULT_TELEOP_TYPE =       TeleopSourceNode::TELEOP_TYPE_KEYBOARD;
const int  TeleopSourceNode::PARAM_DEFAULT_LISTEN_TIMEOUT =     TeleopSourceAdapter::LISTEN_TIMEOUT_DEFAULT;
const double TeleopSourceNode::PARAM_DEFAULT_AXIS_DEAD_ZONE =   TeleopSourceAdapter::AXIS_DEAD_ZONE_DEFAULT;
const int  TeleopSourceNode::PARAM_DEFAULT_KEYBOARD_STEPS =     TeleopSourceKeyboard::STEPS_DEFAULT;

//Ideally this should be TeleopSourceJoystick::getDefaultDevice().c_str(),
//but C++ makes this difficult.  Since we assume the user should set this
//parameter explicitly if needed anyway, we use a hard-coded default.
const char TeleopSourceNode::PARAM_DEFAULT_JOYSTICK_DEVICE[] =  "/dev/input/js0";




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceNode::TeleopSourceNode() :
  mTeleopTopic(PARAM_DEFAULT_TELEOP_TOPIC),
  mTeleopType(PARAM_DEFAULT_TELEOP_TYPE),
  mListenTimeout(PARAM_DEFAULT_LISTEN_TIMEOUT),
  mAxisDeadZone(PARAM_DEFAULT_AXIS_DEAD_ZONE),
  mKeyboardSteps(PARAM_DEFAULT_KEYBOARD_STEPS),
  mJoystickDevice(PARAM_DEFAULT_JOYSTICK_DEVICE),
  mTeleopSource(NULL),
  mIsInitialised(false) {
}
//=============================================================================
TeleopSourceNode::~TeleopSourceNode() {
  //Shutdown (should always succeed)
  if (!shutdown()) {
    ROS_WARN("TeleopSourceNode::~TeleopSourceNode: ignoring error in shutdown");
  }
}
//=============================================================================
bool TeleopSourceNode::init(int argc, char** argv, std::string nodeName, uint32_t rosInitOptions) {
  //Lock access to is initialised
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //If already initialised shutdown first (shutdown should always succeed)
  if (mIsInitialised && !shutdown()) {
    ROS_WARN("TeleopSourceNode::init: ignoring error in shutdown()");
  }

  //Init node
  try {
    ros::init(argc, argv, nodeName, rosInitOptions);
  } catch(ros::InvalidNodeNameException& e) {
    ROS_ERROR("TeleopSourceNode::init: error initialising node");
    return false;
  }

  //Start node manually to avoid node shutdown when last node handle is destroyed
  ros::start();

  //Init parameters
  try {
    //Create private node handle for parameters
    ros::NodeHandle nodeHandlePrivate("~");

    //Read parameters and/or set default values
    nodeHandlePrivate.param(PARAM_KEY_TELEOP_TOPIC,    mTeleopTopic, nodeName + "/" + PARAM_DEFAULT_TELEOP_TOPIC);
    nodeHandlePrivate.param(PARAM_KEY_TELEOP_TYPE,     mTeleopType,     std::string(PARAM_DEFAULT_TELEOP_TYPE));
    nodeHandlePrivate.param(PARAM_KEY_LISTEN_TIMEOUT,  mListenTimeout,  PARAM_DEFAULT_LISTEN_TIMEOUT);
    nodeHandlePrivate.param(PARAM_KEY_AXIS_DEAD_ZONE,  mAxisDeadZone,   PARAM_DEFAULT_AXIS_DEAD_ZONE);
    nodeHandlePrivate.param(PARAM_KEY_KEYBOARD_STEPS,  mKeyboardSteps,  PARAM_DEFAULT_KEYBOARD_STEPS);
    nodeHandlePrivate.param(PARAM_KEY_JOYSTICK_DEVICE, mJoystickDevice, std::string(PARAM_DEFAULT_JOYSTICK_DEVICE));

    //Advertise all parameters to allow introspection
    nodeHandlePrivate.setParam(PARAM_KEY_TELEOP_TOPIC,    mTeleopTopic);
    nodeHandlePrivate.setParam(PARAM_KEY_TELEOP_TYPE,     mTeleopType);
    nodeHandlePrivate.setParam(PARAM_KEY_LISTEN_TIMEOUT,  mListenTimeout);
    nodeHandlePrivate.setParam(PARAM_KEY_AXIS_DEAD_ZONE,  mAxisDeadZone);
    nodeHandlePrivate.setParam(PARAM_KEY_KEYBOARD_STEPS,  mKeyboardSteps);
    nodeHandlePrivate.setParam(PARAM_KEY_JOYSTICK_DEVICE, mJoystickDevice);
  } catch (ros::InvalidNameException& e) {
    ROS_ERROR("TeleopSourceNode::init: error initialising parameters");
    ros::shutdown();
    return false;
  }

  //Create publisher
  try {
    //Create relative node handle for topics
    ros::NodeHandle nodeHandleRelative = ros::NodeHandle();

    //Create publisher with buffer size set to 1 and latching on.  The publisher
    //should basically just always contain the latest state.
    mPublisher = nodeHandleRelative.advertise<teleop_msgs::State>(mTeleopTopic, 1, true);
  } catch (ros::InvalidNameException& e) {
    ROS_ERROR("TeleopSourceNode::init: error creating publisher");
    ros::shutdown();
    return false;
  }

  //Create teleop source
  mTeleopSource = teleopSourceFactory();
  if (NULL == mTeleopSource) {
    ROS_ERROR("TeleopSourceNode::init: error creating teleop source");
    ros::shutdown();
    return false;
  }

  //Initialise and configure teleop source adapter
  if (!mTeleopSourceAdapter.init(mTeleopSource, this)) {
    ROS_ERROR("TeleopSourceNode::init: error initialising teleop source adapter");
    delete mTeleopSource;
    ros::shutdown();
    return false;
  }
  if (!mTeleopSourceAdapter.setListenTimeout((unsigned int)mListenTimeout)
      || !mTeleopSourceAdapter.setAxisDeadZoneForAllAxes((TeleopAxisValue)mAxisDeadZone)) {
    ROS_ERROR("TeleopSourceNode::init: error configuring teleop source adapter");
    delete mTeleopSource;
    ros::shutdown();
    return false;
  }

  //Start teleop source adapter
  if (!mTeleopSourceAdapter.start(false)) {
    ROS_ERROR("TeleopSourceNode::init: error starting teleop source");
    delete mTeleopSource;
    ros::shutdown();
    return false;
  }

  //Update init done flag
  mIsInitialised = true;

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceNode::shutdown() {
  //Lock access to is initialised
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //Check if done
  if (!mIsInitialised) {
    return true;
  }

  //Shutdown teleop source adapter (will stop if necessary)
  if (!mTeleopSourceAdapter.shutdown()) {
    ROS_WARN("TeleopSourceNode::shutdown: ignoring error in teleop source adapter shutdown");
  }

  //Free teleop source
  if (NULL != mTeleopSource) {
    delete mTeleopSource;
    mTeleopSource = NULL;
  } else {
    ROS_WARN("TeleopSourceNode::shutdown: teleop source already deleted");
  }

  //Zero message
  for (size_t i = 0; i < mTeleopStateMsg.axes.size(); i++) {
    mTeleopStateMsg.axes[i].value = 0.0;
  }
  for (size_t i = 0; i < mTeleopStateMsg.buttons.size(); i++) {
    mTeleopStateMsg.buttons[i].value = 0;
  }

  //Publish zero message
  mPublisher.publish(mTeleopStateMsg);

  //Sleep a bit to allow zero message to be published before node is shutdown.
  //Use boost thread sleep rather than ROS sleep since ROS sleep may try to use
  //a simulated clock, which may be stopped.
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  //Shutdown node
  ros::shutdown();

  //Set init done flag
  mIsInitialised = false;

  //Return result
  return true;
}
//=============================================================================
TeleopSource* TeleopSourceNode::teleopSourceFactory() {
  TeleopSource* teleopSource = NULL;
  std::string teleopType = mTeleopType;

  //If type is auto, check for best alternative
  if (0 == teleopType.compare(std::string(TELEOP_TYPE_AUTO))) {
    TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new TeleopSourceJoystick();
    teleopSourceJoystick->setDevice(mJoystickDevice);
    if (teleopSourceJoystick->init()) {
      if (!teleopSourceJoystick->shutdown()) {
        ROS_WARN("TeleopSourceNode::teleopSourceFactory: ignoring error during TeleopSourceJoystick shutdown");
      }
      teleopType = std::string(TELEOP_TYPE_JOYSTICK);
    } else {
      teleopType = std::string(TELEOP_TYPE_KEYBOARD);
    }
    delete teleopSourceJoystick;
  }

  //Handle known types
  if (0 == teleopType.compare(std::string(TELEOP_TYPE_KEYBOARD))) {
    TeleopSourceKeyboard* teleopSourceKeyboard;
    teleopSourceKeyboard = new TeleopSourceKeyboard();
    if (!teleopSourceKeyboard->setSteps((unsigned int)mKeyboardSteps)) {
      ROS_WARN("TeleopSourceNode::teleopSourceFactory: unable to set keyboard steps");
    }
    teleopSource = teleopSourceKeyboard;
  } else if (0 == teleopType.compare(std::string(TELEOP_TYPE_JOYSTICK))) {
    TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new TeleopSourceJoystick();
    if (!teleopSourceJoystick->setDevice(mJoystickDevice)) {
      ROS_WARN("TeleopSourceNode::teleopSourceFactory: unable to set joystick device");
    }
    teleopSource = teleopSourceJoystick;
  } else {
    ROS_ERROR("TeleopSourceNode::teleopSourceFactory: unknown teleop source type (%s)", teleopType.c_str());
    return NULL;
  }

  //Return result
  return teleopSource;
}
//=============================================================================
void TeleopSourceNode::updated(const TeleopState* const teleopState, bool stopping, bool error) {
  //Sanity check teleopState
  if (NULL == teleopState) {
    ROS_ERROR("TeleopSourceNode::updated: NULL teleopState");
    return;
  }

  //Convert from TeleopState to teleop_msgs::State
  if (mTeleopStateMsg.axes.size() != teleopState->axes.size()) {
    mTeleopStateMsg.axes.resize(teleopState->axes.size());
  }
  if (mTeleopStateMsg.buttons.size() != teleopState->buttons.size()) {
    mTeleopStateMsg.buttons.resize(teleopState->buttons.size());
  }
  for (size_t i = 0; i < mTeleopStateMsg.axes.size(); i++) {
    mTeleopStateMsg.axes[i].type = teleopState->axes[i].type;
    mTeleopStateMsg.axes[i].value = teleopState->axes[i].value;
  }
  for (size_t i = 0; i < mTeleopStateMsg.buttons.size(); i++) {
    mTeleopStateMsg.buttons[i].type = teleopState->buttons[i].type;
    mTeleopStateMsg.buttons[i].value = teleopState->buttons[i].value;
  }

  //Publish result
  mPublisher.publish(mTeleopStateMsg);

  //On error just print a message.  The stopping condition will be handled in
  //the stopping() callback, so we can safely ignore it here.
  if (error) {
    ROS_ERROR("TeleopSourceNode::updated: error detected");
  }
}
//=============================================================================
void TeleopSourceNode::stopping(bool error) {
  //On error just print a message.
  if (error) {
    ROS_ERROR("TeleopSourceNode::stopping: error detected");
  }
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
