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
#include <teleop_source_joystick.hpp>
#include <teleop_msgs/State.h>
#include <ros/ros.h>
#include <signal.h>




//=============================================================================
//Defines
//=============================================================================

/**@{ Parameter keys */
#define PARAM_KEY_TOPIC                 "topic"
#define PARAM_KEY_TYPE                  "type"
#define PARAM_KEY_JOYSTICK_DEVICE       "joystick_device"
#define PARAM_KEY_KEYBOARD_STEPS        "keyboard_steps"
#define PARAM_KEY_AXIS_DEAD_ZONE        "axis_dead_zone"
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TOPIC             "teleop"
#define PARAM_DEFAULT_TYPE              "keyboard"
#define PARAM_DEFAULT_JOYSTICK_DEVICE   teleop::TeleopSourceJoystick::getDefaultDevice()
#define PARAM_DEFAULT_KEYBOARD_STEPS    ((int)teleop::TeleopSourceKeyboard::STEPS_DEFAULT)
#define PARAM_DEFAULT_AXIS_DEAD_ZONE    ((teleop::TeleopAxisValue)teleop::TeleopSource::AXIS_DEAD_ZONE_DEFAULT)
/**@}*/




//=============================================================================
//Globals
//=============================================================================
/** Publisher must be global to allow it to be used in callback */
ros::Publisher* gPublisher = NULL;

/** Teleop source must be global to allow it to be used in signal handler */
teleop::TeleopSource* gTeleopSource = NULL;




//=============================================================================
//Function prototypes
//=============================================================================

/**
 * Interrupt signal handler
 *
 *   @param sig - received signal
 */
void quit(int sig);

/**
 * Callback to report an updated teleop state.  This is called from the teleop
 * source listening thread.
 *
 *   @param teleopState [in] - the latest teleop state
 *   @param stopping [in] - true if thread is stopping
 *   @param error [in] - true if there were errors
 */
void teleopSourceCallback(const teleop::TeleopState* const teleopState,
                          bool stopping,
                          bool error);

/**
 * Utility function for creating appropriate teleop source
 *
 *   @param type - string containing teleop type
 *   @param deadZone - default dead zone
 *   @param keyboardSteps - if this is a keyboard, use this many steps
 *   @param joystickDevice - if this is a joystick, use this device
 *
 *   @return teleop source or NULL on error
 */
teleop::TeleopSource* teleopSourceFactory(std::string type,
                                          double axisDeadZone,
                                          int keyboardSteps,
                                          std::string joystickDevice);




//=============================================================================
//Function definitions
//=============================================================================
void quit(int sig) {
  static bool quitting = false;

  //Make sure this only gets done once
  if (!quitting) {
    quitting = true;

    //Stop and free the teleop source
    if (NULL != gTeleopSource) {
      gTeleopSource->stop();
      delete gTeleopSource;
    }

    //Shutdown ROS to end spinning
    ros::shutdown();
  }
}
//=============================================================================
void teleopSourceCallback(const teleop::TeleopState* const teleopState, bool stopping, bool error) {
  //Sanity check publisher and teleopState
  if (NULL == gPublisher || NULL == teleopState) {
    ROS_ERROR("teleopSourceCallback: NULL publisher or teleopState\n");
    quit(0);
    return;
  }

  //Convert from teleop::TeleopState to teleop_msgs::State.  Use a static here to
  //avoid the need to reallocate the state message and its fields every time.
  static teleop_msgs::State teleopStateMsg;
  if (teleopStateMsg.axes.size() != teleopState->axes.size()) {
    teleopStateMsg.axes.resize(teleopState->axes.size());
  }
  if (teleopStateMsg.buttons.size() != teleopState->buttons.size()) {
    teleopStateMsg.buttons.resize(teleopState->buttons.size());
  }
  for (size_t i = 0; i < teleopState->axes.size(); i++) {
    teleopStateMsg.axes[i].type = teleopState->axes[i].type;
    teleopStateMsg.axes[i].value = teleopState->axes[i].value;
  }
  for (size_t i = 0; i < teleopState->buttons.size(); i++) {
    teleopStateMsg.buttons[i].type = teleopState->buttons[i].type;
    teleopStateMsg.buttons[i].value = teleopState->buttons[i].value;
  }

  //Publish result
  gPublisher->publish(teleopStateMsg);

  //If stopping or error, quit.  Do this after publishing, since the teleop
  //source should zero its output on error and we want to publish this.
  if (error || stopping) {
    quit(0);
  }
}
//=============================================================================
teleop::TeleopSource* teleopSourceFactory(std::string type,
                                          double axisDeadZone,
                                          int keyboardSteps,
                                          std::string joystickDevice) {
  teleop::TeleopSource* teleopSource;
  if (0 == type.compare("")) {
    return NULL;
  } else if (0 == type.compare("keyboard")) {
    teleop::TeleopSourceKeyboard* teleopSourceKeyboard;
    teleopSourceKeyboard = new teleop::TeleopSourceKeyboard(&teleopSourceCallback);
    teleopSourceKeyboard->setSteps(keyboardSteps);
    teleopSource = teleopSourceKeyboard;
  } else if (0 == type.compare("joystick")) {
    teleop::TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new teleop::TeleopSourceJoystick(&teleopSourceCallback, joystickDevice);
    teleopSource = teleopSourceJoystick;
  } else {
    ROS_ERROR("Unknown teleop source type\n");
    return NULL;
  }
  teleopSource->setAxisDeadZoneForAllAxes(axisDeadZone);
  return teleopSource;
}
//=============================================================================




//=============================================================================
//Main
//=============================================================================
int main(int argc, char** argv)
{
  //Initialise ROS (exceptions ignored intentionally)
  ros::init(argc, argv, "teleop_source_ros", ros::init_options::NoSigintHandler);

  //Set interrupt handler
  signal(SIGINT, quit);

  //Node handle uses private namespace (exceptions ignored intentionally)
  ros::NodeHandle nodeHandle("~");

  //Declare parameters
  std::string topic;
  std::string type;
  std::string joystickDevice;
  int keyboardSteps;
  double axisDeadZone;

  //Read parameters and set default values
  nodeHandle.param(PARAM_KEY_TOPIC,           topic,          std::string(PARAM_DEFAULT_TOPIC));
  nodeHandle.param(PARAM_KEY_TYPE,            type,           std::string(PARAM_DEFAULT_TYPE));
  nodeHandle.param(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice, PARAM_DEFAULT_JOYSTICK_DEVICE);
  nodeHandle.param(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps,  PARAM_DEFAULT_KEYBOARD_STEPS);
  nodeHandle.param(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone,   PARAM_DEFAULT_AXIS_DEAD_ZONE);

  //Advertise parameters for introspection
  nodeHandle.setParam(PARAM_KEY_TOPIC,           topic);
  nodeHandle.setParam(PARAM_KEY_TYPE,            type);
  nodeHandle.setParam(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice);
  nodeHandle.setParam(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps);
  nodeHandle.setParam(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone);

  //Advertise publisher with buffer size set to 1 and latching on.  The
  //publisher should basically just always contain the latest teleop state.
  ros::Publisher publisher = nodeHandle.advertise<teleop_msgs::State>(topic, 1, true);

  //Point to the publisher with the global gPublisher
  gPublisher = &publisher;

  //Create teleop source and point to it with the global gTeleopSource
  gTeleopSource = teleopSourceFactory(type, axisDeadZone, keyboardSteps, joystickDevice);
  if (NULL == gTeleopSource) {
    ROS_ERROR("NULL teleop source\n");
    return 1;
  }

  //Start teleop source
  if (gTeleopSource->start()) {
    //Spin until we're done
    ros::spin();
  } else {
    ROS_ERROR("Error starting teleop source\n");
    return 1;
  }

  return 0;
}
