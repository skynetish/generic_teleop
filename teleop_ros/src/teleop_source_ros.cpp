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
#include <csignal>
#include <cstdio>




//=============================================================================
//Defines
//=============================================================================

/**@{ Teleop types */
#define TELEOP_TYPE_JOYSTICK            "joystick"
#define TELEOP_TYPE_KEYBOARD            "keyboard"
/**@}*/

/**@{ Parameter keys */
#define PARAM_KEY_TOPIC                 "topic"
#define PARAM_KEY_TYPE                  "type"
#define PARAM_KEY_LISTEN_TIMEOUT        "listen_timeout"
#define PARAM_KEY_JOYSTICK_DEVICE       "joystick_device"
#define PARAM_KEY_KEYBOARD_STEPS        "keyboard_steps"
#define PARAM_KEY_AXIS_DEAD_ZONE        "axis_dead_zone"
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TOPIC             "teleop"
#define PARAM_DEFAULT_TYPE              TELEOP_TYPE_KEYBOARD
#define PARAM_DEFAULT_LISTEN_TIMEOUT \
    ((int)teleop::TeleopSource::LISTEN_TIMEOUT_DEFAULT)
#define PARAM_DEFAULT_JOYSTICK_DEVICE \
    ((std::string)teleop::TeleopSourceJoystick::DEFAULT_DEVICE)
#define PARAM_DEFAULT_KEYBOARD_STEPS \
    ((int)teleop::TeleopSourceKeyboard::STEPS_DEFAULT)
#define PARAM_DEFAULT_AXIS_DEAD_ZONE \
    ((teleop::TeleopAxisValue)teleop::TeleopSource::AXIS_DEAD_ZONE_DEFAULT)
/**@}*/




//=============================================================================
//Types
//=============================================================================

/**
 * Instance of the teleop source callback class used by this node.  A publisher
 * must be provided to the constructor.
 */
class TeleopSourceCallbackRos : public teleop::TeleopSource::TeleopSourceCallback {

public:

  /**
   * Constructor.
   */
  TeleopSourceCallbackRos(const ros::Publisher* const publisher);

  /**
   * Override virtual method from parent.
   */
  void updated(const teleop::TeleopState* const teleopState, bool stopping, bool error);

  /**
   * Override virtual method from parent.
   */
  void stopping(bool error);

private:

  /** Publisher given to constructor and used in updated() */
  const ros::Publisher* const mPublisher;

  /** Teleop message member avoids re-creation for each call to updated() */
  teleop_msgs::State mTeleopStateMsg;

}; //class




//=============================================================================
//Function prototypes
//=============================================================================

/**
 * Signal handler
 *
 *   @param signalNumber - received signal number
 */
void signalHandler(int signalNumber);

/**
 * Clean up and shutdown.
 */
void quit();

/**
 * Utility function for creating a teleop source
 *
 *   @param teleopSourceCallback - callback object to use
 *   @param type - teleop type
 *   @param listenTimeout - listen timeout value in milliseconds
 *   @param axisDeadZone - default dead zone
 *   @param keyboardSteps - if this is a keyboard, use this many steps
 *   @param joystickDevice - if this is a joystick, use this device
 *
 *   @return teleop source or NULL on error
 */
teleop::TeleopSource* teleopSourceFactory(teleop::TeleopSource::TeleopSourceCallback* callback,
                                          std::string type,
                                          int listenTimeout,
                                          double axisDeadZone,
                                          int keyboardSteps,
                                          std::string joystickDevice);

/**
 * Check if we should just print usage information, and if so, print it.
 *
 *   @param argc - number of command line arguments
 *   @param argv - command line arguments
 *
 *   @return true if usage was printed
 */
bool printUsage(int argc, char** argv);




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceCallbackRos::TeleopSourceCallbackRos(const ros::Publisher* const publisher) :
  mPublisher(publisher) {
}
//=============================================================================
void TeleopSourceCallbackRos::updated(const teleop::TeleopState* const teleopState,
                                      bool stopping,
                                      bool error) {
  //Sanity check publisher
  if (NULL == mPublisher) {
    ROS_ERROR("updated: NULL publisher");
    return;
  }

  //Sanity check teleopState
  if (NULL == teleopState) {
    ROS_ERROR("updated: NULL teleopState");
    return;
  }

  //Convert from teleop::TeleopState to teleop_msgs::State
  if (mTeleopStateMsg.axes.size() != teleopState->axes.size()) {
    mTeleopStateMsg.axes.resize(teleopState->axes.size());
  }
  if (mTeleopStateMsg.buttons.size() != teleopState->buttons.size()) {
    mTeleopStateMsg.buttons.resize(teleopState->buttons.size());
  }
  for (size_t i = 0; i < teleopState->axes.size(); i++) {
    mTeleopStateMsg.axes[i].type = teleopState->axes[i].type;
    mTeleopStateMsg.axes[i].value = teleopState->axes[i].value;
  }
  for (size_t i = 0; i < teleopState->buttons.size(); i++) {
    mTeleopStateMsg.buttons[i].type = teleopState->buttons[i].type;
    mTeleopStateMsg.buttons[i].value = teleopState->buttons[i].value;
  }

  //Publish result
  mPublisher->publish(mTeleopStateMsg);

  //On error just print a message.
  if (error) {
    ROS_ERROR("updated: error detected");
  }

  //On stopping just print a message
  if (stopping) {
    ROS_INFO("updated: stopping");
  }
}
//=============================================================================
void TeleopSourceCallbackRos::stopping(bool error) {
  ROS_INFO("stopping: stopping");
  quit();
}
//=============================================================================




//=============================================================================
//Function definitions
//=============================================================================
void signalHandler(int signalNumber) {
  quit();
}
//=============================================================================
void quit() {
  //Shutdown ROS to end spinning
  ros::shutdown();
}
//=============================================================================
teleop::TeleopSource* teleopSourceFactory(teleop::TeleopSource::TeleopSourceCallback* callback,
                                          std::string type,
                                          int listenTimeout,
                                          double axisDeadZone,
                                          int keyboardSteps,
                                          std::string joystickDevice) {
  teleop::TeleopSource* teleopSource;
  if (0 == type.compare("")) {
    return NULL;
  } else if (0 == type.compare(TELEOP_TYPE_KEYBOARD)) {
    teleop::TeleopSourceKeyboard* teleopSourceKeyboard;
    teleopSourceKeyboard = new teleop::TeleopSourceKeyboard(callback);
    teleopSourceKeyboard->setSteps(keyboardSteps);
    teleopSource = teleopSourceKeyboard;
  } else if (0 == type.compare(TELEOP_TYPE_JOYSTICK)) {
    teleop::TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new teleop::TeleopSourceJoystick(callback, joystickDevice);
    teleopSource = teleopSourceJoystick;
  } else {
    ROS_ERROR("teleopSourceFactory: unknown teleop source type");
    return NULL;
  }
  teleopSource->setListenTimeout(listenTimeout);
  teleopSource->setAxisDeadZoneForAllAxes(axisDeadZone);
  return teleopSource;
}
//=============================================================================
bool printUsage(int argc, char** argv) {
  //Check for "-h" or "--help", if found, print usage
  for (int i = 1; i < argc; i++) {
    if ((0 == strcmp(argv[i], "-h")) || (0 == strcmp(argv[i], "--help"))) {
      std::printf("\n");
      std::printf("Usage:\n");
      std::printf("    %s [params]\n", basename(argv[0]));
      std::printf("\n");
      std::printf("Parameters and their default values\n");
      std::printf("    _%s:=%s\n",    PARAM_KEY_TYPE,            PARAM_DEFAULT_TYPE);
      std::printf("    _%s:=%s\n",    PARAM_KEY_TOPIC,           PARAM_DEFAULT_TOPIC);
      std::printf("    _%s:=%d\n",    PARAM_KEY_LISTEN_TIMEOUT,  PARAM_DEFAULT_LISTEN_TIMEOUT);
      std::printf("    _%s:=%.02f\n", PARAM_KEY_AXIS_DEAD_ZONE,  PARAM_DEFAULT_AXIS_DEAD_ZONE);
      std::printf("    _%s:=%d\n",    PARAM_KEY_KEYBOARD_STEPS,  PARAM_DEFAULT_KEYBOARD_STEPS);
      std::printf("    _%s:=%s\n",    PARAM_KEY_JOYSTICK_DEVICE, (PARAM_DEFAULT_JOYSTICK_DEVICE).c_str());
      std::printf("\n");
      return true;
    }
  }
  return false;
}
//=============================================================================




//=============================================================================
//Main
//=============================================================================
int main(int argc, char** argv)
{
  //Check if we should just print usage information and quit
  if (printUsage(argc, argv)) {
    return 0;
  }

  //Initialise ROS (exceptions ignored intentionally)
  ros::init(argc, argv, basename(argv[0]), ros::init_options::NoSigintHandler);

  //Set signal handler
  signal(SIGINT, signalHandler);

  //Node handle uses private namespace (exceptions ignored intentionally)
  ros::NodeHandle nodeHandle("~");

  //Declare parameters
  std::string topic;
  std::string type;
  int listenTimeout;
  std::string joystickDevice;
  int keyboardSteps;
  double axisDeadZone;

  //Read parameters and/or set default values
  nodeHandle.param(PARAM_KEY_TOPIC,           topic,          std::string(PARAM_DEFAULT_TOPIC));
  nodeHandle.param(PARAM_KEY_TYPE,            type,           std::string(PARAM_DEFAULT_TYPE));
  nodeHandle.param(PARAM_KEY_LISTEN_TIMEOUT,  listenTimeout,  PARAM_DEFAULT_LISTEN_TIMEOUT);
  nodeHandle.param(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice, PARAM_DEFAULT_JOYSTICK_DEVICE);
  nodeHandle.param(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps,  PARAM_DEFAULT_KEYBOARD_STEPS);
  nodeHandle.param(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone,   PARAM_DEFAULT_AXIS_DEAD_ZONE);

  //Advertise all parameters for introspection
  nodeHandle.setParam(PARAM_KEY_TOPIC,           topic);
  nodeHandle.setParam(PARAM_KEY_TYPE,            type);
  nodeHandle.setParam(PARAM_KEY_LISTEN_TIMEOUT,  listenTimeout);
  nodeHandle.setParam(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice);
  nodeHandle.setParam(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps);
  nodeHandle.setParam(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone);

  //Create publisher with buffer size set to 1 and latching on.  The publisher
  //should basically just always contain the latest teleop state.
  ros::Publisher publisher = nodeHandle.advertise<teleop_msgs::State>(topic, 1, true);

  //Create callback using publisher
  TeleopSourceCallbackRos callback(&publisher);

  //Create teleop source using callback
  teleop::TeleopSource* teleopSource = teleopSourceFactory(&callback,
                                                           type,
                                                           listenTimeout,
                                                           axisDeadZone,
                                                           keyboardSteps,
                                                           joystickDevice);
  if (NULL == teleopSource) {
    ROS_ERROR("main: NULL teleop source");
    return 1;
  }

  //Start teleop source
  if (!teleopSource->start()) {
    ROS_ERROR("main: error starting teleop source");
    return 1;
  }

  //Spin until shutdown is called in quit()
  ros::spin();

  //Stop and free teleop source
  teleopSource->stop();
  teleopSource->waitForStopped();
  delete teleopSource;

  //Done
  return 0;
}
