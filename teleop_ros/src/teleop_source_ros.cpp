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
#include <boost/thread.hpp>
#include <csignal>
#include <cstdio>




//=============================================================================
//Defines
//=============================================================================

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
#define PARAM_DEFAULT_TYPE              "keyboard"
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
 * Instance of the teleop source callback class used by this node.
 */
class TeleopSourceCallbackROS : public teleop::TeleopSource::TeleopSourceCallback {

public:

  /**
   * Constructor.
   */
  TeleopSourceCallbackROS(const ros::Publisher* const publisher);

  /**
   * Override virtual method from parent.
   */
  void callback(const teleop::TeleopState* const teleopState, bool stopping, bool error);

private:

  /** Publisher given to constructor and used in callback */
  const ros::Publisher* const mPublisher;

  /** Teleop message is a member to avoid re-creation for each callback call */
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
 *
 *   @param teleopSource - teleop source to shutdown
 */
void quit(teleop::TeleopSource* teleopSource);

/**
 * Utility function for creating appropriate teleop source
 *
 *   @param teleopSourceCallback - callback object to use
 *   @param type - teleop type
 *   @param listenTimeout - listen timeout value used by teleop source
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
 * Print usage information.
 */
void printUsage(int argc, char** argv);




//=============================================================================
//Globals
//=============================================================================

/** Teleop source must be accessible from signal handler */
teleop::TeleopSource* gTeleopSource = NULL;




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceCallbackROS::TeleopSourceCallbackROS(const ros::Publisher* const publisher)
: mPublisher(publisher) {
}
//=============================================================================
void TeleopSourceCallbackROS::callback(const teleop::TeleopState* const teleopState,
                                       bool stopping,
                                       bool error) {
  //Sanity check publisher
  if (NULL == mPublisher) {
    ROS_ERROR("teleopSourceCallback: NULL publisher\n");
    return;
  }

  //Sanity check teleopState
  if (NULL == teleopState) {
    ROS_ERROR("teleopSourceCallback: NULL teleopState\n");
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

  //On error just print a message.  The listening thread stopping is detected
  //elsewhere.
  if (error) {
    ROS_ERROR("teleopSourceCallback: listening thread error\n");
  }
}
//=============================================================================




//=============================================================================
//Function definitions
//=============================================================================
void signalHandler(int signalNumber) {
  quit(gTeleopSource);
}
//=============================================================================
void quit(teleop::TeleopSource* teleopSource) {
  //Use a static mutex to ensure we only delete the teleop source once
  static boost::mutex signalMutex;
  boost::unique_lock<boost::mutex> signalMutexLock(signalMutex, boost::try_to_lock);
  if (!signalMutexLock.owns_lock()) {
    return;
  }

  //Free teleop source (mutex and NULL check ensure this only happens once)
  if (NULL != teleopSource) {
    teleopSource->stop();
    delete teleopSource;
    teleopSource = NULL;
  }

  //Shutdown ROS to end spinning (OK if this is done more than once)
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
  } else if (0 == type.compare("keyboard")) {
    teleop::TeleopSourceKeyboard* teleopSourceKeyboard;
    teleopSourceKeyboard = new teleop::TeleopSourceKeyboard(callback);
    teleopSourceKeyboard->setSteps(keyboardSteps);
    teleopSource = teleopSourceKeyboard;
  } else if (0 == type.compare("joystick")) {
    teleop::TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new teleop::TeleopSourceJoystick(callback, joystickDevice);
    teleopSource = teleopSourceJoystick;
  } else {
    ROS_ERROR("Unknown teleop source type\n");
    return NULL;
  }
  teleopSource->setListenTimeout(listenTimeout);
  teleopSource->setAxisDeadZoneForAllAxes(axisDeadZone);
  return teleopSource;
}
//=============================================================================
void printUsage(int argc, char** argv) {
  printf("\n");
  printf("Usage:\n");
  printf("    %s [params]\n", basename(argv[0]));
  printf("\n");
  printf("Parameters and their default values\n");
  printf("    _%s:=%s\n",    PARAM_KEY_TYPE,            PARAM_DEFAULT_TYPE);
  printf("    _%s:=%s\n",    PARAM_KEY_TOPIC,           PARAM_DEFAULT_TOPIC);
  printf("    _%s:=%d\n",    PARAM_KEY_LISTEN_TIMEOUT,  PARAM_DEFAULT_LISTEN_TIMEOUT);
  printf("    _%s:=%.02f\n", PARAM_KEY_AXIS_DEAD_ZONE,  PARAM_DEFAULT_AXIS_DEAD_ZONE);
  printf("    _%s:=%d\n",    PARAM_KEY_KEYBOARD_STEPS,  PARAM_DEFAULT_KEYBOARD_STEPS);
  printf("    _%s:=%s\n",    PARAM_KEY_JOYSTICK_DEVICE, (PARAM_DEFAULT_JOYSTICK_DEVICE).c_str());
  printf("\n");
}
//=============================================================================




//=============================================================================
//Main
//=============================================================================
int main(int argc, char** argv)
{
  //Check for "-h" or "--help"
  for (int i = 1; i < argc; i++) {
    if ((0 == strncmp(argv[i], "-h", strlen("-h")))
        || (0 == strncmp(argv[i], "--help", strlen("--help")))) {
      printUsage(argc, argv);
      return 0;
    }
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

  //Read parameters and set default values
  nodeHandle.param(PARAM_KEY_TOPIC,           topic,          std::string(PARAM_DEFAULT_TOPIC));
  nodeHandle.param(PARAM_KEY_TYPE,            type,           std::string(PARAM_DEFAULT_TYPE));
  nodeHandle.param(PARAM_KEY_LISTEN_TIMEOUT,  listenTimeout,  PARAM_DEFAULT_LISTEN_TIMEOUT);
  nodeHandle.param(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice, PARAM_DEFAULT_JOYSTICK_DEVICE);
  nodeHandle.param(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps,  PARAM_DEFAULT_KEYBOARD_STEPS);
  nodeHandle.param(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone,   PARAM_DEFAULT_AXIS_DEAD_ZONE);

  //Advertise parameters for introspection
  nodeHandle.setParam(PARAM_KEY_TOPIC,           topic);
  nodeHandle.setParam(PARAM_KEY_TYPE,            type);
  nodeHandle.setParam(PARAM_KEY_LISTEN_TIMEOUT,  listenTimeout);
  nodeHandle.setParam(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice);
  nodeHandle.setParam(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps);
  nodeHandle.setParam(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone);

  //Advertise publisher with buffer size set to 1 and latching on.  The
  //publisher should basically just always contain the latest teleop state.
  ros::Publisher publisher = nodeHandle.advertise<teleop_msgs::State>(topic, 1, true);

  //Create callback object using publisher
  TeleopSourceCallbackROS callback(&publisher);

  //Create teleop source and point to it with the global gTeleopSource
  gTeleopSource = teleopSourceFactory(&callback,
                                      type,
                                      listenTimeout,
                                      axisDeadZone,
                                      keyboardSteps,
                                      joystickDevice);
  if (NULL == gTeleopSource) {
    ROS_ERROR("NULL teleop source\n");
    return 1;
  }

  //Start teleop source
  if (!gTeleopSource->start()) {
    ROS_ERROR("Error starting teleop source\n");
    return 1;
  }

  //While ROS is running, check to see if the teleop source has stopped
  //executing.  This is only needed in order to detect fatal errors. Normally
  //the loop will be stopped due to SIGINT (CTRL-C), which will trigger a call
  //to quit().  If we didn't care about fatal errors we could just do a
  //ros::spin() here instead.
  ros::Rate rate(10);
  while (ros::ok()) {
    //If listening thread is no longer executing quit
    if (!gTeleopSource->isExecuting()) {
      quit(gTeleopSource);
    }

    //Sleep away the rest of this iteration
    rate.sleep();
  }

  return 0;
}
