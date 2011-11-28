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

/**@{ Teleop types */
#define TELEOP_TYPE_JOYSTICK            "joystick"
#define TELEOP_TYPE_KEYBOARD            "keyboard"
/**@}*/

/**@{ Parameter keys */
#define PARAM_KEY_TELEOP_TOPIC          "teleop_topic"
#define PARAM_KEY_TELEOP_TYPE           "teleop_type"
#define PARAM_KEY_LISTEN_TIMEOUT        "listen_timeout"
#define PARAM_KEY_JOYSTICK_DEVICE       "joystick_device"
#define PARAM_KEY_KEYBOARD_STEPS        "keyboard_steps"
#define PARAM_KEY_AXIS_DEAD_ZONE        "axis_dead_zone"
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TELEOP_TOPIC      "teleop"
#define PARAM_DEFAULT_TELEOP_TYPE       TELEOP_TYPE_KEYBOARD
#define PARAM_DEFAULT_LISTEN_TIMEOUT    ((int)teleop::TeleopSource::LISTEN_TIMEOUT_DEFAULT)
#define PARAM_DEFAULT_JOYSTICK_DEVICE   (teleop::TeleopSourceJoystick::getDefaultDevice())
#define PARAM_DEFAULT_KEYBOARD_STEPS    ((int)teleop::TeleopSourceKeyboard::STEPS_DEFAULT)
#define PARAM_DEFAULT_AXIS_DEAD_ZONE    ((teleop::TeleopAxisValue)teleop::TeleopSource::AXIS_DEAD_ZONE_DEFAULT)
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
   *
   *   @param publisher [in] - publisher for teleop messages
   */
  explicit TeleopSourceCallbackRos(const ros::Publisher* const publisher);

  /**
   * Destructor.
   */
  ~TeleopSourceCallbackRos();

  /**
   * Override virtual method from parent.
   */
  virtual void updated(const teleop::TeleopState* const teleopState, bool stopping, bool error);

  /**
   * Override virtual method from parent.
   */
  virtual void stopping(bool error);

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
 *   @param signalNumber [in] - received signal number
 */
void signalHandler(int signalNumber);

/**
 * Clean up and shutdown.
 */
void quit();

/**
 * Utility function for creating a teleop source
 *
 *   @param teleopSourceCallback [in] - callback object to use
 *   @param type [in] - teleop type
 *   @param listenTimeout [in] - listen timeout value in milliseconds
 *   @param axisDeadZone [in] - default dead zone
 *   @param keyboardSteps [in] - if this is a keyboard, use this many steps
 *   @param joystickDevice [in] - if this is a joystick, use this device
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
 *   @param argc [in] - number of command line arguments
 *   @param argv [in] - command line arguments
 *
 *   @return true if usage was printed
 */
bool printUsage(int argc, char** argv);




//=============================================================================
//Globals
//=============================================================================

/**@{ Globals used to check for quit request from any thread */
static boost::condition_variable gQuitRequestedCondition;
static boost::mutex gQuitRequestedMutex;
static bool gQuitRequested = false;
/**@}*/




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceCallbackRos::TeleopSourceCallbackRos(const ros::Publisher* const publisher) :
  mPublisher(publisher) {
}
//=============================================================================
TeleopSourceCallbackRos::~TeleopSourceCallbackRos() {
  //Sanity check publisher
  if (NULL == mPublisher) {
    ROS_ERROR("~TeleopSourceCallbackRos: NULL publisher");
    return;
  }

  //Zero message
  for (size_t i = 0; i < mTeleopStateMsg.axes.size(); i++) {
    mTeleopStateMsg.axes[i].value = 0.0;
  }
  for (size_t i = 0; i < mTeleopStateMsg.buttons.size(); i++) {
    mTeleopStateMsg.buttons[i].value = 0.0;
  }

  //Publish zero message
  mPublisher->publish(mTeleopStateMsg);
}
//=============================================================================
void TeleopSourceCallbackRos::updated(const teleop::TeleopState* const teleopState, bool stopping, bool error) {
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
  for (size_t i = 0; i < mTeleopStateMsg.axes.size(); i++) {
    mTeleopStateMsg.axes[i].type = teleopState->axes[i].type;
    mTeleopStateMsg.axes[i].value = teleopState->axes[i].value;
  }
  for (size_t i = 0; i < mTeleopStateMsg.buttons.size(); i++) {
    mTeleopStateMsg.buttons[i].type = teleopState->buttons[i].type;
    mTeleopStateMsg.buttons[i].value = teleopState->buttons[i].value;
  }

  //Publish result
  mPublisher->publish(mTeleopStateMsg);

  //On error just print a message.  The stopping condition will be handled in
  //the stopping() callback, so we can safely ignore it here.
  if (error) {
    ROS_ERROR("updated: error detected");
  }
}
//=============================================================================
void TeleopSourceCallbackRos::stopping(bool error) {
  //If the teleop source is stopping for any reason just quit
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
  //Notify that quit has been requested if we haven't already done so
  boost::lock_guard<boost::mutex> quitRequestedLock(gQuitRequestedMutex);
  if (!gQuitRequested) {
    gQuitRequested = true;
    gQuitRequestedCondition.notify_all();
  }
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
  using std::printf;

  //Check for "-h" or "--help", if found, print usage
  for (int i = 1; i < argc; i++) {
    if ((0 == strcmp(argv[i], "-h")) || (0 == strcmp(argv[i], "--help"))) {
      printf("\n");
      printf("Usage:\n");
      printf("    %s [params]\n", basename(argv[0]));
      printf("\n");
      printf("Parameters and their default values\n");
      printf("    _%s:=%s\n",    PARAM_KEY_TELEOP_TOPIC,
                                 (std::string("<node>/") + std::string(PARAM_DEFAULT_TELEOP_TOPIC)).c_str());
      printf("    _%s:=%s\n",    PARAM_KEY_TELEOP_TYPE,     PARAM_DEFAULT_TELEOP_TYPE);
      printf("    _%s:=%d\n",    PARAM_KEY_LISTEN_TIMEOUT,  PARAM_DEFAULT_LISTEN_TIMEOUT);
      printf("    _%s:=%.02f\n", PARAM_KEY_AXIS_DEAD_ZONE,  PARAM_DEFAULT_AXIS_DEAD_ZONE);
      printf("    _%s:=%d\n",    PARAM_KEY_KEYBOARD_STEPS,  PARAM_DEFAULT_KEYBOARD_STEPS);
      printf("    _%s:=%s\n",    PARAM_KEY_JOYSTICK_DEVICE, (PARAM_DEFAULT_JOYSTICK_DEVICE).c_str());
      printf("\n");
      return true;
    }
  }
  return false;
}
//=============================================================================




//=============================================================================
//Main
//=============================================================================
int main(int argc, char** argv) {
  int result = 0;

  //Check if we should just print usage information and quit
  if (printUsage(argc, argv)) {
    return result;
  }

  //Set signal handler
  signal(SIGINT, signalHandler);

  //Initialise ROS (exceptions ignored intentionally)
  std::string nodeName(basename(argv[0]));
  ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);

  //Node handles using private and default namespaces (exceptions ignored intentionally)
  ros::NodeHandle nodeHandlePrivate("~");
  ros::NodeHandle nodeHandle("");

  //Declare parameters
  std::string teleopTopic;
  std::string teleopType;
  int listenTimeout;
  std::string joystickDevice;
  int keyboardSteps;
  double axisDeadZone;

  //Read parameters and/or set default values
  nodeHandlePrivate.param(PARAM_KEY_TELEOP_TOPIC,    teleopTopic,
                          nodeName + std::string("/") + std::string(PARAM_DEFAULT_TELEOP_TOPIC));
  nodeHandlePrivate.param(PARAM_KEY_TELEOP_TYPE,     teleopType,     std::string(PARAM_DEFAULT_TELEOP_TYPE));
  nodeHandlePrivate.param(PARAM_KEY_LISTEN_TIMEOUT,  listenTimeout,  PARAM_DEFAULT_LISTEN_TIMEOUT);
  nodeHandlePrivate.param(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice, PARAM_DEFAULT_JOYSTICK_DEVICE);
  nodeHandlePrivate.param(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps,  PARAM_DEFAULT_KEYBOARD_STEPS);
  nodeHandlePrivate.param(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone,   PARAM_DEFAULT_AXIS_DEAD_ZONE);

  //Advertise all parameters to allow introspection
  nodeHandlePrivate.setParam(PARAM_KEY_TELEOP_TOPIC,    teleopTopic);
  nodeHandlePrivate.setParam(PARAM_KEY_TELEOP_TYPE,     teleopType);
  nodeHandlePrivate.setParam(PARAM_KEY_LISTEN_TIMEOUT,  listenTimeout);
  nodeHandlePrivate.setParam(PARAM_KEY_JOYSTICK_DEVICE, joystickDevice);
  nodeHandlePrivate.setParam(PARAM_KEY_KEYBOARD_STEPS,  keyboardSteps);
  nodeHandlePrivate.setParam(PARAM_KEY_AXIS_DEAD_ZONE,  axisDeadZone);

  //Create publisher with buffer size set to 1 and latching on.  The publisher
  //should basically just always contain the latest teleop state.
  ros::Publisher publisher = nodeHandle.advertise<teleop_msgs::State>(teleopTopic, 1, true);

  //Create callback using publisher.  The callback destructor may want to
  //publish a final message, so we use dynamic allocation here.  This means we
  //can free this object and sleep a bit before the publisher and node handle
  //go out of scope and get destroyed.
  TeleopSourceCallbackRos* callback = new TeleopSourceCallbackRos(&publisher);

  //Create teleop source using callback and parameters
  teleop::TeleopSource* teleopSource = teleopSourceFactory(callback,
                                                           teleopType,
                                                           listenTimeout,
                                                           axisDeadZone,
                                                           keyboardSteps,
                                                           joystickDevice);

  //Use teleop source if it was successfully created
  if (NULL == teleopSource) {
    ROS_ERROR("main: NULL teleop source");
    result = 1;
  } else {
    //Start teleop source
    if (!teleopSource->start()) {
      ROS_ERROR("main: error starting teleop source");
      result = 1;
    } else {
      //Wait for quit request
      boost::unique_lock<boost::mutex> quitRequestedLock(gQuitRequestedMutex);
      while(!gQuitRequested) {
        gQuitRequestedCondition.wait(quitRequestedLock);
      }
      quitRequestedLock.unlock();
    }

    //Stop and free teleop source
    teleopSource->stop();
    teleopSource->waitForStopped();
    delete teleopSource;
  }

  //Free callback object
  delete callback;

  //Sleep a bit to allow final messages to be published, if possible.  Use
  //boost thread sleep rather than ROS sleep since ROS sleep may try to use
  //a simulated clock which has been stopped at the same time as this node.
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  //Done
  return result;
}
