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
#include <teleop_source_adapter.hpp>
#include <teleop_source_keyboard.hpp>
#include <teleop_source_joystick.hpp>
#include <teleop_msgs/State.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <signal.h>
#include <stdio.h>




//=============================================================================
//Namespace
//=============================================================================
namespace {




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
 * The start() and stop() methods control the teleop source (and corresponding
 * message publication).
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
class TeleopSourceNode : public teleop::TeleopSourceAdapter::TeleopSourceAdapterCallback {

public:

  /**@{ Teleop types */
  static const char TELEOP_TYPE_AUTO[];
  static const char TELEOP_TYPE_KEYBOARD[];
  static const char TELEOP_TYPE_JOYSTICK[];
  /**@}*/

  /**@{ Parameter keys */
  static const char PARAM_KEY_TELEOP_TOPIC[];
  static const char PARAM_KEY_TELEOP_TYPE[];
  static const char PARAM_KEY_LISTEN_TIMEOUT[];
  static const char PARAM_KEY_AXIS_DEAD_ZONE[];
  static const char PARAM_KEY_KEYBOARD_STEPS[];
  static const char PARAM_KEY_JOYSTICK_DEVICE[];
  /**@}*/

  /**@{ Parameter default values */
  static const char   PARAM_DEFAULT_TELEOP_TOPIC[];
  static const char*  PARAM_DEFAULT_TELEOP_TYPE;
  static const int    PARAM_DEFAULT_LISTEN_TIMEOUT;
  static const double PARAM_DEFAULT_AXIS_DEAD_ZONE;
  static const int    PARAM_DEFAULT_KEYBOARD_STEPS;
  static const char   PARAM_DEFAULT_JOYSTICK_DEVICE[];
  /**@}*/

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

  /**
   * Start node.
   *
   *   @param blocking - if true block until stopped
   *
   *   @return true on success
   */
  bool start(bool blocking);

  /**
   * Stop node.
   *
   *   @param blocking - if true block until stopped
   *
   *   @return true on success
   */
  bool stop(bool blocking);

  /**
   * Check if node is initialised.
   *
   *   @return true if initialised
   */
  bool isInitialised();

  /**
   * Check if node is running.
   *
   *   @return true if running
   */
  bool isRunning();

private:

  /**@{ Parameters */
  std::string mTeleopTopic;
  std::string mTeleopType;
  int mListenTimeout;
  double mAxisDeadZone;
  int mKeyboardSteps;
  std::string mJoystickDevice;
  /**@}*/

  /** Teleop source (dynamically allocated) */
  teleop::TeleopSource* mTeleopSource;

  /** Teleop source adapter */
  teleop::TeleopSourceAdapter mTeleopSourceAdapter;

  /** Publisher for teleop messages */
  ros::Publisher mPublisher;

  /** Teleop message member to avoid re-creation at every call to updated() */
  teleop_msgs::State mTeleopStateMsg;

  /** Init done flag */
  bool mIsInitialised;

  /** Mutex to protect is initialised flag */
  boost::recursive_mutex mIsInitialisedMutex;

  /**
   * Utility method for creating a teleop source, given the currently set
   * parameters.  The produced teleop source is dynamically allocated.
   *
   *   @return teleop source on success and NULL on error
   */
  teleop::TeleopSource* teleopSourceFactory();

  /**
   * Override virtual method from parent.
   */
  virtual void updated(const teleop::TeleopState* const teleopState, bool stopping, bool error);

  /**
   * Override virtual method from parent.
   */
  virtual void stopping(bool error);

}; //class




//=============================================================================
//Function prototypes
//=============================================================================
/**
 * Signal handler.
 *
 *   @param signalNumber [in] - received signal number
 */
void signalHandler(int signalNumber);

/**
 * Check if we should just print usage information, and if so, print it.
 *
 *   @param nodeName [in] - node name
 *   @param argc [in] - number of command line arguments
 *   @param argv [in] - command line arguments
 *
 *   @return true if usage was printed
 */
bool printUsage(int argc, char** argv);




//=============================================================================
//Globals
//=============================================================================
//Global atomic flag used to indicate if an interrupt has been requested.
sig_atomic_t gInterruptRequested = 0;




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
const int  TeleopSourceNode::PARAM_DEFAULT_LISTEN_TIMEOUT =     teleop::TeleopSourceAdapter::LISTEN_TIMEOUT_DEFAULT;
const double TeleopSourceNode::PARAM_DEFAULT_AXIS_DEAD_ZONE =   teleop::TeleopSourceAdapter::AXIS_DEAD_ZONE_DEFAULT;
const int  TeleopSourceNode::PARAM_DEFAULT_KEYBOARD_STEPS =     teleop::TeleopSourceKeyboard::STEPS_DEFAULT;

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
  //Lock access to init status
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

  //Start node manually to avoid node shutdown when last handle is destroyed
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
    //Create relative node handle for topics (exceptions ignored intentionally).
    ros::NodeHandle nodeHandleRelative = ros::NodeHandle("");

    //Create publisher with buffer size set to 1 and latching on.  The publisher
    //should basically just always contain the latest teleop state.
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
      || !mTeleopSourceAdapter.setAxisDeadZoneForAllAxes((teleop::TeleopAxisValue)mAxisDeadZone)) {
    ROS_ERROR("TeleopSourceNode::init: error configuring teleop source adapter");
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
  //Check init done flag
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);
  if (!mIsInitialised) {
    return true;
  }

  //Shutdown teleop source adapter
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

  return true;
}
//=============================================================================
bool TeleopSourceNode::start(bool blocking) {
  //Check init done flag
  boost::unique_lock<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);
  if (!mIsInitialised) {
    ROS_ERROR("TeleopSourceNode::start: node not initialised");
    return false;
  }
  isInitialisedLock.unlock();

  if (!mTeleopSourceAdapter.start(blocking)) {
    ROS_ERROR("TeleopSourceNode::start: error starting teleop source");
    return false;
  } else {
    return true;
  }
}
//=============================================================================
bool TeleopSourceNode::stop(bool blocking) {
  //Check init done flag
  boost::unique_lock<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);
  if (!mIsInitialised) {
    ROS_ERROR("TeleopSourceNode::stop: node not initialised");
    return false;
  }
  isInitialisedLock.unlock();

  if (!mTeleopSourceAdapter.stop(blocking)) {
    ROS_ERROR("TeleopSourceNode::stop: error stopping teleop source");
    return false;
  } else {
    return true;
  }
}
//=============================================================================
bool TeleopSourceNode::isInitialised() {
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);
  return mIsInitialised;
}
//=============================================================================
bool TeleopSourceNode::isRunning() {
  //Check init done flag
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);
  if (!mIsInitialised) {
    return false;
  } else {
    return mTeleopSourceAdapter.isRunning();
  }
}
//=============================================================================
teleop::TeleopSource* TeleopSourceNode::teleopSourceFactory() {
  teleop::TeleopSource* teleopSource = NULL;
  std::string teleopType = mTeleopType;

  //If type is auto, check for best alternative
  if (0 == teleopType.compare(std::string(TELEOP_TYPE_AUTO))) {
    teleop::TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new teleop::TeleopSourceJoystick();
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
    teleop::TeleopSourceKeyboard* teleopSourceKeyboard;
    teleopSourceKeyboard = new teleop::TeleopSourceKeyboard();
    if (!teleopSourceKeyboard->setSteps((unsigned int)mKeyboardSteps)) {
      ROS_WARN("TeleopSourceNode::teleopSourceFactory: unable to set keyboard steps");
    }
    teleopSource = teleopSourceKeyboard;
  } else if (0 == teleopType.compare(std::string(TELEOP_TYPE_JOYSTICK))) {
    teleop::TeleopSourceJoystick* teleopSourceJoystick;
    teleopSourceJoystick = new teleop::TeleopSourceJoystick();
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
void TeleopSourceNode::updated(const teleop::TeleopState* const teleopState, bool stopping, bool error) {
  //Sanity check teleopState
  if (NULL == teleopState) {
    ROS_ERROR("TeleopSourceNode::updated: NULL teleopState");
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
//Function definitions
//=============================================================================
void signalHandler(int signalNumber) {
  gInterruptRequested = 1;
}
//=============================================================================
bool printUsage(std::string nodeName, int argc, char** argv) {
  //Check for "-h" or "--help", if found, print usage
  for (int i = 1; i < argc; i++) {
    if ((0 == strcmp(argv[i], "-h")) || (0 == strcmp(argv[i], "--help"))) {
      printf("\n");
      printf("Usage:\n");
      printf("    %s [params]\n", nodeName.c_str());
      printf("\n");
      printf("Parameters and their default values\n");
      printf("    _%s:=%s\n",
             TeleopSourceNode::PARAM_KEY_TELEOP_TOPIC,
             (nodeName + std::string("/") + std::string(TeleopSourceNode::PARAM_DEFAULT_TELEOP_TOPIC)).c_str());
      printf("    _%s:=%s\n",
             TeleopSourceNode::PARAM_KEY_TELEOP_TYPE,
             TeleopSourceNode::PARAM_DEFAULT_TELEOP_TYPE);
      printf("    _%s:=%d\n",
             TeleopSourceNode::PARAM_KEY_LISTEN_TIMEOUT,
             TeleopSourceNode::PARAM_DEFAULT_LISTEN_TIMEOUT);
      printf("    _%s:=%.02f\n",
             TeleopSourceNode::PARAM_KEY_AXIS_DEAD_ZONE,
             TeleopSourceNode::PARAM_DEFAULT_AXIS_DEAD_ZONE);
      printf("    _%s:=%d\n",
             TeleopSourceNode::PARAM_KEY_KEYBOARD_STEPS,
             TeleopSourceNode::PARAM_DEFAULT_KEYBOARD_STEPS);
      printf("    _%s:=%s\n",
             TeleopSourceNode::PARAM_KEY_JOYSTICK_DEVICE,
             TeleopSourceNode::PARAM_DEFAULT_JOYSTICK_DEVICE);
      printf("\n");
      return true;
    }
  }
  return false;
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
//Main
//=============================================================================
int main(int argc, char** argv) {
  //Use first argument (executable name) as node name
  std::string nodeName(basename(argv[0]));

  //Check if we should just print usage information and quit
  if (printUsage(nodeName, argc, argv)) {
    return 0;
  }

  //Set SIGINT signal handler
  signal(SIGINT, signalHandler);

  //Create the node object
  TeleopSourceNode TeleopSourceNode;

  //Init node object
  if (!TeleopSourceNode.init(argc, argv, nodeName, ros::init_options::NoSigintHandler)) {
    ROS_ERROR("main: error initialising node");
    return 1;
  }

  //Start the node in non-blocking mode
  if (!TeleopSourceNode.start(false)) {
    ROS_ERROR("main: error starting node");
    return 1;
  }

  //Periodically check for interruption
  while (0 == gInterruptRequested) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  }

  //Done
  return 0;
}
//=============================================================================
