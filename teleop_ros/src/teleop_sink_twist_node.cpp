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
#include <teleop_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdint.h>
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
 * This class creates a teleop sink ROS node which subscribes to a teleop topic
 * provided by a teleop source device and publishes a corresponding twist
 * topic.
 *
 * The sink node can represent various types of devices, as long as they can
 * interpret twist messages.  Available axes and axis characteristics (e.g.
 * grow quadradically and obey throttle axis if available) can be set.
 *
 * The init() and shutdown() methods control the life cycle of the object.
 * Note that shutdown() is always called on destruction.
 *
 * Optional parameters for the node are the following:
 *
 *   teleop_topic:                teleop topic to which to subscribe
 *   twist_topic:                 twist topic to which to publish
 *   has_(lin|rot)_(x|y|z)        true if this sink device has the given axis
 *   min_(lin|rot)_(x|y|z)        min value for given axis
 *   max_(lin|rot)_(x|y|z)        max value for given axis
 *   quadratic_(lin|rot)_(x|y|z)  true if given axis should grow quadratically
 *   throttle_(lin|rot)_(x|y|z)   true if given axis should obey the throttle
 */
class TeleopSinkTwistNode  {

public:

  //Parameter keys
  static const char PARAM_KEY_TELEOP_TOPIC[];
  static const char PARAM_KEY_TWIST_TOPIC[];
  static const char PARAM_KEY_HAS_LIN_X[];
  static const char PARAM_KEY_HAS_LIN_Y[];
  static const char PARAM_KEY_HAS_LIN_Z[];
  static const char PARAM_KEY_HAS_ROT_X[];
  static const char PARAM_KEY_HAS_ROT_Y[];
  static const char PARAM_KEY_HAS_ROT_Z[];
  static const char PARAM_KEY_MIN_LIN_X[];
  static const char PARAM_KEY_MIN_LIN_Y[];
  static const char PARAM_KEY_MIN_LIN_Z[];
  static const char PARAM_KEY_MIN_ROT_X[];
  static const char PARAM_KEY_MIN_ROT_Y[];
  static const char PARAM_KEY_MIN_ROT_Z[];
  static const char PARAM_KEY_MAX_LIN_X[];
  static const char PARAM_KEY_MAX_LIN_Y[];
  static const char PARAM_KEY_MAX_LIN_Z[];
  static const char PARAM_KEY_MAX_ROT_X[];
  static const char PARAM_KEY_MAX_ROT_Y[];
  static const char PARAM_KEY_MAX_ROT_Z[];
  static const char PARAM_KEY_QUADRATIC_LIN_X[];
  static const char PARAM_KEY_QUADRATIC_LIN_Y[];
  static const char PARAM_KEY_QUADRATIC_LIN_Z[];
  static const char PARAM_KEY_QUADRATIC_ROT_X[];
  static const char PARAM_KEY_QUADRATIC_ROT_Y[];
  static const char PARAM_KEY_QUADRATIC_ROT_Z[];
  static const char PARAM_KEY_THROTTLE_LIN_X[];
  static const char PARAM_KEY_THROTTLE_LIN_Y[];
  static const char PARAM_KEY_THROTTLE_LIN_Z[];
  static const char PARAM_KEY_THROTTLE_ROT_X[];
  static const char PARAM_KEY_THROTTLE_ROT_Y[];
  static const char PARAM_KEY_THROTTLE_ROT_Z[];

  //Parameter default values
  static const char PARAM_DEFAULT_TELEOP_TOPIC[];
  static const char PARAM_DEFAULT_TWIST_TOPIC[];
  static const bool PARAM_DEFAULT_HAS_LIN_X;
  static const bool PARAM_DEFAULT_HAS_LIN_Y;
  static const bool PARAM_DEFAULT_HAS_LIN_Z;
  static const bool PARAM_DEFAULT_HAS_ROT_X;
  static const bool PARAM_DEFAULT_HAS_ROT_Y;
  static const bool PARAM_DEFAULT_HAS_ROT_Z;
  static const double PARAM_DEFAULT_MIN_LIN_X;
  static const double PARAM_DEFAULT_MIN_LIN_Y;
  static const double PARAM_DEFAULT_MIN_LIN_Z;
  static const double PARAM_DEFAULT_MIN_ROT_X;
  static const double PARAM_DEFAULT_MIN_ROT_Y;
  static const double PARAM_DEFAULT_MIN_ROT_Z;
  static const double PARAM_DEFAULT_MAX_LIN_X;
  static const double PARAM_DEFAULT_MAX_LIN_Y;
  static const double PARAM_DEFAULT_MAX_LIN_Z;
  static const double PARAM_DEFAULT_MAX_ROT_X;
  static const double PARAM_DEFAULT_MAX_ROT_Y;
  static const double PARAM_DEFAULT_MAX_ROT_Z;
  static const bool PARAM_DEFAULT_QUADRATIC_LIN_X;
  static const bool PARAM_DEFAULT_QUADRATIC_LIN_Y;
  static const bool PARAM_DEFAULT_QUADRATIC_LIN_Z;
  static const bool PARAM_DEFAULT_QUADRATIC_ROT_X;
  static const bool PARAM_DEFAULT_QUADRATIC_ROT_Y;
  static const bool PARAM_DEFAULT_QUADRATIC_ROT_Z;
  static const bool PARAM_DEFAULT_THROTTLE_LIN_X;
  static const bool PARAM_DEFAULT_THROTTLE_LIN_Y;
  static const bool PARAM_DEFAULT_THROTTLE_LIN_Z;
  static const bool PARAM_DEFAULT_THROTTLE_ROT_X;
  static const bool PARAM_DEFAULT_THROTTLE_ROT_Y;
  static const bool PARAM_DEFAULT_THROTTLE_ROT_Z;

  /**
   * Constructor.
   */
  TeleopSinkTwistNode();

  /**
   * Destructor.
   */
  ~TeleopSinkTwistNode();

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
  std::string mTwistTopic;
  bool mHasLinX, mHasLinY, mHasLinZ, mHasRotX, mHasRotY, mHasRotZ;
  double mMinLinX, mMinLinY, mMinLinZ, mMinRotX, mMinRotY, mMinRotZ;
  double mMaxLinX, mMaxLinY, mMaxLinZ, mMaxRotX, mMaxRotY, mMaxRotZ;
  bool mQuadraticLinX, mQuadraticLinY, mQuadraticLinZ, mQuadraticRotX, mQuadraticRotY, mQuadraticRotZ;
  bool mThrottleLinX, mThrottleLinY, mThrottleLinZ, mThrottleRotX, mThrottleRotY, mThrottleRotZ;

  /** Publisher for twist messages */
  ros::Publisher mPublisher;

  /** Subscriber for teleop messages */
  ros::Subscriber mSubscriber;

  /** Latest message */
  geometry_msgs::Twist mTwistMsg;

  /** Spinner to receive ROS events */
  ros::AsyncSpinner* mSpinner;

  /** Is initialised flag */
  bool mIsInitialised;

  /** Mutex to protect is initialised flag */
  boost::recursive_mutex mIsInitialisedMutex;

  /**
   * Called when teleop topic is updated.  This converts the given teleop state
   * message to a twist message and publishes it.
   *
   *   @param teleopStateMsg [in] - received teleop state message
   */
  void updated(const teleop_msgs::State& teleopStateMsg);

  /**
   * Convert teleop state to twist.
   *
   *  @param teleopStateMsg [in] - teleop state to convert
   *  @param twistMsg [out] - resulting twist message
   *
   *  @return true on success
   */
  inline bool teleopStateToTwist(const teleop_msgs::State* const teleopStateMsg, geometry_msgs::Twist* const twistMsg);

  /**
   * Apply quadratic factor to teleop axis value if enabled.
   *
   *   @param enabled [in] - true if enabled
   *   @param teleopAxisValue [in] - original teleop axis value
   *
   *   @return updated teleopAxisValue
   */
  inline teleop::TeleopAxisValue applyQuadratic(bool enabled, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Apply throttle factor to teleop axis value if enabled.
   *
   *   @param enabled [in] - true if enabled
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - original teleop axis value
   *
   *   @return updated teleopAxisValue
   */
  inline teleop::TeleopAxisValue applyThrottle(bool enabled, double throttle, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistLinX(double throttle, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistLinY(double throttle, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistLinZ(double throttle, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistRotX(double throttle, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistRotY(double throttle, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistRotZ(double throttle, teleop::TeleopAxisValue teleopAxisValue);

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
/** Global atomic flag used to indicate if an interrupt has been requested. */
sig_atomic_t gInterruptRequested = 0;




//=============================================================================
//Static member definitions
//=============================================================================
const char   TeleopSinkTwistNode::PARAM_KEY_TELEOP_TOPIC[] =      "teleop_topic";
const char   TeleopSinkTwistNode::PARAM_KEY_TWIST_TOPIC[] =       "twist_topic";
const char   TeleopSinkTwistNode::PARAM_KEY_HAS_LIN_X[] =         "has_lin_x";
const char   TeleopSinkTwistNode::PARAM_KEY_HAS_LIN_Y[] =         "has_lin_y";
const char   TeleopSinkTwistNode::PARAM_KEY_HAS_LIN_Z[] =         "has_lin_z";
const char   TeleopSinkTwistNode::PARAM_KEY_HAS_ROT_X[] =         "has_rot_x";
const char   TeleopSinkTwistNode::PARAM_KEY_HAS_ROT_Y[] =         "has_rot_y";
const char   TeleopSinkTwistNode::PARAM_KEY_HAS_ROT_Z[] =         "has_rot_z";
const char   TeleopSinkTwistNode::PARAM_KEY_MIN_LIN_X[] =         "min_lin_x";
const char   TeleopSinkTwistNode::PARAM_KEY_MIN_LIN_Y[] =         "min_lin_y";
const char   TeleopSinkTwistNode::PARAM_KEY_MIN_LIN_Z[] =         "min_lin_z";
const char   TeleopSinkTwistNode::PARAM_KEY_MIN_ROT_X[] =         "min_rot_x";
const char   TeleopSinkTwistNode::PARAM_KEY_MIN_ROT_Y[] =         "min_rot_y";
const char   TeleopSinkTwistNode::PARAM_KEY_MIN_ROT_Z[] =         "min_rot_z";
const char   TeleopSinkTwistNode::PARAM_KEY_MAX_LIN_X[] =         "max_lin_x";
const char   TeleopSinkTwistNode::PARAM_KEY_MAX_LIN_Y[] =         "max_lin_y";
const char   TeleopSinkTwistNode::PARAM_KEY_MAX_LIN_Z[] =         "max_lin_z";
const char   TeleopSinkTwistNode::PARAM_KEY_MAX_ROT_X[] =         "max_rot_x";
const char   TeleopSinkTwistNode::PARAM_KEY_MAX_ROT_Y[] =         "max_rot_y";
const char   TeleopSinkTwistNode::PARAM_KEY_MAX_ROT_Z[] =         "max_rot_z";
const char   TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_LIN_X[] =   "quadratic_lin_x";
const char   TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_LIN_Y[] =   "quadratic_lin_y";
const char   TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_LIN_Z[] =   "quadratic_lin_z";
const char   TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_ROT_X[] =   "quadratic_rot_x";
const char   TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_ROT_Y[] =   "quadratic_rot_y";
const char   TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_ROT_Z[] =   "quadratic_rot_z";
const char   TeleopSinkTwistNode::PARAM_KEY_THROTTLE_LIN_X[] =    "throttle_lin_x";
const char   TeleopSinkTwistNode::PARAM_KEY_THROTTLE_LIN_Y[] =    "throttle_lin_y";
const char   TeleopSinkTwistNode::PARAM_KEY_THROTTLE_LIN_Z[] =    "throttle_lin_z";
const char   TeleopSinkTwistNode::PARAM_KEY_THROTTLE_ROT_X[] =    "throttle_rot_x";
const char   TeleopSinkTwistNode::PARAM_KEY_THROTTLE_ROT_Y[] =    "throttle_rot_y";
const char   TeleopSinkTwistNode::PARAM_KEY_THROTTLE_ROT_Z[] =    "throttle_rot_z";

const char   TeleopSinkTwistNode::PARAM_DEFAULT_TELEOP_TOPIC[] =  "teleop";
const char   TeleopSinkTwistNode::PARAM_DEFAULT_TWIST_TOPIC[] =   "twist";
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_HAS_LIN_X =       false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_HAS_LIN_Y =       false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_HAS_LIN_Z =       false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_HAS_ROT_X =       false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_HAS_ROT_Y =       false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_HAS_ROT_Z =       false;
const double TeleopSinkTwistNode::PARAM_DEFAULT_MIN_LIN_X =       (-0.5);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MIN_LIN_Y =       (-0.5);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MIN_LIN_Z =       (-0.5);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MIN_ROT_X =       (-0.8);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MIN_ROT_Y =       (-0.8);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MIN_ROT_Z =       (-0.8);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MAX_LIN_X =       (0.5);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MAX_LIN_Y =       (0.5);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MAX_LIN_Z =       (0.5);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MAX_ROT_X =       (0.8);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MAX_ROT_Y =       (0.8);
const double TeleopSinkTwistNode::PARAM_DEFAULT_MAX_ROT_Z =       (0.8);
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_LIN_X = true;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_LIN_Y = true;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_LIN_Z = true;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_ROT_X = false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_ROT_Y = false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_ROT_Z = false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_LIN_X =  false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_LIN_Y =  false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_LIN_Z =  false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_ROT_X =  false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_ROT_Y =  false;
const bool   TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_ROT_Z =  false;




//=============================================================================
//Method definitions
//=============================================================================
TeleopSinkTwistNode::TeleopSinkTwistNode() :
  mTeleopTopic(PARAM_DEFAULT_TELEOP_TOPIC),
  mTwistTopic(PARAM_DEFAULT_TWIST_TOPIC),
  mHasLinX(PARAM_DEFAULT_HAS_LIN_X),
  mHasLinY(PARAM_DEFAULT_HAS_LIN_Y),
  mHasLinZ(PARAM_DEFAULT_HAS_LIN_Z),
  mHasRotX(PARAM_DEFAULT_HAS_ROT_X),
  mHasRotY(PARAM_DEFAULT_HAS_ROT_Y),
  mHasRotZ(PARAM_DEFAULT_HAS_ROT_Z),
  mMinLinX(PARAM_DEFAULT_MIN_LIN_X),
  mMinLinY(PARAM_DEFAULT_MIN_LIN_Y),
  mMinLinZ(PARAM_DEFAULT_MIN_LIN_Z),
  mMinRotX(PARAM_DEFAULT_MIN_ROT_X),
  mMinRotY(PARAM_DEFAULT_MIN_ROT_Y),
  mMinRotZ(PARAM_DEFAULT_MIN_ROT_Z),
  mMaxLinX(PARAM_DEFAULT_MAX_LIN_X),
  mMaxLinY(PARAM_DEFAULT_MAX_LIN_Y),
  mMaxLinZ(PARAM_DEFAULT_MAX_LIN_Z),
  mMaxRotX(PARAM_DEFAULT_MAX_ROT_X),
  mMaxRotY(PARAM_DEFAULT_MAX_ROT_Y),
  mMaxRotZ(PARAM_DEFAULT_MAX_ROT_Z),
  mQuadraticLinX(PARAM_DEFAULT_QUADRATIC_LIN_X),
  mQuadraticLinY(PARAM_DEFAULT_QUADRATIC_LIN_Y),
  mQuadraticLinZ(PARAM_DEFAULT_QUADRATIC_LIN_Z),
  mQuadraticRotX(PARAM_DEFAULT_QUADRATIC_ROT_X),
  mQuadraticRotY(PARAM_DEFAULT_QUADRATIC_ROT_Y),
  mQuadraticRotZ(PARAM_DEFAULT_QUADRATIC_ROT_Z),
  mThrottleLinX(PARAM_DEFAULT_THROTTLE_LIN_X),
  mThrottleLinY(PARAM_DEFAULT_THROTTLE_LIN_Y),
  mThrottleLinZ(PARAM_DEFAULT_THROTTLE_LIN_Z),
  mThrottleRotX(PARAM_DEFAULT_THROTTLE_ROT_X),
  mThrottleRotY(PARAM_DEFAULT_THROTTLE_ROT_Y),
  mThrottleRotZ(PARAM_DEFAULT_THROTTLE_ROT_Z),
  mSpinner(NULL),
  mIsInitialised(false) {

  //Sanity checks: min <= max, 0 <= max, 0 >= min
  if (mMinLinX > mMaxLinX || 0 > mMaxLinX || 0 < mMinLinX) {
    ROS_WARN("TeleopSinkTwistCallbackRos: invalid lin X min and/or max values, resetting both to defaults");
    mMaxLinX = PARAM_DEFAULT_MAX_LIN_X;
    mMinLinX = PARAM_DEFAULT_MIN_LIN_X;
  }
  if (mMinLinY > mMaxLinY || 0 > mMaxLinY || 0 < mMinLinY) {
    ROS_WARN("TeleopSinkTwistCallbackRos: invalid lin Y min and/or max values, resetting both to defaults");
    mMaxLinY = PARAM_DEFAULT_MAX_LIN_Y;
    mMinLinY = PARAM_DEFAULT_MIN_LIN_Y;
  }
  if (mMinLinZ > mMaxLinZ || 0 > mMaxLinZ || 0 < mMinLinZ) {
    ROS_WARN("TeleopSinkTwistCallbackRos: invalid lin Z min and/or max values, resetting both to defaults");
    mMaxLinZ = PARAM_DEFAULT_MAX_LIN_Z;
    mMinLinZ = PARAM_DEFAULT_MIN_LIN_Z;
  }
  if (mMinRotX > mMaxRotX || 0 > mMaxRotX || 0 < mMinRotX) {
    ROS_WARN("TeleopSinkTwistCallbackRos: invalid rot X min and/or max values, resetting both to defaults");
    mMaxRotX = PARAM_DEFAULT_MAX_ROT_X;
    mMinRotX = PARAM_DEFAULT_MIN_ROT_X;
  }
  if (mMinRotY > mMaxRotY || 0 > mMaxRotY || 0 < mMinRotY) {
    ROS_WARN("TeleopSinkTwistCallbackRos: invalid rot Y min and/or max values, resetting both to defaults");
    mMaxRotY = PARAM_DEFAULT_MAX_ROT_Y;
    mMinRotY = PARAM_DEFAULT_MIN_ROT_Y;
  }
  if (mMinRotZ > mMaxRotZ || 0 > mMaxRotZ || 0 < mMinRotZ) {
    ROS_WARN("TeleopSinkTwistCallbackRos: invalid rot Z min and/or max values, resetting both to defaults");
    mMaxRotZ = PARAM_DEFAULT_MAX_ROT_Z;
    mMinRotZ = PARAM_DEFAULT_MIN_ROT_Z;
  }
}
//=============================================================================
TeleopSinkTwistNode::~TeleopSinkTwistNode() {
  //Shutdown (should always succeed)
  if (!shutdown()) {
    ROS_WARN("TeleopSinkTwistNode::~TeleopSinkTwistNode: ignoring error in shutdown");
  }
}
//=============================================================================
bool TeleopSinkTwistNode::init(int argc, char** argv, std::string nodeName, uint32_t rosInitOptions) {
  //Lock access to is initialised
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //If already initialised shutdown first (shutdown should always succeed)
  if (mIsInitialised && !shutdown()) {
    ROS_WARN("TeleopSinkTwistNode::init: ignoring error in shutdown()");
  }

  //Init node
  try {
    ros::init(argc, argv, nodeName, rosInitOptions);
  } catch(ros::InvalidNodeNameException& e) {
    ROS_ERROR("TeleopSinkTwistNode::init: error initialising node");
    return false;
  }

  //Start node manually to avoid node shutdown when last node handle is destroyed
  ros::start();

  //Init parameters
  try {
    //Create private node handle for parameters
    ros::NodeHandle nodeHandlePrivate("~");

    //Read parameters and/or set default values
    nodeHandlePrivate.param(PARAM_KEY_TELEOP_TOPIC,    mTeleopTopic,   nodeName + "/" + PARAM_DEFAULT_TELEOP_TOPIC);
    nodeHandlePrivate.param(PARAM_KEY_TWIST_TOPIC,     mTwistTopic,    nodeName + "/" + PARAM_DEFAULT_TWIST_TOPIC);
    nodeHandlePrivate.param(PARAM_KEY_HAS_LIN_X,       mHasLinX,       PARAM_DEFAULT_HAS_LIN_X);
    nodeHandlePrivate.param(PARAM_KEY_HAS_LIN_Y,       mHasLinY,       PARAM_DEFAULT_HAS_LIN_Y);
    nodeHandlePrivate.param(PARAM_KEY_HAS_LIN_Z,       mHasLinZ,       PARAM_DEFAULT_HAS_LIN_Z);
    nodeHandlePrivate.param(PARAM_KEY_HAS_ROT_X,       mHasRotX,       PARAM_DEFAULT_HAS_ROT_X);
    nodeHandlePrivate.param(PARAM_KEY_HAS_ROT_Y,       mHasRotY,       PARAM_DEFAULT_HAS_ROT_Y);
    nodeHandlePrivate.param(PARAM_KEY_HAS_ROT_Z,       mHasRotZ,       PARAM_DEFAULT_HAS_ROT_Z);
    nodeHandlePrivate.param(PARAM_KEY_MIN_LIN_X,       mMinLinX,       PARAM_DEFAULT_MIN_LIN_X);
    nodeHandlePrivate.param(PARAM_KEY_MIN_LIN_Y,       mMinLinY,       PARAM_DEFAULT_MIN_LIN_Y);
    nodeHandlePrivate.param(PARAM_KEY_MIN_LIN_Z,       mMinLinZ,       PARAM_DEFAULT_MIN_LIN_Z);
    nodeHandlePrivate.param(PARAM_KEY_MIN_ROT_X,       mMinRotX,       PARAM_DEFAULT_MIN_ROT_X);
    nodeHandlePrivate.param(PARAM_KEY_MIN_ROT_Y,       mMinRotY,       PARAM_DEFAULT_MIN_ROT_Y);
    nodeHandlePrivate.param(PARAM_KEY_MIN_ROT_Z,       mMinRotZ,       PARAM_DEFAULT_MIN_ROT_Z);
    nodeHandlePrivate.param(PARAM_KEY_MAX_LIN_X,       mMaxLinX,       PARAM_DEFAULT_MAX_LIN_X);
    nodeHandlePrivate.param(PARAM_KEY_MAX_LIN_Y,       mMaxLinY,       PARAM_DEFAULT_MAX_LIN_Y);
    nodeHandlePrivate.param(PARAM_KEY_MAX_LIN_Z,       mMaxLinZ,       PARAM_DEFAULT_MAX_LIN_Z);
    nodeHandlePrivate.param(PARAM_KEY_MAX_ROT_X,       mMaxRotX,       PARAM_DEFAULT_MAX_ROT_X);
    nodeHandlePrivate.param(PARAM_KEY_MAX_ROT_Y,       mMaxRotY,       PARAM_DEFAULT_MAX_ROT_Y);
    nodeHandlePrivate.param(PARAM_KEY_MAX_ROT_Z,       mMaxRotZ,       PARAM_DEFAULT_MAX_ROT_Z);
    nodeHandlePrivate.param(PARAM_KEY_QUADRATIC_LIN_X, mQuadraticLinX, PARAM_DEFAULT_QUADRATIC_LIN_X);
    nodeHandlePrivate.param(PARAM_KEY_QUADRATIC_LIN_Y, mQuadraticLinY, PARAM_DEFAULT_QUADRATIC_LIN_Y);
    nodeHandlePrivate.param(PARAM_KEY_QUADRATIC_LIN_Z, mQuadraticLinZ, PARAM_DEFAULT_QUADRATIC_LIN_Z);
    nodeHandlePrivate.param(PARAM_KEY_QUADRATIC_ROT_X, mQuadraticRotX, PARAM_DEFAULT_QUADRATIC_ROT_X);
    nodeHandlePrivate.param(PARAM_KEY_QUADRATIC_ROT_Y, mQuadraticRotY, PARAM_DEFAULT_QUADRATIC_ROT_Y);
    nodeHandlePrivate.param(PARAM_KEY_QUADRATIC_ROT_Z, mQuadraticRotZ, PARAM_DEFAULT_QUADRATIC_ROT_Z);
    nodeHandlePrivate.param(PARAM_KEY_THROTTLE_LIN_X,  mThrottleLinX,  PARAM_DEFAULT_THROTTLE_LIN_X);
    nodeHandlePrivate.param(PARAM_KEY_THROTTLE_LIN_Y,  mThrottleLinY,  PARAM_DEFAULT_THROTTLE_LIN_Y);
    nodeHandlePrivate.param(PARAM_KEY_THROTTLE_LIN_Z,  mThrottleLinZ,  PARAM_DEFAULT_THROTTLE_LIN_Z);
    nodeHandlePrivate.param(PARAM_KEY_THROTTLE_ROT_X,  mThrottleRotX,  PARAM_DEFAULT_THROTTLE_ROT_X);
    nodeHandlePrivate.param(PARAM_KEY_THROTTLE_ROT_Y,  mThrottleRotY,  PARAM_DEFAULT_THROTTLE_ROT_Y);
    nodeHandlePrivate.param(PARAM_KEY_THROTTLE_ROT_Z,  mThrottleRotZ,  PARAM_DEFAULT_THROTTLE_ROT_Z);

    //Advertise all parameters to allow introspection
    nodeHandlePrivate.setParam(PARAM_KEY_TELEOP_TOPIC,    mTeleopTopic);
    nodeHandlePrivate.setParam(PARAM_KEY_TWIST_TOPIC,     mTwistTopic);
    nodeHandlePrivate.setParam(PARAM_KEY_HAS_LIN_X,       mHasLinX);
    nodeHandlePrivate.setParam(PARAM_KEY_HAS_LIN_Y,       mHasLinY);
    nodeHandlePrivate.setParam(PARAM_KEY_HAS_LIN_Z,       mHasLinZ);
    nodeHandlePrivate.setParam(PARAM_KEY_HAS_ROT_X,       mHasRotX);
    nodeHandlePrivate.setParam(PARAM_KEY_HAS_ROT_Y,       mHasRotY);
    nodeHandlePrivate.setParam(PARAM_KEY_HAS_ROT_Z,       mHasRotZ);
    nodeHandlePrivate.setParam(PARAM_KEY_MIN_LIN_X,       mMinLinX);
    nodeHandlePrivate.setParam(PARAM_KEY_MIN_LIN_Y,       mMinLinY);
    nodeHandlePrivate.setParam(PARAM_KEY_MIN_LIN_Z,       mMinLinZ);
    nodeHandlePrivate.setParam(PARAM_KEY_MIN_ROT_X,       mMinRotX);
    nodeHandlePrivate.setParam(PARAM_KEY_MIN_ROT_Y,       mMinRotY);
    nodeHandlePrivate.setParam(PARAM_KEY_MIN_ROT_Z,       mMinRotZ);
    nodeHandlePrivate.setParam(PARAM_KEY_MAX_LIN_X,       mMaxLinX);
    nodeHandlePrivate.setParam(PARAM_KEY_MAX_LIN_Y,       mMaxLinY);
    nodeHandlePrivate.setParam(PARAM_KEY_MAX_LIN_Z,       mMaxLinZ);
    nodeHandlePrivate.setParam(PARAM_KEY_MAX_ROT_X,       mMaxRotX);
    nodeHandlePrivate.setParam(PARAM_KEY_MAX_ROT_Y,       mMaxRotY);
    nodeHandlePrivate.setParam(PARAM_KEY_MAX_ROT_Z,       mMaxRotZ);
    nodeHandlePrivate.setParam(PARAM_KEY_QUADRATIC_LIN_X, mQuadraticLinX);
    nodeHandlePrivate.setParam(PARAM_KEY_QUADRATIC_LIN_Y, mQuadraticLinY);
    nodeHandlePrivate.setParam(PARAM_KEY_QUADRATIC_LIN_Z, mQuadraticLinZ);
    nodeHandlePrivate.setParam(PARAM_KEY_QUADRATIC_ROT_X, mQuadraticRotX);
    nodeHandlePrivate.setParam(PARAM_KEY_QUADRATIC_ROT_Y, mQuadraticRotY);
    nodeHandlePrivate.setParam(PARAM_KEY_QUADRATIC_ROT_Z, mQuadraticRotZ);
    nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_LIN_X,  mThrottleLinX);
    nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_LIN_Y,  mThrottleLinY);
    nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_LIN_Z,  mThrottleLinZ);
    nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_ROT_X,  mThrottleRotX);
    nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_ROT_Y,  mThrottleRotY);
    nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_ROT_Z,  mThrottleRotZ);
  } catch (ros::InvalidNameException& e) {
    ROS_ERROR("TeleopSinkTwistNode::init: error initialising parameters");
    ros::shutdown();
    return false;
  }

  //Create publisher and subscriber
  try {
    //Create relative node handle for topics
    ros::NodeHandle nodeHandleRelative = ros::NodeHandle();

    //Create publisher with buffer size set to 1 and latching on.  The publisher
    //should basically just always contain the latest state.
    mPublisher = nodeHandleRelative.advertise<geometry_msgs::Twist>(mTwistTopic, 1, true);

    //Subscribe to teleop topic
    mSubscriber = nodeHandleRelative.subscribe(mTeleopTopic, 1, &TeleopSinkTwistNode::updated, this);
  } catch (ros::InvalidNameException& e) {
    ROS_ERROR("TeleopSinkTwistNode::init: error during topic registration (InvalidNameException)");
    ros::shutdown();
    return false;
  } catch (ros::ConflictingSubscriptionException& e) {
    ROS_ERROR("TeleopSinkTwistNode::init: error during topic registration (ConflictingSubscriptionException)");
    ros::shutdown();
    return false;
  }

  //Create and start single-threaded asynchronous spinner to handle incoming
  //ROS messages via our sole subscriber.  Use only one thread, since the
  //callback method is not thread-safe.
  mSpinner = new ros::AsyncSpinner(1);
  mSpinner->start();

  //Update init done flag
  mIsInitialised = true;

  //Return result
  return true;
}
//=============================================================================
bool TeleopSinkTwistNode::shutdown() {
  //Lock access to is initialised
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //Check if done
  if (!mIsInitialised) {
    return true;
  }

  //Stop the subscriber so we don't get any more teleop events
  mSubscriber.shutdown();

  //Delete the spinner
  delete mSpinner;

  //Zero message
  mTwistMsg.linear.x = 0.0;
  mTwistMsg.linear.y = 0.0;
  mTwistMsg.linear.z = 0.0;
  mTwistMsg.angular.x = 0.0;
  mTwistMsg.angular.y = 0.0;
  mTwistMsg.angular.z = 0.0;

  //Publish zero message
  mPublisher.publish(mTwistMsg);

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
void TeleopSinkTwistNode::updated(const teleop_msgs::State& teleopStateMsg) {
  //Convert teleop state message to twist message and publish it
  if (teleopStateToTwist(&teleopStateMsg, &mTwistMsg)) {
    mPublisher.publish(mTwistMsg);
  } else {
    ROS_WARN("updated: ignoring problem converting teleop state to twist");
  }
}
//=============================================================================
bool TeleopSinkTwistNode::teleopStateToTwist(const teleop_msgs::State* const teleopStateMsg,
                                             geometry_msgs::Twist* const twistMsg) {
  //Sanity check parameters
  if (NULL == teleopStateMsg || NULL == twistMsg) {
    ROS_ERROR("teleopStateToTwist: NULL parameter");
    return false;
  }

  //Get indices for each relevant axis in teleop state
  int indexLinX = -1;
  int indexLinY = -1;
  int indexLinZ = -1;
  int indexRotX = -1;
  int indexRotY = -1;
  int indexRotZ = -1;
  int indexThrottle = -1;
  for (size_t i = 0; i < teleopStateMsg->axes.size(); i++) {
    switch (teleopStateMsg->axes[i].type) {
      case teleop::TELEOP_AXIS_TYPE_LIN_X:    indexLinX = i;     break;
      case teleop::TELEOP_AXIS_TYPE_LIN_Y:    indexLinY = i;     break;
      case teleop::TELEOP_AXIS_TYPE_LIN_Z:    indexLinZ = i;     break;
      case teleop::TELEOP_AXIS_TYPE_ROT_X:    indexRotX = i;     break;
      case teleop::TELEOP_AXIS_TYPE_ROT_Y:    indexRotY = i;     break;
      case teleop::TELEOP_AXIS_TYPE_ROT_Z:    indexRotZ = i;     break;
      case teleop::TELEOP_AXIS_TYPE_THROTTLE: indexThrottle = i; break;
      default:
        //Ignore other axes
        break;
    }
  }

  //Find throttle value
  double throttle = 1.0;
  if (-1 != indexThrottle) {
    throttle = teleopStateMsg->axes[indexThrottle].value;
  }

  //Set applicable twist message values for each axis using teleop axis values.
  //Teleop axes can be mapped to various twist axes, depending on which axes
  //the sink object has and which axes the teleop source provides.  Each teleop
  //axis is mapped according to the following priority scheme.
  //
  // ---------------
  // teleop -> twist
  // ---------------
  // LIN_X  -> linear.x (otherwise angular.y or angular.z)
  // LIN_Y  -> linear.y (otherwise angular.x or angular.z)
  // LIN_Z  -> linear.z (otherwise angular.z)
  // ROT_X  -> angular.x (otherwise linear.y or linear.z)
  // ROT_Y  -> angular.y (otherwise linear.x or linear.z)
  // ROT_Z  -> angular.z (otherwise linear.z)
  // ---------------

  //Handle lin X
  if (-1 != indexLinX) {
    if (mHasLinX) {
      twistMsg->linear.x = teleopToTwistLinX(throttle, teleopStateMsg->axes[indexLinX].value);
    } else if (mHasRotY && -1 == indexRotY) {
      twistMsg->angular.y = teleopToTwistRotY(throttle, teleopStateMsg->axes[indexLinX].value);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(throttle, teleopStateMsg->axes[indexLinX].value);
    }
  }

  //Handle lin Y
  if (-1 != indexLinY) {
    if (mHasLinY) {
      twistMsg->linear.y = teleopToTwistLinY(throttle, teleopStateMsg->axes[indexLinY].value);
    } else if (mHasRotX && -1 == indexRotX) {
      twistMsg->angular.x = teleopToTwistRotX(throttle, teleopStateMsg->axes[indexLinY].value);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(throttle, teleopStateMsg->axes[indexLinY].value);
    }
  }

  //Handle lin Z
  if (-1 != indexLinZ) {
    if (mHasLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(throttle, teleopStateMsg->axes[indexLinZ].value);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(throttle, teleopStateMsg->axes[indexLinZ].value);
    }
  }

  //Handle rot X
  if (-1 != indexRotX) {
    if (mHasRotX) {
      twistMsg->angular.x = teleopToTwistRotX(throttle, teleopStateMsg->axes[indexRotX].value);
    } else if (mHasLinY && -1 == indexLinY) {
      twistMsg->linear.y = teleopToTwistLinY(throttle, teleopStateMsg->axes[indexRotX].value);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(throttle, teleopStateMsg->axes[indexRotX].value);
    }
  }

  //Handle rot Y
  if (-1 != indexRotY) {
    if (mHasRotY) {
      twistMsg->angular.y = teleopToTwistRotY(throttle, teleopStateMsg->axes[indexRotY].value);
    } else if (mHasLinX && -1 == indexLinX) {
      twistMsg->linear.x = teleopToTwistLinX(throttle, teleopStateMsg->axes[indexRotY].value);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(throttle, teleopStateMsg->axes[indexRotY].value);
    }
  }

  //Handle rot Z
  if (-1 != indexRotZ) {
    if (mHasRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(throttle, teleopStateMsg->axes[indexRotZ].value);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(throttle, teleopStateMsg->axes[indexRotZ].value);
    }
  }

  //Return result
  return true;
}
//=============================================================================
teleop::TeleopAxisValue TeleopSinkTwistNode::applyQuadratic(bool enabled, teleop::TeleopAxisValue teleopAxisValue) {
  if (enabled) {
    if (0.0 <= teleopAxisValue) {
      return (teleopAxisValue * teleopAxisValue);
    } else {
      return -(teleopAxisValue * teleopAxisValue);
    }
  } else {
    return teleopAxisValue;
  }
}
//=============================================================================
teleop::TeleopAxisValue TeleopSinkTwistNode::applyThrottle(bool enabled,
                                                           double throttle,
                                                           teleop::TeleopAxisValue teleopAxisValue) {
  if (enabled) {
    return (throttle * teleopAxisValue);
  } else {
    return teleopAxisValue;
  }
}
//=============================================================================
double TeleopSinkTwistNode::teleopToTwistLinX(double throttle, teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleLinX, throttle, (applyQuadratic(mQuadraticLinX, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxLinX;
  } else {
    return -level * mMinLinX;
  }
}
//=============================================================================
double TeleopSinkTwistNode::teleopToTwistLinY(double throttle,teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleLinY, throttle, (applyQuadratic(mQuadraticLinY, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxLinY;
  } else {
    return -level * mMinLinY;
  }
}
//=============================================================================
double TeleopSinkTwistNode::teleopToTwistLinZ(double throttle, teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleLinZ, throttle, (applyQuadratic(mQuadraticLinZ, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxLinZ;
  } else {
    return -level * mMinLinZ;
  }
}
//=============================================================================
double TeleopSinkTwistNode::teleopToTwistRotX(double throttle, teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleRotX, throttle, (applyQuadratic(mQuadraticRotX, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxRotX;
  } else {
    return -level * mMinRotX;
  }
}
//=============================================================================
double TeleopSinkTwistNode::teleopToTwistRotY(double throttle, teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleRotY, throttle, (applyQuadratic(mQuadraticRotY, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxRotY;
  } else {
    return -level * mMinRotY;
  }
}
//=============================================================================
double TeleopSinkTwistNode::teleopToTwistRotZ(double throttle, teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleRotZ, throttle, (applyQuadratic(mQuadraticRotZ, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxRotZ;
  } else {
    return -level * mMinRotZ;
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
             TeleopSinkTwistNode::PARAM_KEY_TELEOP_TOPIC,
             (nodeName + "/" + TeleopSinkTwistNode::PARAM_DEFAULT_TELEOP_TOPIC).c_str());
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_TWIST_TOPIC,
             (nodeName + "/" + TeleopSinkTwistNode::PARAM_DEFAULT_TWIST_TOPIC).c_str());
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_HAS_LIN_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_HAS_LIN_X ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_HAS_LIN_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_HAS_LIN_Y ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_HAS_LIN_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_HAS_LIN_Z ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_HAS_ROT_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_HAS_ROT_X ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_HAS_ROT_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_HAS_ROT_Y ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_HAS_ROT_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_HAS_ROT_Z ? "true" : "false");
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MIN_LIN_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_MIN_LIN_X);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MIN_LIN_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_MIN_LIN_Y);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MIN_LIN_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_MIN_LIN_Z);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MIN_ROT_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_MIN_ROT_X);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MIN_ROT_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_MIN_ROT_Y);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MIN_ROT_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_MIN_ROT_Z);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MAX_LIN_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_MAX_LIN_X);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MAX_LIN_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_MAX_LIN_Y);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MAX_LIN_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_MAX_LIN_Z);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MAX_ROT_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_MAX_ROT_X);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MAX_ROT_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_MAX_ROT_Y);
      printf("    _%s:=%lf\n",
             TeleopSinkTwistNode::PARAM_KEY_MAX_ROT_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_MAX_ROT_Z);
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_LIN_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_LIN_X ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_LIN_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_LIN_Y ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_LIN_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_LIN_Z ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_ROT_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_ROT_X ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_ROT_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_ROT_Y ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_QUADRATIC_ROT_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_QUADRATIC_ROT_Z ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_THROTTLE_LIN_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_LIN_X ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_THROTTLE_LIN_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_LIN_Y ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_THROTTLE_LIN_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_LIN_Z ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_THROTTLE_ROT_X,
             TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_ROT_X ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_THROTTLE_ROT_Y,
             TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_ROT_Y ? "true" : "false");
      printf("    _%s:=%s\n",
             TeleopSinkTwistNode::PARAM_KEY_THROTTLE_ROT_Z,
             TeleopSinkTwistNode::PARAM_DEFAULT_THROTTLE_ROT_Z ? "true" : "false");
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
  TeleopSinkTwistNode node;

  //Init node object
  if (!node.init(argc, argv, nodeName, ros::init_options::NoSigintHandler)) {
    ROS_ERROR("main: error initialising node");
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
