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
#ifndef INCLUDE_TELEOP_SINK_TWIST_NODE_HPP
#define INCLUDE_TELEOP_SINK_TWIST_NODE_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <teleop_msgs/State.h>
#include <geometry_msgs/Twist.h>
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
  inline TeleopAxisValue applyQuadratic(bool enabled, TeleopAxisValue teleopAxisValue);

  /**
   * Apply throttle factor to teleop axis value if enabled.
   *
   *   @param enabled [in] - true if enabled
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - original teleop axis value
   *
   *   @return updated teleopAxisValue
   */
  inline TeleopAxisValue applyThrottle(bool enabled, double throttle, TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistLinX(double throttle, TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistLinY(double throttle, TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistLinZ(double throttle, TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistRotX(double throttle, TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistRotY(double throttle, TeleopAxisValue teleopAxisValue);

  /**
   * Convert normalised teleop axis value into twist value.  This method takes
   * all relevant data members into consideration when computing twist values.
   *
   *   @param throttle [in] - throttle value
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  inline double teleopToTwistRotZ(double throttle, TeleopAxisValue teleopAxisValue);

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SINK_TWIST_NODE_HPP
//=============================================================================
