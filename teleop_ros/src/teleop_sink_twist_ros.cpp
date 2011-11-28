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
#include <csignal>
#include <cstdio>




//=============================================================================
//Defines
//=============================================================================

/**@{ Parameter keys */
#define PARAM_KEY_TELEOP_TOPIC          "teleop_topic"
#define PARAM_KEY_TWIST_TOPIC           "twist_topic"

#define PARAM_KEY_HAS_LIN_X             "has_lin_x"
#define PARAM_KEY_HAS_LIN_Y             "has_lin_y"
#define PARAM_KEY_HAS_LIN_Z             "has_lin_z"
#define PARAM_KEY_HAS_ROT_X             "has_rot_x"
#define PARAM_KEY_HAS_ROT_Y             "has_rot_y"
#define PARAM_KEY_HAS_ROT_Z             "has_rot_z"

#define PARAM_KEY_MIN_LIN_X             "min_lin_x"
#define PARAM_KEY_MIN_LIN_Y             "min_lin_y"
#define PARAM_KEY_MIN_LIN_Z             "min_lin_z"
#define PARAM_KEY_MIN_ROT_X             "min_rot_x"
#define PARAM_KEY_MIN_ROT_Y             "min_rot_y"
#define PARAM_KEY_MIN_ROT_Z             "min_rot_z"

#define PARAM_KEY_MAX_LIN_X             "max_lin_x"
#define PARAM_KEY_MAX_LIN_Y             "max_lin_y"
#define PARAM_KEY_MAX_LIN_Z             "max_lin_z"
#define PARAM_KEY_MAX_ROT_X             "max_rot_x"
#define PARAM_KEY_MAX_ROT_Y             "max_rot_y"
#define PARAM_KEY_MAX_ROT_Z             "max_rot_z"

#define PARAM_KEY_EXP_LIN_X             "exp_lin_x"
#define PARAM_KEY_EXP_LIN_Y             "exp_lin_y"
#define PARAM_KEY_EXP_LIN_Z             "exp_lin_z"
#define PARAM_KEY_EXP_ROT_X             "exp_rot_x"
#define PARAM_KEY_EXP_ROT_Y             "exp_rot_y"
#define PARAM_KEY_EXP_ROT_Z             "exp_rot_z"

#define PARAM_KEY_THROTTLE_LIN_X        "throttle_lin_x"
#define PARAM_KEY_THROTTLE_LIN_Y        "throttle_lin_y"
#define PARAM_KEY_THROTTLE_LIN_Z        "throttle_lin_z"
#define PARAM_KEY_THROTTLE_ROT_X        "throttle_rot_x"
#define PARAM_KEY_THROTTLE_ROT_Y        "throttle_rot_y"
#define PARAM_KEY_THROTTLE_ROT_Z        "throttle_rot_z"
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TELEOP_TOPIC      "teleop"
#define PARAM_DEFAULT_TWIST_TOPIC       "cmd_vel"

#define PARAM_DEFAULT_HAS_LIN_X         0
#define PARAM_DEFAULT_HAS_LIN_Y         0
#define PARAM_DEFAULT_HAS_LIN_Z         0
#define PARAM_DEFAULT_HAS_ROT_X         0
#define PARAM_DEFAULT_HAS_ROT_Y         0
#define PARAM_DEFAULT_HAS_ROT_Z         0

#define PARAM_DEFAULT_MIN_LIN_X         (-0.5)
#define PARAM_DEFAULT_MIN_LIN_Y         (-0.5)
#define PARAM_DEFAULT_MIN_LIN_Z         (-0.5)
#define PARAM_DEFAULT_MIN_ROT_X         (-0.8)
#define PARAM_DEFAULT_MIN_ROT_Y         (-0.8)
#define PARAM_DEFAULT_MIN_ROT_Z         (-0.8)

#define PARAM_DEFAULT_MAX_LIN_X         (0.5)
#define PARAM_DEFAULT_MAX_LIN_Y         (0.5)
#define PARAM_DEFAULT_MAX_LIN_Z         (0.5)
#define PARAM_DEFAULT_MAX_ROT_X         (0.8)
#define PARAM_DEFAULT_MAX_ROT_Y         (0.8)
#define PARAM_DEFAULT_MAX_ROT_Z         (0.8)

#define PARAM_DEFAULT_EXP_LIN_X         1
#define PARAM_DEFAULT_EXP_LIN_Y         1
#define PARAM_DEFAULT_EXP_LIN_Z         1
#define PARAM_DEFAULT_EXP_ROT_X         0
#define PARAM_DEFAULT_EXP_ROT_Y         0
#define PARAM_DEFAULT_EXP_ROT_Z         0

#define PARAM_DEFAULT_THROTTLE_LIN_X    0
#define PARAM_DEFAULT_THROTTLE_LIN_Y    0
#define PARAM_DEFAULT_THROTTLE_LIN_Z    0
#define PARAM_DEFAULT_THROTTLE_ROT_X    0
#define PARAM_DEFAULT_THROTTLE_ROT_Y    0
#define PARAM_DEFAULT_THROTTLE_ROT_Z    0
/**@}*/




//=============================================================================
//Types
//=============================================================================

/**
 * Callback class used by this node to translate teleop state messages to twist
 * messages and publish them.  A publisher must be provided to the constructor.
 */
class TeleopSinkTwistCallbackRos  {

public:

  /**
   * Constructor.
   *
   *   @param publisher [in] - publisher for twist messages
   *   @param hasLinX [in] - true if sink has this axis
   *   @param hasLinY [in] - true if sink has this axis
   *   @param hasLinZ [in] - true if sink has this axis
   *   @param hasRotX [in] - true if sink has this axis
   *   @param hasRotY [in] - true if sink has this axis
   *   @param hasRotZ [in] - true if sink has this axis
   *   @param minLinX [in] - min value for this axis
   *   @param minLinY [in] - min value for this axis
   *   @param minLinZ [in] - min value for this axis
   *   @param minRotX [in] - min value for this axis
   *   @param minRotY [in] - min value for this axis
   *   @param minRotZ [in] - min value for this axis
   *   @param maxLinX [in] - max value for this axis
   *   @param maxLinY [in] - max value for this axis
   *   @param maxLinZ [in] - max value for this axis
   *   @param maxRotX [in] - max value for this axis
   *   @param maxRotY [in] - max value for this axis
   *   @param maxRotZ [in] - max value for this axis
   *   @param expLinX [in] - true if axis should be exponential
   *   @param expLinY [in] - true if axis should be exponential
   *   @param expLinZ [in] - true if axis should be exponential
   *   @param expRotX [in] - true if axis should be exponential
   *   @param expRotY [in] - true if axis should be exponential
   *   @param expRotZ [in] - true if axis should be exponential
   *   @param throttleLinX [in] - true if axis should be affected by throttle
   *   @param throttleLinY [in] - true if axis should be affected by throttle
   *   @param throttleLinZ [in] - true if axis should be affected by throttle
   *   @param throttleRotX [in] - true if axis should be affected by throttle
   *   @param throttleRotY [in] - true if axis should be affected by throttle
   *   @param throttleRotZ [in] - true if axis should be affected by throttle
   */
  TeleopSinkTwistCallbackRos(const ros::Publisher* const publisher,
                             bool hasLinX, bool hasLinY, bool hasLinZ,
                             bool hasRotX, bool hasRotY, bool hasRotZ,
                             double minLinX, double minLinY, double minLinZ,
                             double minRotX, double minRotY, double minRotZ,
                             double maxLinX, double maxLinY, double maxLinZ,
                             double maxRotX, double maxRotY, double maxRotZ,
                             bool expLinX, bool expLinY, bool expLinZ,
                             bool expRotX, bool expRotY, bool expRotZ,
                             bool throttleLinX, bool throttleLinY, bool throttleLinZ,
                             bool throttleRotX, bool throttleRotY, bool throttleRotZ);

  /**
   * Destructor.
   */
  ~TeleopSinkTwistCallbackRos();

  /**
   * Called when teleop topic is updated.  This converts the given teleop state
   * message to a twist message and publishes it.
   *
   *   @param teleopStateMsg [in] - received teleop state message
   */
  void updated(const teleop_msgs::State& teleopStateMsg);

private:

  /** Publisher given to constructor and used in updated() */
  const ros::Publisher* const mPublisher;

  /** Throttle value */
  double mThrottle;

  /**@{ Parameters for each twist axis */
  bool mHasLinX, mHasLinY, mHasLinZ, mHasRotX, mHasRotY, mHasRotZ;
  double mMinLinX, mMinLinY, mMinLinZ, mMinRotX, mMinRotY, mMinRotZ;
  double mMaxLinX, mMaxLinY, mMaxLinZ, mMaxRotX, mMaxRotY, mMaxRotZ;
  bool mExpLinX, mExpLinY, mExpLinZ, mExpRotX, mExpRotY, mExpRotZ;
  bool mThrottleLinX, mThrottleLinY, mThrottleLinZ, mThrottleRotX, mThrottleRotY, mThrottleRotZ;
  /**@}*/

  /**
   * Twist member avoids re-creation for each call to updated(), and allows
   * the message to be available from the destructor.
   */
  geometry_msgs::Twist mTwistMsg;

  /**
   * Convert teleop state to twist.
   *
   *  @param teleopStateMsg [in] - teleop state to convert
   *  @param twistMsg [out] - resulting twist message
   *
   *  @return true on success
   */
  bool teleopStateToTwist(const teleop_msgs::State* const teleopStateMsg, geometry_msgs::Twist* const twistMsg);

  /**
   * Apply exponential factor to teleop axis value if enabled
   *
   *   @param enabled [in] - true if enabled
   *   @param teleopAxisValue [in] - original teleop axis value
   *
   *   @return updated teleopAxisValue
   */
  teleop::TeleopAxisValue applyExponential(bool enabled, teleop::TeleopAxisValue teleopAxisValue);

  /**
   * Apply throttle factor to teleop axis value if enabled
   *
   *   @param enabled [in] - true if enabled
   *   @param teleopAxisValue [in] - original teleop axis value
   *
   *   @return updated teleopAxisValue
   */
  teleop::TeleopAxisValue applyThrottle(bool enabled, teleop::TeleopAxisValue teleopAxisValue);

  /**@{
   * Methods to convert normalised teleop axis values into unnormalised twist
   * values.  These methods use the min and max member values for each axis
   * to compute the resulting twist value.
   *
   *   @param teleopAxisValue [in] - teleop axis value to convert
   *
   *   @return twist value
   */
  double teleopToTwistLinX(teleop::TeleopAxisValue teleopAxisValue);
  double teleopToTwistLinY(teleop::TeleopAxisValue teleopAxisValue);
  double teleopToTwistLinZ(teleop::TeleopAxisValue teleopAxisValue);
  double teleopToTwistRotX(teleop::TeleopAxisValue teleopAxisValue);
  double teleopToTwistRotY(teleop::TeleopAxisValue teleopAxisValue);
  double teleopToTwistRotZ(teleop::TeleopAxisValue teleopAxisValue);
   /**@}*/

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
TeleopSinkTwistCallbackRos::TeleopSinkTwistCallbackRos(const ros::Publisher* const publisher,
                                                       bool hasLinX, bool hasLinY, bool hasLinZ,
                                                       bool hasRotX, bool hasRotY, bool hasRotZ,
                                                       double minLinX, double minLinY, double minLinZ,
                                                       double minRotX, double minRotY, double minRotZ,
                                                       double maxLinX, double maxLinY, double maxLinZ,
                                                       double maxRotX, double maxRotY, double maxRotZ,
                                                       bool expLinX, bool expLinY, bool expLinZ,
                                                       bool expRotX, bool expRotY, bool expRotZ,
                                                       bool throttleLinX, bool throttleLinY, bool throttleLinZ,
                                                       bool throttleRotX, bool throttleRotY, bool throttleRotZ) :
  mPublisher(publisher),
  mHasLinX(hasLinX),
  mHasLinY(hasLinY),
  mHasLinZ(hasLinZ),
  mHasRotX(hasRotX),
  mHasRotY(hasRotY),
  mHasRotZ(hasRotZ),
  mMinLinX(minLinX),
  mMinLinY(minLinY),
  mMinLinZ(minLinZ),
  mMinRotX(minRotX),
  mMinRotY(minRotY),
  mMinRotZ(minRotZ),
  mMaxLinX(maxLinX),
  mMaxLinY(maxLinY),
  mMaxLinZ(maxLinZ),
  mMaxRotX(maxRotX),
  mMaxRotY(maxRotY),
  mMaxRotZ(maxRotZ),
  mExpLinX(expLinX),
  mExpLinY(expLinY),
  mExpLinZ(expLinZ),
  mExpRotX(expRotX),
  mExpRotY(expRotY),
  mExpRotZ(expRotZ),
  mThrottleLinX(throttleLinX),
  mThrottleLinY(throttleLinY),
  mThrottleLinZ(throttleLinZ),
  mThrottleRotX(throttleRotX),
  mThrottleRotY(throttleRotY),
  mThrottleRotZ(throttleRotZ) {

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
TeleopSinkTwistCallbackRos::~TeleopSinkTwistCallbackRos() {
  //Sanity check publisher
  if (NULL == mPublisher) {
    ROS_ERROR("~TeleopSinkTwistCallbackRos: NULL publisher");
    return;
  }

  //Zero message
  mTwistMsg.linear.x = 0.0;
  mTwistMsg.linear.y = 0.0;
  mTwistMsg.linear.z = 0.0;
  mTwistMsg.angular.x = 0.0;
  mTwistMsg.angular.y = 0.0;
  mTwistMsg.angular.z = 0.0;

  //Publish zero message
  mPublisher->publish(mTwistMsg);
}
//=============================================================================
void TeleopSinkTwistCallbackRos::updated(const teleop_msgs::State& teleopStateMsg) {
  //Sanity check publisher
  if (NULL == mPublisher) {
    ROS_ERROR("updated: NULL publisher");
    quit();
    return;
  }

  //Convert teleop state message to twist message and publish it
  if (teleopStateToTwist(&teleopStateMsg, &mTwistMsg)) {
    mPublisher->publish(mTwistMsg);
  } else {
    ROS_ERROR("updated: error converting teleop state to twist");
    quit();
  }
}
//=============================================================================
bool TeleopSinkTwistCallbackRos::teleopStateToTwist(const teleop_msgs::State* const teleopStateMsg,
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

  //Find throttle value used to modify all directional axis values
  mThrottle = 1.0;
  if (-1 != indexThrottle) {
    mThrottle = teleopStateMsg->axes[indexThrottle].value;
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
      twistMsg->linear.x = teleopToTwistLinX(teleopStateMsg->axes[indexLinX].value);
    } else if (mHasRotY && -1 == indexRotY) {
      twistMsg->angular.y = teleopToTwistRotY(teleopStateMsg->axes[indexLinX].value);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexLinX].value);
    }
  }

  //Handle lin Y
  if (-1 != indexLinY) {
    if (mHasLinY) {
      twistMsg->linear.y = teleopToTwistLinY(teleopStateMsg->axes[indexLinY].value);
    } else if (mHasRotX && -1 == indexRotX) {
      twistMsg->angular.x = teleopToTwistRotX(teleopStateMsg->axes[indexLinY].value);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexLinY].value);
    }
  }

  //Handle lin Z
  if (-1 != indexLinZ) {
    if (mHasLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexLinZ].value);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexLinZ].value);
    }
  }

  //Handle rot X
  if (-1 != indexRotX) {
    if (mHasRotX) {
      twistMsg->angular.x = teleopToTwistRotX(teleopStateMsg->axes[indexRotX].value);
    } else if (mHasLinY && -1 == indexLinY) {
      twistMsg->linear.y = teleopToTwistLinY(teleopStateMsg->axes[indexRotX].value);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexRotX].value);
    }
  }

  //Handle rot Y
  if (-1 != indexRotY) {
    if (mHasRotY) {
      twistMsg->angular.y = teleopToTwistRotY(teleopStateMsg->axes[indexRotY].value);
    } else if (mHasLinX && -1 == indexLinX) {
      twistMsg->linear.x = teleopToTwistLinX(teleopStateMsg->axes[indexRotY].value);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexRotY].value);
    }
  }

  //Handle rot Z
  if (-1 != indexRotZ) {
    if (mHasRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexRotZ].value);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexRotZ].value);
    }
  }

  //Return result
  return true;
}
//=============================================================================
teleop::TeleopAxisValue TeleopSinkTwistCallbackRos::applyExponential(bool enabled,
                                                                     teleop::TeleopAxisValue teleopAxisValue) {
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
teleop::TeleopAxisValue TeleopSinkTwistCallbackRos::applyThrottle(bool enabled,
                                                                  teleop::TeleopAxisValue teleopAxisValue) {
  if (enabled) {
    return (mThrottle * teleopAxisValue);
  } else {
    return teleopAxisValue;
  }
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistLinX(teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleLinX, (applyExponential(mExpLinX, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxLinX;
  } else {
    return -level * mMinLinX;
  }
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistLinY(teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleLinY, (applyExponential(mExpLinY, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxLinY;
  } else {
    return -level * mMinLinY;
  }
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistLinZ(teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleLinZ, (applyExponential(mExpLinZ, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxLinZ;
  } else {
    return -level * mMinLinZ;
  }
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistRotX(teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleRotX, (applyExponential(mExpRotX, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxRotX;
  } else {
    return -level * mMinRotX;
  }
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistRotY(teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleRotY, (applyExponential(mExpRotY, teleopAxisValue)));
  if (0.0 <= teleopAxisValue) {
    return level * mMaxRotY;
  } else {
    return -level * mMinRotY;
  }
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistRotZ(teleop::TeleopAxisValue teleopAxisValue) {
  double level = applyThrottle(mThrottleRotZ, (applyExponential(mExpRotZ, teleopAxisValue)));
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
      printf("    _%s:=%s\n",    PARAM_KEY_TWIST_TOPIC,
                                 (std::string("<node>/") + std::string(PARAM_DEFAULT_TWIST_TOPIC)).c_str());
      printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_X,      PARAM_DEFAULT_HAS_LIN_X);
      printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_Y,      PARAM_DEFAULT_HAS_LIN_Y);
      printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_Z,      PARAM_DEFAULT_HAS_LIN_Z);
      printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_X,      PARAM_DEFAULT_HAS_ROT_X);
      printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_Y,      PARAM_DEFAULT_HAS_ROT_Y);
      printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_Z,      PARAM_DEFAULT_HAS_ROT_Z);
      printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_X,      PARAM_DEFAULT_MIN_LIN_X);
      printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_Y,      PARAM_DEFAULT_MIN_LIN_Y);
      printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_Z,      PARAM_DEFAULT_MIN_LIN_Z);
      printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_X,      PARAM_DEFAULT_MIN_ROT_X);
      printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_Y,      PARAM_DEFAULT_MIN_ROT_Y);
      printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_Z,      PARAM_DEFAULT_MIN_ROT_Z);
      printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_X,      PARAM_DEFAULT_MAX_LIN_X);
      printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_Y,      PARAM_DEFAULT_MAX_LIN_Y);
      printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_Z,      PARAM_DEFAULT_MAX_LIN_Z);
      printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_X,      PARAM_DEFAULT_MAX_ROT_X);
      printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_Y,      PARAM_DEFAULT_MAX_ROT_Y);
      printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_Z,      PARAM_DEFAULT_MAX_ROT_Z);
      printf("    _%s:=%d\n",    PARAM_KEY_EXP_LIN_X,      PARAM_DEFAULT_EXP_LIN_X);
      printf("    _%s:=%d\n",    PARAM_KEY_EXP_LIN_Y,      PARAM_DEFAULT_EXP_LIN_Y);
      printf("    _%s:=%d\n",    PARAM_KEY_EXP_LIN_Z,      PARAM_DEFAULT_EXP_LIN_Z);
      printf("    _%s:=%d\n",    PARAM_KEY_EXP_ROT_X,      PARAM_DEFAULT_EXP_ROT_X);
      printf("    _%s:=%d\n",    PARAM_KEY_EXP_ROT_Y,      PARAM_DEFAULT_EXP_ROT_Y);
      printf("    _%s:=%d\n",    PARAM_KEY_EXP_ROT_Z,      PARAM_DEFAULT_EXP_ROT_Z);
      printf("    _%s:=%d\n",    PARAM_KEY_THROTTLE_LIN_X, PARAM_DEFAULT_THROTTLE_LIN_X);
      printf("    _%s:=%d\n",    PARAM_KEY_THROTTLE_LIN_Y, PARAM_DEFAULT_THROTTLE_LIN_Y);
      printf("    _%s:=%d\n",    PARAM_KEY_THROTTLE_LIN_Z, PARAM_DEFAULT_THROTTLE_LIN_Z);
      printf("    _%s:=%d\n",    PARAM_KEY_THROTTLE_ROT_X, PARAM_DEFAULT_THROTTLE_ROT_X);
      printf("    _%s:=%d\n",    PARAM_KEY_THROTTLE_ROT_Y, PARAM_DEFAULT_THROTTLE_ROT_Y);
      printf("    _%s:=%d\n",    PARAM_KEY_THROTTLE_ROT_Z, PARAM_DEFAULT_THROTTLE_ROT_Z);
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
  //Check if we should just print usage information and quit
  if (printUsage(argc, argv)) {
    return 0;
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
  std::string twistTopic;
  int hasLinX, hasLinY, hasLinZ, hasRotX, hasRotY, hasRotZ;
  double maxLinX, maxLinY, maxLinZ, maxRotX, maxRotY, maxRotZ;
  double minLinX, minLinY, minLinZ, minRotX, minRotY, minRotZ;
  int expLinX, expLinY, expLinZ, expRotX, expRotY, expRotZ;
  int throttleLinX, throttleLinY, throttleLinZ, throttleRotX, throttleRotY, throttleRotZ;

  //Read parameters and set default values
  nodeHandlePrivate.param(PARAM_KEY_TELEOP_TOPIC,   teleopTopic,
                          nodeName + std::string("/") + std::string(PARAM_DEFAULT_TELEOP_TOPIC));
  nodeHandlePrivate.param(PARAM_KEY_TWIST_TOPIC,    twistTopic,
                          nodeName + std::string("/") + std::string(PARAM_DEFAULT_TWIST_TOPIC));
  nodeHandlePrivate.param(PARAM_KEY_HAS_LIN_X,      hasLinX,      PARAM_DEFAULT_HAS_LIN_X);
  nodeHandlePrivate.param(PARAM_KEY_HAS_LIN_Y,      hasLinY,      PARAM_DEFAULT_HAS_LIN_Y);
  nodeHandlePrivate.param(PARAM_KEY_HAS_LIN_Z,      hasLinZ,      PARAM_DEFAULT_HAS_LIN_Z);
  nodeHandlePrivate.param(PARAM_KEY_HAS_ROT_X,      hasRotX,      PARAM_DEFAULT_HAS_ROT_X);
  nodeHandlePrivate.param(PARAM_KEY_HAS_ROT_Y,      hasRotY,      PARAM_DEFAULT_HAS_ROT_Y);
  nodeHandlePrivate.param(PARAM_KEY_HAS_ROT_Z,      hasRotZ,      PARAM_DEFAULT_HAS_ROT_Z);
  nodeHandlePrivate.param(PARAM_KEY_MIN_LIN_X,      minLinX,      PARAM_DEFAULT_MIN_LIN_X);
  nodeHandlePrivate.param(PARAM_KEY_MIN_LIN_Y,      minLinY,      PARAM_DEFAULT_MIN_LIN_Y);
  nodeHandlePrivate.param(PARAM_KEY_MIN_LIN_Z,      minLinZ,      PARAM_DEFAULT_MIN_LIN_Z);
  nodeHandlePrivate.param(PARAM_KEY_MIN_ROT_X,      minRotX,      PARAM_DEFAULT_MIN_ROT_X);
  nodeHandlePrivate.param(PARAM_KEY_MIN_ROT_Y,      minRotY,      PARAM_DEFAULT_MIN_ROT_Y);
  nodeHandlePrivate.param(PARAM_KEY_MIN_ROT_Z,      minRotZ,      PARAM_DEFAULT_MIN_ROT_Z);
  nodeHandlePrivate.param(PARAM_KEY_MAX_LIN_X,      maxLinX,      PARAM_DEFAULT_MAX_LIN_X);
  nodeHandlePrivate.param(PARAM_KEY_MAX_LIN_Y,      maxLinY,      PARAM_DEFAULT_MAX_LIN_Y);
  nodeHandlePrivate.param(PARAM_KEY_MAX_LIN_Z,      maxLinZ,      PARAM_DEFAULT_MAX_LIN_Z);
  nodeHandlePrivate.param(PARAM_KEY_MAX_ROT_X,      maxRotX,      PARAM_DEFAULT_MAX_ROT_X);
  nodeHandlePrivate.param(PARAM_KEY_MAX_ROT_Y,      maxRotY,      PARAM_DEFAULT_MAX_ROT_Y);
  nodeHandlePrivate.param(PARAM_KEY_MAX_ROT_Z,      maxRotZ,      PARAM_DEFAULT_MAX_ROT_Z);
  nodeHandlePrivate.param(PARAM_KEY_EXP_LIN_X,      expLinX,      PARAM_DEFAULT_EXP_LIN_X);
  nodeHandlePrivate.param(PARAM_KEY_EXP_LIN_Y,      expLinY,      PARAM_DEFAULT_EXP_LIN_Y);
  nodeHandlePrivate.param(PARAM_KEY_EXP_LIN_Z,      expLinZ,      PARAM_DEFAULT_EXP_LIN_Z);
  nodeHandlePrivate.param(PARAM_KEY_EXP_ROT_X,      expRotX,      PARAM_DEFAULT_EXP_ROT_X);
  nodeHandlePrivate.param(PARAM_KEY_EXP_ROT_Y,      expRotY,      PARAM_DEFAULT_EXP_ROT_Y);
  nodeHandlePrivate.param(PARAM_KEY_EXP_ROT_Z,      expRotZ,      PARAM_DEFAULT_EXP_ROT_Z);
  nodeHandlePrivate.param(PARAM_KEY_THROTTLE_LIN_X, throttleLinX, PARAM_DEFAULT_THROTTLE_LIN_X);
  nodeHandlePrivate.param(PARAM_KEY_THROTTLE_LIN_Y, throttleLinY, PARAM_DEFAULT_THROTTLE_LIN_Y);
  nodeHandlePrivate.param(PARAM_KEY_THROTTLE_LIN_Z, throttleLinZ, PARAM_DEFAULT_THROTTLE_LIN_Z);
  nodeHandlePrivate.param(PARAM_KEY_THROTTLE_ROT_X, throttleRotX, PARAM_DEFAULT_THROTTLE_ROT_X);
  nodeHandlePrivate.param(PARAM_KEY_THROTTLE_ROT_Y, throttleRotY, PARAM_DEFAULT_THROTTLE_ROT_Y);
  nodeHandlePrivate.param(PARAM_KEY_THROTTLE_ROT_Z, throttleRotZ, PARAM_DEFAULT_THROTTLE_ROT_Z);

  //Advertise all parameters to allow introspection
  nodeHandlePrivate.setParam(PARAM_KEY_TELEOP_TOPIC,   teleopTopic);
  nodeHandlePrivate.setParam(PARAM_KEY_TWIST_TOPIC,    twistTopic);
  nodeHandlePrivate.setParam(PARAM_KEY_HAS_LIN_X,      hasLinX);
  nodeHandlePrivate.setParam(PARAM_KEY_HAS_LIN_Y,      hasLinY);
  nodeHandlePrivate.setParam(PARAM_KEY_HAS_LIN_Z,      hasLinZ);
  nodeHandlePrivate.setParam(PARAM_KEY_HAS_ROT_X,      hasRotX);
  nodeHandlePrivate.setParam(PARAM_KEY_HAS_ROT_Y,      hasRotY);
  nodeHandlePrivate.setParam(PARAM_KEY_HAS_ROT_Z,      hasRotZ);
  nodeHandlePrivate.setParam(PARAM_KEY_MIN_LIN_X,      minLinX);
  nodeHandlePrivate.setParam(PARAM_KEY_MIN_LIN_Y,      minLinY);
  nodeHandlePrivate.setParam(PARAM_KEY_MIN_LIN_Z,      minLinZ);
  nodeHandlePrivate.setParam(PARAM_KEY_MIN_ROT_X,      minRotX);
  nodeHandlePrivate.setParam(PARAM_KEY_MIN_ROT_Y,      minRotY);
  nodeHandlePrivate.setParam(PARAM_KEY_MIN_ROT_Z,      minRotZ);
  nodeHandlePrivate.setParam(PARAM_KEY_MAX_LIN_X,      maxLinX);
  nodeHandlePrivate.setParam(PARAM_KEY_MAX_LIN_Y,      maxLinY);
  nodeHandlePrivate.setParam(PARAM_KEY_MAX_LIN_Z,      maxLinZ);
  nodeHandlePrivate.setParam(PARAM_KEY_MAX_ROT_X,      maxRotX);
  nodeHandlePrivate.setParam(PARAM_KEY_MAX_ROT_Y,      maxRotY);
  nodeHandlePrivate.setParam(PARAM_KEY_MAX_ROT_Z,      maxRotZ);
  nodeHandlePrivate.setParam(PARAM_KEY_EXP_LIN_X,      expLinX);
  nodeHandlePrivate.setParam(PARAM_KEY_EXP_LIN_Y,      expLinY);
  nodeHandlePrivate.setParam(PARAM_KEY_EXP_LIN_Z,      expLinZ);
  nodeHandlePrivate.setParam(PARAM_KEY_EXP_ROT_X,      expRotX);
  nodeHandlePrivate.setParam(PARAM_KEY_EXP_ROT_Y,      expRotY);
  nodeHandlePrivate.setParam(PARAM_KEY_EXP_ROT_Z,      expRotZ);
  nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_LIN_X, throttleLinX);
  nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_LIN_Y, throttleLinY);
  nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_LIN_Z, throttleLinZ);
  nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_ROT_X, throttleRotX);
  nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_ROT_Y, throttleRotY);
  nodeHandlePrivate.setParam(PARAM_KEY_THROTTLE_ROT_Z, throttleRotZ);

  //Create publisher with buffer size set to 1 and latching on.  The publisher
  //should basically just always contain the latest desired velocity.
  ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Twist>(twistTopic, 1, true);

  //Create callback using publisher and parameters.  The callback destructor
  //may want to publish a final message, so we use dynamic allocation here.
  //This means we can free this object and sleep a bit before the publisher and
  //node handle go out of scope and get destroyed.
  TeleopSinkTwistCallbackRos* callback = new TeleopSinkTwistCallbackRos(
      &publisher,
      0 != hasLinX, 0 != hasLinY, 0 != hasLinZ,
      0 != hasRotX, 0 != hasRotY, 0 != hasRotZ,
      minLinX, minLinY, minLinZ, minRotX,minRotY, minRotZ,
      maxLinX, maxLinY, maxLinZ, maxRotX, maxRotY, maxRotZ,
      0 != expLinX, 0 != expLinY, 0 != expLinZ,
      0 != expRotX, 0 != expRotY, 0 != expRotZ,
      0 != throttleLinX, 0 != throttleLinY, 0 != throttleLinZ,
      0 != throttleRotX, 0 != throttleRotY, 0 != throttleRotZ);

  //Subscribe to teleop topic using callback
  ros::Subscriber subscriber = nodeHandle.subscribe(teleopTopic, 1, &TeleopSinkTwistCallbackRos::updated, callback);

  //Start single-threaded asynchronous spinner to handle ROS events.  The callback can't handle multiple threads.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Wait for quit request
  boost::unique_lock<boost::mutex> quitRequestedLock(gQuitRequestedMutex);
  while(!gQuitRequested) {
    gQuitRequestedCondition.wait(quitRequestedLock);
  }
  quitRequestedLock.unlock();

  //Stop subscriber to ensure the callback is no longer used
  subscriber.shutdown();

  //Free callback object
  delete callback;

  //Sleep a bit to allow final messages to be published, if possible.  Use
  //boost thread sleep rather than ROS sleep since ROS sleep may try to use
  //a simulated clock which has been stopped at the same time as this node.
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  //Done
  return 0;
}
