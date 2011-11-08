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
#include <csignal>
#include <cmath>
#include <cstdio>




//=============================================================================
//Defines
//=============================================================================

/**@{ Parameter keys */
#define PARAM_KEY_TOPIC_TELEOP          "topic-teleop"
#define PARAM_KEY_TOPIC_TWIST           "topic-twist"

#define PARAM_KEY_EXPONENTIAL           "exponential"

#define PARAM_KEY_HAS_LIN_X             "has_lin_x"
#define PARAM_KEY_HAS_LIN_Y             "has_lin_y"
#define PARAM_KEY_HAS_LIN_Z             "has_lin_z"
#define PARAM_KEY_HAS_ROT_X             "has_rot_x"
#define PARAM_KEY_HAS_ROT_Y             "has_rot_y"
#define PARAM_KEY_HAS_ROT_Z             "has_rot_z"

#define PARAM_KEY_MAX_LIN_X             "max_lin_x"
#define PARAM_KEY_MAX_LIN_Y             "max_lin_y"
#define PARAM_KEY_MAX_LIN_Z             "max_lin_z"
#define PARAM_KEY_MAX_ROT_X             "max_rot_x"
#define PARAM_KEY_MAX_ROT_Y             "max_rot_y"
#define PARAM_KEY_MAX_ROT_Z             "max_rot_z"

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
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TOPIC_TELEOP      "teleop"
#define PARAM_DEFAULT_TOPIC_TWIST       "cmd_vel"

#define PARAM_DEFAULT_EXPONENTIAL       1

#define PARAM_DEFAULT_HAS_LIN_X         1
#define PARAM_DEFAULT_HAS_LIN_Y         1
#define PARAM_DEFAULT_HAS_LIN_Z         1
#define PARAM_DEFAULT_HAS_ROT_X         1
#define PARAM_DEFAULT_HAS_ROT_Y         1
#define PARAM_DEFAULT_HAS_ROT_Z         1

#define PARAM_DEFAULT_MIN_LIN_X         (-1.0)
#define PARAM_DEFAULT_MIN_LIN_Y         (-1.0)
#define PARAM_DEFAULT_MIN_LIN_Z         (-1.0)
#define PARAM_DEFAULT_MIN_ROT_X         (-M_PI)
#define PARAM_DEFAULT_MIN_ROT_Y         (-M_PI)
#define PARAM_DEFAULT_MIN_ROT_Z         (-M_PI)

#define PARAM_DEFAULT_MAX_LIN_X         (1.0)
#define PARAM_DEFAULT_MAX_LIN_Y         (1.0)
#define PARAM_DEFAULT_MAX_LIN_Z         (1.0)
#define PARAM_DEFAULT_MAX_ROT_X         (M_PI)
#define PARAM_DEFAULT_MAX_ROT_Y         (M_PI)
#define PARAM_DEFAULT_MAX_ROT_Z         (M_PI)
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
   *   @param exponential [in] - true if values should grow exponentially
   *   @param hasLinX [in] - true if sink has lin X axis
   *   @param hasLinY [in] - true if sink has lin Y axis
   *   @param hasLinZ [in] - true if sink has lin Z axis
   *   @param hasRotX [in] - true if sink has rot X axis
   *   @param hasRotY [in] - true if sink has rot Y axis
   *   @param hasRotZ [in] - true if sink has rot Z axis
   *   @param minLinX [in] - min value for lin X axis
   *   @param minLinY [in] - min value for lin Y axis
   *   @param minLinZ [in] - min value for lin Z axis
   *   @param minRotX [in] - min value for rot X axis
   *   @param minRotY [in] - min value for rot Y axis
   *   @param minRotZ [in] - min value for rot Z axis
   *   @param maxLinX [in] - max value for lin X axis
   *   @param maxLinY [in] - max value for lin Y axis
   *   @param maxLinZ [in] - max value for lin Z axis
   *   @param maxRotX [in] - max value for rot X axis
   *   @param maxRotY [in] - max value for rot Y axis
   *   @param maxRotZ [in] - max value for rot Z axis
   */
  TeleopSinkTwistCallbackRos(const ros::Publisher* const publisher, bool exponential,
                             bool hasLinX, bool hasLinY, bool hasLinZ,
                             bool hasRotX, bool hasRotY, bool hasRotZ,
                             double minLinX, double minLinY, double minLinZ,
                             double minRotX, double minRotY, double minRotZ,
                             double maxLinX, double maxLinY, double maxLinZ,
                             double maxRotX, double maxRotY, double maxRotZ);

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

  /** True if values should grow exponentially */
  bool mExponential;

  /**@{ Parameters for existence and range for each twist axis */
  bool mHasLinX, mHasLinY, mHasLinZ, mHasRotX, mHasRotY, mHasRotZ;
  double mMinLinX, mMinLinY, mMinLinZ, mMinRotX, mMinRotY, mMinRotZ;
  double mMaxLinX, mMaxLinY, mMaxLinZ, mMaxRotX, mMaxRotY, mMaxRotZ;
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
//Method definitions
//=============================================================================
TeleopSinkTwistCallbackRos::TeleopSinkTwistCallbackRos(const ros::Publisher* const publisher, bool exponential,
                                                       bool hasLinX, bool hasLinY, bool hasLinZ,
                                                       bool hasRotX, bool hasRotY, bool hasRotZ,
                                                       double minLinX, double minLinY, double minLinZ,
                                                       double minRotX, double minRotY, double minRotZ,
                                                       double maxLinX, double maxLinY, double maxLinZ,
                                                       double maxRotX, double maxRotY, double maxRotZ) :
  mPublisher(publisher),
  mExponential(exponential),
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
  mMaxRotZ(maxRotZ) {
}
//=============================================================================
TeleopSinkTwistCallbackRos::~TeleopSinkTwistCallbackRos() {
  //Sanity check publisher
  if (NULL == mPublisher) {
    ROS_ERROR("~TeleopSinkTwistCallbackRos: NULL publisher");
    return;
  }

  //Zero twist message
  mTwistMsg.linear.x = 0.0;
  mTwistMsg.linear.y = 0.0;
  mTwistMsg.linear.z = 0.0;
  mTwistMsg.angular.x = 0.0;
  mTwistMsg.angular.y = 0.0;
  mTwistMsg.angular.z = 0.0;

  //Publish zero twist message
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
      twistMsg->linear.x = teleopToTwistLinX(teleopStateMsg->axes[indexLinX].value * throttle);
    } else if (mHasRotY && -1 == indexRotY) {
      twistMsg->angular.y = teleopToTwistRotY(teleopStateMsg->axes[indexLinX].value * throttle);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexLinX].value * throttle);
    }
  }

  //Handle lin Y
  if (-1 != indexLinY) {
    if (mHasLinY) {
      twistMsg->linear.y = teleopToTwistLinY(teleopStateMsg->axes[indexLinY].value * throttle);
    } else if (mHasRotX && -1 == indexRotX) {
      twistMsg->angular.x = teleopToTwistRotX(teleopStateMsg->axes[indexLinY].value * throttle);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexLinY].value * throttle);
    }
  }

  //Handle lin Z
  if (-1 != indexLinZ) {
    if (mHasLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexLinZ].value * throttle);
    } else if (mHasRotZ && -1 == indexRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexLinZ].value * throttle);
    }
  }

  //Handle rot X
  if (-1 != indexRotX) {
    if (mHasRotX) {
      twistMsg->angular.x = teleopToTwistRotX(teleopStateMsg->axes[indexRotX].value * throttle);
    } else if (mHasLinY && -1 == indexLinY) {
      twistMsg->linear.y = teleopToTwistLinY(teleopStateMsg->axes[indexRotX].value * throttle);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexRotX].value * throttle);
    }
  }

  //Handle rot Y
  if (-1 != indexRotY) {
    if (mHasRotY) {
      twistMsg->angular.y = teleopToTwistRotY(teleopStateMsg->axes[indexRotY].value * throttle);
    } else if (mHasLinX && -1 == indexLinX) {
      twistMsg->linear.x = teleopToTwistLinX(teleopStateMsg->axes[indexRotY].value * throttle);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexRotY].value * throttle);
    }
  }

  //Handle rot Z
  if (-1 != indexRotZ) {
    if (mHasRotZ) {
      twistMsg->angular.z = teleopToTwistRotZ(teleopStateMsg->axes[indexRotZ].value * throttle);
    } else if (mHasLinZ && -1 == indexLinZ) {
      twistMsg->linear.z = teleopToTwistLinZ(teleopStateMsg->axes[indexRotZ].value * throttle);
    }
  }

  //Return result
  return true;
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistLinX(teleop::TeleopAxisValue teleopAxisValue) {
  if (mExponential) {
    teleopAxisValue *= teleopAxisValue;
  }
  return (mMinLinX + (teleopAxisValue*(mMaxLinX - mMinLinX)));
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistLinY(teleop::TeleopAxisValue teleopAxisValue) {
  if (mExponential) {
    teleopAxisValue *= teleopAxisValue;
  }
  return (mMinLinY + (teleopAxisValue*(mMaxLinY - mMinLinY)));
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistLinZ(teleop::TeleopAxisValue teleopAxisValue) {
  if (mExponential) {
    teleopAxisValue *= teleopAxisValue;
  }
  return (mMinLinZ + (teleopAxisValue*(mMaxLinZ - mMinLinZ)));
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistRotX(teleop::TeleopAxisValue teleopAxisValue) {
  if (mExponential) {
    teleopAxisValue *= teleopAxisValue;
  }
  return (mMinRotX + (teleopAxisValue*(mMaxRotX - mMinRotX)));
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistRotY(teleop::TeleopAxisValue teleopAxisValue) {
  if (mExponential) {
    teleopAxisValue *= teleopAxisValue;
  }
  return (mMinRotY + (teleopAxisValue*(mMaxRotY - mMinRotY)));
}
//=============================================================================
double TeleopSinkTwistCallbackRos::teleopToTwistRotZ(teleop::TeleopAxisValue teleopAxisValue) {
  if (mExponential) {
    teleopAxisValue *= teleopAxisValue;
  }
  return (mMinRotZ + (teleopAxisValue*(mMaxRotZ - mMinRotZ)));
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
bool printUsage(int argc, char** argv) {
  //Check for "-h" or "--help", if found, print usage
  for (int i = 1; i < argc; i++) {
    if ((0 == strcmp(argv[i], "-h")) || (0 == strcmp(argv[i], "--help"))) {
      std::printf("\n");
      std::printf("Usage:\n");
      std::printf("    %s [params]\n", basename(argv[0]));
      std::printf("\n");
      std::printf("Parameters and their default values\n");
      std::printf("    _%s:=%s\n",    PARAM_KEY_TOPIC_TELEOP, PARAM_DEFAULT_TOPIC_TELEOP);
      std::printf("    _%s:=%s\n",    PARAM_KEY_TOPIC_TWIST,  PARAM_DEFAULT_TOPIC_TWIST);
      std::printf("    _%s:=%d\n",    PARAM_KEY_EXPONENTIAL,  PARAM_DEFAULT_EXPONENTIAL);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_X,    PARAM_DEFAULT_HAS_LIN_X);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_Y,    PARAM_DEFAULT_HAS_LIN_Y);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_Z,    PARAM_DEFAULT_HAS_LIN_Z);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_X,    PARAM_DEFAULT_HAS_ROT_X);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_Y,    PARAM_DEFAULT_HAS_ROT_Y);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_Z,    PARAM_DEFAULT_HAS_ROT_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_X,    PARAM_DEFAULT_MIN_LIN_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_Y,    PARAM_DEFAULT_MIN_LIN_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_Z,    PARAM_DEFAULT_MIN_LIN_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_X,    PARAM_DEFAULT_MIN_ROT_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_Y,    PARAM_DEFAULT_MIN_ROT_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_Z,    PARAM_DEFAULT_MIN_ROT_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_X,    PARAM_DEFAULT_MAX_LIN_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_Y,    PARAM_DEFAULT_MAX_LIN_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_Z,    PARAM_DEFAULT_MAX_LIN_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_X,    PARAM_DEFAULT_MAX_ROT_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_Y,    PARAM_DEFAULT_MAX_ROT_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_Z,    PARAM_DEFAULT_MAX_ROT_Z);
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
  std::string topicTeleop;
  std::string topicTwist;
  int exponential;
  int hasLinX, hasLinY, hasLinZ, hasRotX, hasRotY, hasRotZ;
  double maxLinX, maxLinY, maxLinZ, maxRotX, maxRotY, maxRotZ;
  double minLinX, minLinY, minLinZ, minRotX, minRotY, minRotZ;

  //Read parameters and set default values
  nodeHandle.param(PARAM_KEY_TOPIC_TELEOP, topicTeleop, std::string(PARAM_DEFAULT_TOPIC_TELEOP));
  nodeHandle.param(PARAM_KEY_TOPIC_TWIST,  topicTwist,  std::string(PARAM_DEFAULT_TOPIC_TWIST));
  nodeHandle.param(PARAM_KEY_EXPONENTIAL,  exponential, PARAM_DEFAULT_EXPONENTIAL);
  nodeHandle.param(PARAM_KEY_HAS_LIN_X,    hasLinX,     PARAM_DEFAULT_HAS_LIN_X);
  nodeHandle.param(PARAM_KEY_HAS_LIN_Y,    hasLinY,     PARAM_DEFAULT_HAS_LIN_Y);
  nodeHandle.param(PARAM_KEY_HAS_LIN_Z,    hasLinZ,     PARAM_DEFAULT_HAS_LIN_Z);
  nodeHandle.param(PARAM_KEY_HAS_ROT_X,    hasRotX,     PARAM_DEFAULT_HAS_ROT_X);
  nodeHandle.param(PARAM_KEY_HAS_ROT_Y,    hasRotY,     PARAM_DEFAULT_HAS_ROT_Y);
  nodeHandle.param(PARAM_KEY_HAS_ROT_Z,    hasRotZ,     PARAM_DEFAULT_HAS_ROT_Z);
  nodeHandle.param(PARAM_KEY_MIN_LIN_X,    minLinX,     PARAM_DEFAULT_MIN_LIN_X);
  nodeHandle.param(PARAM_KEY_MIN_LIN_Y,    minLinY,     PARAM_DEFAULT_MIN_LIN_Y);
  nodeHandle.param(PARAM_KEY_MIN_LIN_Z,    minLinZ,     PARAM_DEFAULT_MIN_LIN_Z);
  nodeHandle.param(PARAM_KEY_MIN_ROT_X,    minRotX,     PARAM_DEFAULT_MIN_ROT_X);
  nodeHandle.param(PARAM_KEY_MIN_ROT_Y,    minRotY,     PARAM_DEFAULT_MIN_ROT_Y);
  nodeHandle.param(PARAM_KEY_MIN_ROT_Z,    minRotZ,     PARAM_DEFAULT_MIN_ROT_Z);
  nodeHandle.param(PARAM_KEY_MAX_LIN_X,    maxLinX,     PARAM_DEFAULT_MAX_LIN_X);
  nodeHandle.param(PARAM_KEY_MAX_LIN_Y,    maxLinY,     PARAM_DEFAULT_MAX_LIN_Y);
  nodeHandle.param(PARAM_KEY_MAX_LIN_Z,    maxLinZ,     PARAM_DEFAULT_MAX_LIN_Z);
  nodeHandle.param(PARAM_KEY_MAX_ROT_X,    maxRotX,     PARAM_DEFAULT_MAX_ROT_X);
  nodeHandle.param(PARAM_KEY_MAX_ROT_Y,    maxRotY,     PARAM_DEFAULT_MAX_ROT_Y);
  nodeHandle.param(PARAM_KEY_MAX_ROT_Z,    maxRotZ,     PARAM_DEFAULT_MAX_ROT_Z);

  //Advertise parameters for introspection
  nodeHandle.setParam(PARAM_KEY_TOPIC_TELEOP, topicTeleop);
  nodeHandle.setParam(PARAM_KEY_TOPIC_TWIST,  topicTwist);
  nodeHandle.setParam(PARAM_KEY_EXPONENTIAL,  exponential);
  nodeHandle.setParam(PARAM_KEY_HAS_LIN_X,    hasLinX);
  nodeHandle.setParam(PARAM_KEY_HAS_LIN_Y,    hasLinY);
  nodeHandle.setParam(PARAM_KEY_HAS_LIN_Z,    hasLinZ);
  nodeHandle.setParam(PARAM_KEY_HAS_ROT_X,    hasRotX);
  nodeHandle.setParam(PARAM_KEY_HAS_ROT_Y,    hasRotY);
  nodeHandle.setParam(PARAM_KEY_HAS_ROT_Z,    hasRotZ);
  nodeHandle.setParam(PARAM_KEY_MIN_LIN_X,    minLinX);
  nodeHandle.setParam(PARAM_KEY_MIN_LIN_Y,    minLinY);
  nodeHandle.setParam(PARAM_KEY_MIN_LIN_Z,    minLinZ);
  nodeHandle.setParam(PARAM_KEY_MIN_ROT_X,    minRotX);
  nodeHandle.setParam(PARAM_KEY_MIN_ROT_Y,    minRotY);
  nodeHandle.setParam(PARAM_KEY_MIN_ROT_Z,    minRotZ);
  nodeHandle.setParam(PARAM_KEY_MAX_LIN_X,    maxLinX);
  nodeHandle.setParam(PARAM_KEY_MAX_LIN_Y,    maxLinY);
  nodeHandle.setParam(PARAM_KEY_MAX_LIN_Z,    maxLinZ);
  nodeHandle.setParam(PARAM_KEY_MAX_ROT_X,    maxRotX);
  nodeHandle.setParam(PARAM_KEY_MAX_ROT_Y,    maxRotY);
  nodeHandle.setParam(PARAM_KEY_MAX_ROT_Z,    maxRotZ);

  //Create publisher with buffer size set to 1 and latching on.  The publisher
  //should basically just always contain the latest desired velocity.
  ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Twist>(topicTwist, 1, true);

  //Create callback using publisher and parameters
  TeleopSinkTwistCallbackRos callback(&publisher, 0 != exponential,
                                      0 != hasLinX, 0 != hasLinY, 0 != hasLinZ,
                                      0 != hasRotX, 0 != hasRotY, 0 != hasRotZ,
                                      minLinX, minLinY, minLinZ, minRotX, minRotY, minRotZ,
                                      maxLinX, maxLinY, maxLinZ, maxRotX, maxRotY, maxRotZ);

  //Subscribe to teleop topic using callback
  nodeHandle.subscribe(topicTeleop, 1, &TeleopSinkTwistCallbackRos::updated, &callback);

  //Spin until we're done
  ros::spin();

  //Done
  return 0;
}
