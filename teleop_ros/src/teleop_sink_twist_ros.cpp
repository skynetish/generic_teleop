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
#include <teleop_msgs/State.h>
#include <ros/ros.h>
#include <cmath>
#include <cstdio>




//=============================================================================
//Defines
//=============================================================================

/**@{ Parameter keys */
#define PARAM_KEY_TOPIC_TELEOP          "topic-teleop"
#define PARAM_KEY_TOPIC_TWIST           "topic-twist"

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
/**@}*/

/**@{ Parameter default values */
#define PARAM_DEFAULT_TOPIC_TELEOP      "teleop"
#define PARAM_DEFAULT_TOPIC_TWIST       "cmd_vel"

#define PARAM_DEFAULT_HAS_LIN_X         1
#define PARAM_DEFAULT_HAS_LIN_Y         0
#define PARAM_DEFAULT_HAS_LIN_Z         0
#define PARAM_DEFAULT_HAS_ROT_X         0
#define PARAM_DEFAULT_HAS_ROT_Y         0
#define PARAM_DEFAULT_HAS_ROT_Z         1

#define PARAM_DEFAULT_MAX_LIN_X         (1.0)
#define PARAM_DEFAULT_MAX_LIN_Y         (1.0)
#define PARAM_DEFAULT_MAX_LIN_Z         (1.0)
#define PARAM_DEFAULT_MAX_ROT_X         (M_PI)
#define PARAM_DEFAULT_MAX_ROT_Y         (M_PI)
#define PARAM_DEFAULT_MAX_ROT_Z         (M_PI)

#define PARAM_DEFAULT_MIN_LIN_X         (-1.0)
#define PARAM_DEFAULT_MIN_LIN_Y         (-1.0)
#define PARAM_DEFAULT_MIN_LIN_Z         (-1.0)
#define PARAM_DEFAULT_MIN_ROT_X         (-M_PI)
#define PARAM_DEFAULT_MIN_ROT_Y         (-M_PI)
#define PARAM_DEFAULT_MIN_ROT_Z         (-M_PI)
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
   */
  TeleopSinkTwistCallbackRos(const ros::Publisher* const publisher,
                             int hasLinX, int hasLinY, int hasLinZ,
                             int hasRotX, int hasRotY, int hasRotZ,
                             double maxLinX, double maxLinY, double maxLinZ,
                             double maxRotX, double maxRotY, double maxRotZ,
                             double minLinX, double minLinY, double minLinZ,
                             double minRotX, double minRotY, double minRotZ);

  /**
   * Called when teleop topic is updated.  This converts the latest teleop
   * state message to a twist message and publishes it.
   */
  void updated(const teleop_msgs::State& teleopStateMsg);

private:

  /** Publisher given to constructor and used in updated() */
  const ros::Publisher* const mPublisher;

  /**@{ Parameters */
  int mHasLinX, mHasLinY, mHasLinZ, mHasRotX, mHasRotY, mHasRotZ;
  double mMaxLinX, mMaxLinY, mMaxLinZ, mMaxRotX, mMaxRotY, mMaxRotZ;
  double mMinLinX, mMinLinY, mMinLinZ, mMinRotX, mMinRotY, mMinRotZ;
  /**@}*/

  /** Twist member avoids re-creation for each call to updated() */
  geometry_msgs::Twist mTwistMsg;

}; //class




//=============================================================================
//Function prototypes
//=============================================================================

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
TeleopSinkTwistCallbackRos::TeleopSinkTwistCallbackRos(const ros::Publisher* const publisher,
                                                       int hasLinX, int hasLinY, int hasLinZ,
                                                       int hasRotX, int hasRotY, int hasRotZ,
                                                       double maxLinX, double maxLinY, double maxLinZ,
                                                       double maxRotX, double maxRotY, double maxRotZ,
                                                       double minLinX, double minLinY, double minLinZ,
                                                       double minRotX, double minRotY, double minRotZ) :
  mPublisher(publisher),
  mHasLinX(hasLinX),
  mHasLinY(hasLinY),
  mHasLinZ(hasLinZ),
  mHasRotX(hasRotX),
  mHasRotY(hasRotY),
  mHasRotZ(hasRotZ),
  mMaxLinX(maxLinX),
  mMaxLinY(maxLinY),
  mMaxLinZ(maxLinZ),
  mMaxRotX(maxRotX),
  mMaxRotY(maxRotY),
  mMaxRotZ(maxRotZ),
  mMinLinX(minLinX),
  mMinLinY(minLinY),
  mMinLinZ(minLinZ),
  mMinRotX(minRotX),
  mMinRotY(minRotY),
  mMinRotZ(minRotZ) {
}
//=============================================================================
void TeleopSinkTwistCallbackRos::updated(const teleop_msgs::State& teleopStateMsg) {
  //TODO
}




//=============================================================================
//Function definitions
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
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_X,    PARAM_DEFAULT_HAS_LIN_X);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_Y,    PARAM_DEFAULT_HAS_LIN_Y);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_LIN_Z,    PARAM_DEFAULT_HAS_LIN_Z);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_X,    PARAM_DEFAULT_HAS_ROT_X);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_Y,    PARAM_DEFAULT_HAS_ROT_Y);
      std::printf("    _%s:=%d\n",    PARAM_KEY_HAS_ROT_Z,    PARAM_DEFAULT_HAS_ROT_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_X,    PARAM_DEFAULT_MAX_LIN_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_Y,    PARAM_DEFAULT_MAX_LIN_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_LIN_Z,    PARAM_DEFAULT_MAX_LIN_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_X,    PARAM_DEFAULT_MAX_ROT_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_Y,    PARAM_DEFAULT_MAX_ROT_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MAX_ROT_Z,    PARAM_DEFAULT_MAX_ROT_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_X,    PARAM_DEFAULT_MIN_LIN_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_Y,    PARAM_DEFAULT_MIN_LIN_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_LIN_Z,    PARAM_DEFAULT_MIN_LIN_Z);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_X,    PARAM_DEFAULT_MIN_ROT_X);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_Y,    PARAM_DEFAULT_MIN_ROT_Y);
      std::printf("    _%s:=%lf\n",   PARAM_KEY_MIN_ROT_Z,    PARAM_DEFAULT_MIN_ROT_Z);
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
  ros::init(argc, argv, basename(argv[0]));

  //Node handle uses private namespace (exceptions ignored intentionally)
  ros::NodeHandle nodeHandle("~");

  //Declare parameters
  std::string topicTeleop;
  std::string topicTwist;
  int hasLinX, hasLinY, hasLinZ, hasRotX, hasRotY, hasRotZ;
  double maxLinX, maxLinY, maxLinZ, maxRotX, maxRotY, maxRotZ;
  double minLinX, minLinY, minLinZ, minRotX, minRotY, minRotZ;

  //Read parameters and set default values
  nodeHandle.param(PARAM_KEY_TOPIC_TELEOP, topicTeleop, std::string(PARAM_DEFAULT_TOPIC_TELEOP));
  nodeHandle.param(PARAM_KEY_TOPIC_TWIST,  topicTwist,  std::string(PARAM_DEFAULT_TOPIC_TWIST));
  nodeHandle.param(PARAM_KEY_HAS_LIN_X,    hasLinX,     PARAM_DEFAULT_HAS_LIN_X);
  nodeHandle.param(PARAM_KEY_HAS_LIN_Y,    hasLinY,     PARAM_DEFAULT_HAS_LIN_Y);
  nodeHandle.param(PARAM_KEY_HAS_LIN_Z,    hasLinZ,     PARAM_DEFAULT_HAS_LIN_Z);
  nodeHandle.param(PARAM_KEY_HAS_ROT_X,    hasRotX,     PARAM_DEFAULT_HAS_ROT_X);
  nodeHandle.param(PARAM_KEY_HAS_ROT_Y,    hasRotY,     PARAM_DEFAULT_HAS_ROT_Y);
  nodeHandle.param(PARAM_KEY_HAS_ROT_Z,    hasRotZ,     PARAM_DEFAULT_HAS_ROT_Z);
  nodeHandle.param(PARAM_KEY_MAX_LIN_X,    maxLinX,     PARAM_DEFAULT_MAX_LIN_X);
  nodeHandle.param(PARAM_KEY_MAX_LIN_Y,    maxLinY,     PARAM_DEFAULT_MAX_LIN_Y);
  nodeHandle.param(PARAM_KEY_MAX_LIN_Z,    maxLinZ,     PARAM_DEFAULT_MAX_LIN_Z);
  nodeHandle.param(PARAM_KEY_MAX_ROT_X,    maxRotX,     PARAM_DEFAULT_MAX_ROT_X);
  nodeHandle.param(PARAM_KEY_MAX_ROT_Y,    maxRotY,     PARAM_DEFAULT_MAX_ROT_Y);
  nodeHandle.param(PARAM_KEY_MAX_ROT_Z,    maxRotZ,     PARAM_DEFAULT_MAX_ROT_Z);
  nodeHandle.param(PARAM_KEY_MIN_LIN_X,    minLinX,     PARAM_DEFAULT_MIN_LIN_X);
  nodeHandle.param(PARAM_KEY_MIN_LIN_Y,    minLinY,     PARAM_DEFAULT_MIN_LIN_Y);
  nodeHandle.param(PARAM_KEY_MIN_LIN_Z,    minLinZ,     PARAM_DEFAULT_MIN_LIN_Z);
  nodeHandle.param(PARAM_KEY_MIN_ROT_X,    minRotX,     PARAM_DEFAULT_MIN_ROT_X);
  nodeHandle.param(PARAM_KEY_MIN_ROT_Y,    minRotY,     PARAM_DEFAULT_MIN_ROT_Y);
  nodeHandle.param(PARAM_KEY_MIN_ROT_Z,    minRotZ,     PARAM_DEFAULT_MIN_ROT_Z);

  //Advertise parameters for introspection
  nodeHandle.setParam(PARAM_KEY_TOPIC_TELEOP, topicTeleop);
  nodeHandle.setParam(PARAM_KEY_TOPIC_TWIST,  topicTwist);
  nodeHandle.setParam(PARAM_KEY_HAS_LIN_X,    hasLinX);
  nodeHandle.setParam(PARAM_KEY_HAS_LIN_Y,    hasLinY);
  nodeHandle.setParam(PARAM_KEY_HAS_LIN_Z,    hasLinZ);
  nodeHandle.setParam(PARAM_KEY_HAS_ROT_X,    hasRotX);
  nodeHandle.setParam(PARAM_KEY_HAS_ROT_Y,    hasRotY);
  nodeHandle.setParam(PARAM_KEY_HAS_ROT_Z,    hasRotZ);
  nodeHandle.setParam(PARAM_KEY_MAX_LIN_X,    maxLinX);
  nodeHandle.setParam(PARAM_KEY_MAX_LIN_Y,    maxLinY);
  nodeHandle.setParam(PARAM_KEY_MAX_LIN_Z,    maxLinZ);
  nodeHandle.setParam(PARAM_KEY_MAX_ROT_X,    maxRotX);
  nodeHandle.setParam(PARAM_KEY_MAX_ROT_Y,    maxRotY);
  nodeHandle.setParam(PARAM_KEY_MAX_ROT_Z,    maxRotZ);
  nodeHandle.setParam(PARAM_KEY_MIN_LIN_X,    minLinX);
  nodeHandle.setParam(PARAM_KEY_MIN_LIN_Y,    minLinY);
  nodeHandle.setParam(PARAM_KEY_MIN_LIN_Z,    minLinZ);
  nodeHandle.setParam(PARAM_KEY_MIN_ROT_X,    minRotX);
  nodeHandle.setParam(PARAM_KEY_MIN_ROT_Y,    minRotY);
  nodeHandle.setParam(PARAM_KEY_MIN_ROT_Z,    minRotZ);

  //Create publisher with buffer size set to 1 and latching on.  The publisher
  //should basically just always contain the latest desired velocity.
  ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Twist>(topicTwist, 1, true);

  //Create callback using publisher and parameters
  TeleopSinkTwistCallbackRos = callback(&publisher,
                                        hasLinX, hasLinY, hasLinZ, hasRotX, hasRotY, hasRotZ,
                                        maxLinX, maxLinY, maxLinZ, maxRotX, maxRotY, maxRotZ,
                                        minLinX, minLinY, minLinZ, minRotX, minRotY, minRotZ);

  //Subscribe to teleop topic using callback
  nodeHandle.subscribe(topicTeleop, 1, &TeleopSinkTwistCallbackRos::updated, &callback);

  //Spin until shutdown is called
  ros::spin();

  //Done
  return 0;
}
