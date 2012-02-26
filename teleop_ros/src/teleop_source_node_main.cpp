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
#include <teleop_source_node.hpp>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <signal.h>
#include <string.h>
#include <stdio.h>




//=============================================================================
//Function prototypes
//=============================================================================
/**
 * Signal handler.
 *
 *   @param signalNumber [in] - received signal number
 */
static void signalHandler(int signalNumber);

/**
 * Check if we should just print usage information, and if so, print it.
 *
 *   @param nodeName [in] - node name
 *   @param argc [in] - number of command line arguments
 *   @param argv [in] - command line arguments
 *
 *   @return true if usage was printed
 */
static bool printUsage(std::string nodeName, int argc, char** argv);




//=============================================================================
//Globals
//=============================================================================
/** Global atomic flag used to indicate if an interrupt has been requested. */
static sig_atomic_t gInterruptRequested = 0;




//=============================================================================
//Function definitions
//=============================================================================
static void signalHandler(int signalNumber) {
  gInterruptRequested = 1;
}
//=============================================================================
static bool printUsage(std::string nodeName, int argc, char** argv) {
  //Check for "-h" or "--help", if found, print usage
  for (int i = 1; i < argc; i++) {
    if ((0 == strcmp(argv[i], "-h")) || (0 == strcmp(argv[i], "--help"))) {
      printf("\n");
      printf("Usage:\n");
      printf("    %s [params]\n", nodeName.c_str());
      printf("\n");
      printf("Parameters and their default values\n");
      printf("    _%s:=%s\n",
             teleop::TeleopSourceNode::PARAM_KEY_TELEOP_TOPIC,
             (nodeName + "/" + teleop::TeleopSourceNode::PARAM_DEFAULT_TELEOP_TOPIC).c_str());
      printf("    _%s:=%s\n",
             teleop::TeleopSourceNode::PARAM_KEY_TELEOP_TYPE,
             teleop::TeleopSourceNode::PARAM_DEFAULT_TELEOP_TYPE);
      printf("    _%s:=%d\n",
             teleop::TeleopSourceNode::PARAM_KEY_LISTEN_TIMEOUT,
             teleop::TeleopSourceNode::PARAM_DEFAULT_LISTEN_TIMEOUT);
      printf("    _%s:=%lf\n",
             teleop::TeleopSourceNode::PARAM_KEY_AXIS_DEAD_ZONE,
             teleop::TeleopSourceNode::PARAM_DEFAULT_AXIS_DEAD_ZONE);
      printf("    _%s:=%d\n",
             teleop::TeleopSourceNode::PARAM_KEY_KEYBOARD_STEPS,
             teleop::TeleopSourceNode::PARAM_DEFAULT_KEYBOARD_STEPS);
      printf("    _%s:=%s\n",
             teleop::TeleopSourceNode::PARAM_KEY_JOYSTICK_DEVICE,
             teleop::TeleopSourceNode::PARAM_DEFAULT_JOYSTICK_DEVICE);
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
  //Use first argument (executable name) as node name
  std::string nodeName(basename(argv[0]));

  //Check if we should just print usage information and quit
  if (printUsage(nodeName, argc, argv)) {
    return 0;
  }

  //Set SIGINT signal handler
  signal(SIGINT, signalHandler);

  //Create the node object
  teleop::TeleopSourceNode node;

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
