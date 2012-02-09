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
#include <teleop_source_adapter.hpp>
#include <boost/thread.hpp>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Static member definitions
//=============================================================================
const unsigned int    TeleopSourceAdapter::LISTEN_TIMEOUT_DEFAULT = 200;

const TeleopAxisValue TeleopSourceAdapter::AXIS_DEAD_ZONE_DEFAULT = 0.05;
const TeleopAxisValue TeleopSourceAdapter::AXIS_DEAD_ZONE_MIN     = 0.01;
const TeleopAxisValue TeleopSourceAdapter::AXIS_DEAD_ZONE_MAX     = 0.99;




//=============================================================================
//Method definitions
//=============================================================================
TeleopSourceAdapter::TeleopSourceAdapter() :
  mTeleopSource(NULL),
  mCallback(NULL),
  mIsInitialised(false),
  mIsRunning(false),
  mListenTimeout(LISTEN_TIMEOUT_DEFAULT) {
  //Initialise array members
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = AXIS_DEAD_ZONE_DEFAULT;
    mAxisInverted[i] = false;
  }
}
//=============================================================================
TeleopSourceAdapter::~TeleopSourceAdapter() {
  //Sanity check
  if (isListeningThread()) {
    fprintf(stderr, "TeleopSourceAdapter::~TeleopSourceAdapter: error: not allowed from listening thread\n");
    fprintf(stderr, "TeleopSourceAdapter::~TeleopSourceAdapter: error: aborting to avoid undefined behaviour\n");
    abort();
  }

  //Shutdown (should always succeed if not called from listening thread)
  if (!shutdown()) {
    fprintf(stderr, "TeleopSourceAdapter::~TeleopSourceAdapter: warning: ignoring error in shutdown\n");
  }
}
//=============================================================================
bool TeleopSourceAdapter::init(TeleopSource* teleopSource, TeleopSourceAdapterCallback* callback) {
  //Sanity checks
  if (isListeningThread()) {
    fprintf(stderr, "TeleopSourceAdapter::init: error: not allowed from listening thread\n");
    return false;
  }
  if (NULL == teleopSource) {
    fprintf(stderr, "TeleopSourceAdapter::init: error: NULL teleopSource\n");
    return false;
  }
  if (NULL == callback) {
    fprintf(stderr, "TeleopSourceAdapter::init: error: NULL callback\n");
    return false;
  }

  //Lock access to init status
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //If already initialised shutdown first (shutdown should always succeed if
  //not called from listening thread)
  if (mIsInitialised && !shutdown()) {
    fprintf(stderr, "TeleopSourceAdapter::init: warning: ignoring error in shutdown\n");
  }

  //Assign members
  mTeleopSource = teleopSource;
  mCallback = callback;
  mIsInitialised = true;

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::shutdown() {
  //Sanity check
  if (isListeningThread()) {
    fprintf(stderr, "TeleopSourceAdapter::shutdown: error: not allowed from listening thread\n");
    fprintf(stderr, "TeleopSourceAdapter::shutdown: error: aborting to avoid undefined behaviour\n");
    abort();
  }

  //Lock access to init status
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //Check if we're done
  if (!mIsInitialised) {
    return true;
  }

  //Stop (should always succeed if initialised and not called from listening thread)
  if (!stop(true)) {
    fprintf(stderr, "TeleopSourceAdapter::shutdown: warning: ignoring error in stop\n");
  }

  //Assign members
  mTeleopSource = NULL;
  mCallback = NULL;
  mIsInitialised = false;

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::start(bool blocking) {
  //Sanity check
  if (isListeningThread()) {
    fprintf(stderr, "TeleopSourceAdapter::start: error: not allowed from listening thread\n");
    return false;
  }

  //Lock access to init status
  boost::unique_lock<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //Sanity check
  if (!mIsInitialised) {
    fprintf(stderr, "TeleopSourceAdapter::start: error: not initialised\n");
    return false;
  }

  //Lock access to running status
  boost::unique_lock<boost::mutex> isRunningLock(mIsRunningMutex);

  //Check if we're done
  if (!mIsRunning) {
    //Note that we are running
    mIsRunning = true;

    //Join previous instance of listening thread
    mListeningThread.join();

    //Create thread which executes listen loop
    mListeningThread = boost::thread(&TeleopSourceAdapter::listeningThread, this);
  }

  //If blocking, unlock everything and wait for listening thread to end
  if (blocking) {
    isInitialisedLock.unlock();
    isRunningLock.unlock();
    mListeningThread.join();
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::stop(bool blocking) {
  //Sanity check
  if (isListeningThread()) {
    fprintf(stderr, "TeleopSourceAdapter::stop: error: not allowed from listening thread\n");
    return false;
  }

  //Lock access to init status
  boost::unique_lock<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //Sanity check
  if (!mIsInitialised) {
    fprintf(stderr, "TeleopSourceAdapter::stop: error: not initialised\n");
    return false;
  }

  //Lock access to running status
  boost::unique_lock<boost::mutex> isRunningLock(mIsRunningMutex);

  //Check if we're done
  if (mIsRunning) {
    //Interrupt the listening thread, can be done multiple times
    mListeningThread.interrupt();
  }

  //If blocking, unlock everything and wait for listening thread to end
  if (blocking) {
    isInitialisedLock.unlock();
    isRunningLock.unlock();
    mListeningThread.join();
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::waitForStopped() {
  //Sanity check
  if (isListeningThread()) {
    fprintf(stderr, "TeleopSourceAdapter::waitForStopped: error: not allowed from listening thread\n");
    return false;
  }

  //Lock access to init status
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

  //Sanity check
  if (!mIsInitialised) {
    fprintf(stderr, "TeleopSourceAdapter::waitForStopped: error: not initialised\n");
    return false;
  }

  //Join listening thread
  mListeningThread.join();

  //Return result
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::isInitialised() {
  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);
  return mIsInitialised;
}
//=============================================================================
bool TeleopSourceAdapter::isRunning() {
  boost::lock_guard<boost::mutex> isRunningLock(mIsRunningMutex);
  return mIsRunning;
}
//=============================================================================
bool TeleopSourceAdapter::isListeningThread() {
  if (mListeningThread.get_id() == boost::this_thread::get_id()) {
    return true;
  } else {
    return false;
  }
}
//=============================================================================
void TeleopSourceAdapter::listeningThread() {
  TeleopState teleopState;
  bool result;
  bool updated;

  //Block all signals from this thread, let the main thread handle them
  sigset_t set;
  sigfillset(&set);
  pthread_sigmask(SIG_BLOCK, &set, NULL);

  //Init teleop source
  result = mTeleopSource->init();
  if (!result) {
    fprintf(stderr, "TeleopSourceAdapter::listeningThread: error: teleop source init() failed\n");
  }

  //Loop until interrupted or error occurs
  while (result && !boost::this_thread::interruption_requested()) {
    //Listen for events (locking access to listen timeout each time)
    boost::unique_lock<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);
    result = mTeleopSource->listen(mListenTimeout, &teleopState, &updated);
    listenTimeoutLock.unlock();

    //Deal with result
    if (!result) {
      //Error
      fprintf(stderr, "TeleopSourceAdapter::listeningThread: error: teleop source listen() failed\n");
    } else if (updated) {
      //Lock access to axis inversion and dead zone
      boost::unique_lock<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
      boost::unique_lock<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);

      //Enforce axis inversion and dead zone
      for (size_t i = 0; i < teleopState.axes.size(); i++) {
        if (mAxisInverted[teleopState.axes[i].type]) {
          teleopState.axes[i].value *= -1.0;
        }
        if (mAxisDeadZone[teleopState.axes[i].type] > fabs(teleopState.axes[i].value)) {
          teleopState.axes[i].value = 0.0;
        }
      }

      //Unlock access to axis inversion and dead zone
      axisInvertedLock.unlock();
      axisDeadZoneLock.unlock();

      //Call callback
      mCallback->updated(&teleopState, false, false);
    }
  }

  //When done, zero all outputs
  for (size_t i = 0; i < teleopState.axes.size(); i++) {
    teleopState.axes[i].value = 0.0;
  }
  for (size_t i = 0; i < teleopState.buttons.size(); i++) {
    teleopState.buttons[i].value = 0;
  }

  //On termination call updated callback with latest (zeroed) status and
  //the appropriate stopping and error flags.
  mCallback->updated(&teleopState, true, !result);

  //Shutdown
  if (!(mTeleopSource->shutdown())) {
    fprintf(stderr, "TeleopSourceAdapter::listeningThread: warning: ignoring error in teleop source shutdown\n");
  }

  //Call stopping callback
  mCallback->stopping(!result);

  //Lock access to running status
  boost::lock_guard<boost::mutex> isRunningLock(mIsRunningMutex);

  //Note that we're no longer running
  mIsRunning = false;
}
  //=============================================================================
bool TeleopSourceAdapter::setListenTimeout(unsigned int listenTimeout) {
  //Set listen timeout
  boost::lock_guard<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);
  mListenTimeout = listenTimeout;
  return true;
}
//=============================================================================
unsigned int TeleopSourceAdapter::getListenTimeout() {
  //Get listen timeout
  boost::lock_guard<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);
  return mListenTimeout;
}
//=============================================================================
bool TeleopSourceAdapter::setAxisDeadZoneForAllAxes(TeleopAxisValue axisDeadZone) {
  //Sanity check
  if (AXIS_DEAD_ZONE_MIN > axisDeadZone || AXIS_DEAD_ZONE_MAX < axisDeadZone) {
    fprintf(stderr, "TeleopSourceAdapter::setAxisDeadZoneForAllAxes: error: invalid axis dead zone (%f)\n", axisDeadZone);
    return false;
  }

  //Set axis dead zone
  boost::lock_guard<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = axisDeadZone;
  }
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::setAxisDeadZone(TeleopAxisValue axisDeadZone, TeleopAxisType axisType) {
  //Sanity check
  if (AXIS_DEAD_ZONE_MIN > axisDeadZone || AXIS_DEAD_ZONE_MAX < axisDeadZone) {
    fprintf(stderr, "TeleopSourceAdapter::setAxisDeadZone: error: invalid axis dead zone (%f)\n", axisDeadZone);
    return false;
  }

  //Set axis dead zone
  boost::lock_guard<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);
  mAxisDeadZone[axisType] = axisDeadZone;
  return true;
}
//=============================================================================
TeleopAxisValue TeleopSourceAdapter::getAxisDeadZone(TeleopAxisType axisType) {
  //Get axis dead zone
  boost::lock_guard<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);
  return mAxisDeadZone[axisType];
}
//=============================================================================
bool TeleopSourceAdapter::setAxisInvertedForAllAxes(bool axisInverted) {
  //Set axis inverted
  boost::lock_guard<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisInverted[i] = axisInverted;
  }
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::setAxisInverted(bool axisInverted, TeleopAxisType axisType) {
  //Set axis inverted
  boost::lock_guard<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
  mAxisInverted[axisType] = axisInverted;
  return true;
}
//=============================================================================
bool TeleopSourceAdapter::getAxisInverted(TeleopAxisType axisType) {
  //Get axis inverted
  boost::lock_guard<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
  return mAxisInverted[axisType];
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
