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
#include <boost/thread.hpp>
#include <pthread.h>
#include <csignal>
#include <cstdio>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSource::TeleopSource(TeleopSourceCallback* callback) :
  mCallback(callback),
  mIsRunning(false),
  mDestructionInitiated(false),
  mListenTimeout(LISTEN_TIMEOUT_DEFAULT) {

  //Initialise array members
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = AXIS_DEAD_ZONE_DEFAULT;
    mAxisInverted[i] = false;
  }
}
//=============================================================================
TeleopSource::~TeleopSource() {
  //Check if destruction has been initiated from the listening thread.
  //Destruction from this thread is not permitted and can result in undefined
  //behaviour.
  if (isListeningThread()) {
    std::fprintf(stderr, "TeleopSource::~TeleopSource: not allowed from listening thread\n");
    std::fprintf(stderr, "TeleopSource::~TeleopSource: aborting to avoid undefined behaviour\n");
    abort();
  }

  //Check that mDestructionInitiated set, which means sub-class has called the
  //preDestroy() method as it should.  In this case we know that the listening
  //thread is permanently stopped and we can safely just let this object be
  //destroyed.  Failure by a sub-class to call the preDestroy() method in its
  //destructor can lead to undefined behaviour.
  boost::lock_guard<boost::mutex> destructionInitiatedLock(mDestructionInitiatedMutex);
  if (!mDestructionInitiated) {
    std::fprintf(stderr, "TeleopSource::~TeleopSource: sub-class has not called preDestroy\n");
    std::fprintf(stderr, "TeleopSource::~TeleopSource: aborting to avoid undefined behaviour\n");
    abort();
  }
}
//=============================================================================
void TeleopSource::preDestroy() {
  //Lock access to destruction initiated
  boost::lock_guard<boost::mutex> destructionInitiatedLock(mDestructionInitiatedMutex);

  //Initiate destruction
  mDestructionInitiated = true;

  //Check if destruction has been initiated from the listening thread.
  //Destruction from this thread is not permitted and can result in undefined
  //behaviour.
  if (isListeningThread()) {
    std::fprintf(stderr, "TeleopSource::preDestroy: not allowed from listening thread\n");
    std::fprintf(stderr, "TeleopSource::preDestroy: aborting to avoid undefined behaviour\n");
    abort();
  }

  //Stop listening thread and wait for it to stop.  On error, we can't do
  //anything, and this may lead to undefined behaviour, so we abort.  In
  //practice this should never happen.  The stop() method should never fail,
  //and waitForStop() should only fail if called from the listening thread,
  //which is something for which we've already checked.
  if (!stop()) {
    std::fprintf(stderr, "TeleopSource::preDestroy: error in stop\n");
    std::fprintf(stderr, "TeleopSource::preDestroy: aborting to avoid undefined behaviour\n");
    abort();
  } else if (!waitForStopped()) {
    std::fprintf(stderr, "TeleopSource::preDestroy: error in waitForStopped\n");
    std::fprintf(stderr, "TeleopSource::preDestroy: aborting to avoid undefined behaviour\n");
    abort();
  }
}
//=============================================================================
bool TeleopSource::start() {
  //Sanity check callback here (rather than throwing a constructor exception)
  if (NULL == mCallback) {
    std::fprintf(stderr, "TeleopSource::start: NULL callback\n");
    return false;
  }

  //Lock access to destruction initiated
  boost::lock_guard<boost::mutex> destructionInitiatedLock(mDestructionInitiatedMutex);

  //Make sure we never start once destruction is initiated
  if (mDestructionInitiated) {
    std::fprintf(stderr, "TeleopSource::start: cannot start after destruction initiated\n");
    return false;
  }

  //Lock access to running status
  boost::lock_guard<boost::mutex> isRunningLock(mIsRunningMutex);

  //Check if already running
  if (mIsRunning) {
    return true;
  }

  //Note that we are running
  mIsRunning = true;

  //Join previous instance of listening thread
  mListeningThread.join();

  //Create thread which executes listen loop
  mListeningThread = boost::thread(&TeleopSource::listeningThread, this);

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::stop() {
  //Lock access to running status
  boost::lock_guard<boost::mutex> isRunningLock(mIsRunningMutex);

  //Check if running
  if (!mIsRunning) {
    return true;
  }

  //Interrupt the listening thread, can be done multiple times
  mListeningThread.interrupt();

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::waitForStopped() {
  //Check if waiting from listening thread, which is not allowed.
  if (isListeningThread()) {
    std::fprintf(stderr, "TeleopSource::waitForStopped: not allowed from listening thread\n");
    return false;
  }

  //Join listening thread
  mListeningThread.join();

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::isRunning() {
  //Lock access to running status
  boost::lock_guard<boost::mutex> isRunningLock(mIsRunningMutex);
  return mIsRunning;
}
//=============================================================================
bool TeleopSource::isListeningThread() {
  //Check if this is the listening thread
  if (mListeningThread.get_id() == boost::this_thread::get_id()) {
    return true;
  } else {
    return false;
  }
}
//=============================================================================
void TeleopSource::listeningThread() {
  TeleopState teleopState;    //latest teleop state
  ListenResult listenResult;  //listen result
  bool error = false;         //true on error

  //Block all signals from this thread, let the main thread handle them
  sigset_t set;
  sigfillset(&set);
  pthread_sigmask(SIG_BLOCK, &set, NULL);

  //Listen prepare
  if (!listenPrepare()) {
    std::fprintf(stderr, "TeleopSource::listeningThread: error in listenPrepare()\n");
    error = true;
  }

  //Loop until interrupted or error occurs
  while (!error && !boost::this_thread::interruption_requested()) {
    //Listen for events (locking access to listen timeout each time)
    boost::unique_lock<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);
    listenResult = listen(mListenTimeout, &teleopState);
    listenTimeoutLock.unlock();

    //Deal with result
    switch (listenResult) {
      case LISTEN_RESULT_ERROR:
        //Error
        std::fprintf(stderr, "TeleopSource::listeningThread: error in listen()\n");
        error = true;
        break;
      case LISTEN_RESULT_UNCHANGED:
        //Do nothing this time around
        break;
      case LISTEN_RESULT_CHANGED: {
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
        break;
      }
      default:
        //Invalid result
        std::fprintf(stderr, "TeleopSource::listeningThread: invalid result from listen() (%d)\n", listenResult);
        error = true;
        break;
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
  mCallback->updated(&teleopState, true, error);

  //Listen cleanup
  if (!listenCleanup()) {
    std::fprintf(stderr, "TeleopSource::listeningThread: error in listenCleanup()\n");
  }

  //Call stopping callback
  mCallback->stopping(error);

  //Lock access to running status
  boost::lock_guard<boost::mutex> isRunningLock(mIsRunningMutex);

  //Note that we're no longer running
  mIsRunning = false;
}
  //=============================================================================
bool TeleopSource::setListenTimeout(int listenTimeout) {
  if (LISTEN_TIMEOUT_MIN > listenTimeout || LISTEN_TIMEOUT_MAX < listenTimeout) {
    std::fprintf(stderr, "TeleopSource::setListenTimeout: invalid listen timeout (%d)\n", listenTimeout);
    return false;
  }

  //Lock access to listen timeout
  boost::lock_guard<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);
  mListenTimeout = listenTimeout;
  return true;
}
//=============================================================================
int TeleopSource::getListenTimeout() {
  //Lock access to listen timeout
  boost::lock_guard<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);
  return mListenTimeout;
}
//=============================================================================
bool TeleopSource::setAxisDeadZoneForAllAxes(TeleopAxisValue axisDeadZone) {
  if (AXIS_DEAD_ZONE_MIN > axisDeadZone || AXIS_DEAD_ZONE_MAX < axisDeadZone) {
    std::fprintf(stderr, "TeleopSource::setAxisDeadZoneForAllAxes: invalid axis dead zone (%f)\n", axisDeadZone);
    return false;
  }

  //Lock access to axis dead zone
  boost::lock_guard<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = axisDeadZone;
  }
  return true;
}
//=============================================================================
bool TeleopSource::setAxisDeadZone(TeleopAxisValue axisDeadZone, TeleopAxisType axisType) {
  if (AXIS_DEAD_ZONE_MIN > axisDeadZone || AXIS_DEAD_ZONE_MAX < axisDeadZone) {
    std::fprintf(stderr, "TeleopSource::setAxisDeadZone: invalid axis dead zone (%f)\n", axisDeadZone);
    return false;
  }

  //Lock access to axis dead zone
  boost::lock_guard<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);
  mAxisDeadZone[axisType] = axisDeadZone;
  return true;
}
//=============================================================================
TeleopAxisValue TeleopSource::getAxisDeadZone(TeleopAxisType axisType) {
  //Lock access to axis dead zone
  boost::lock_guard<boost::mutex> axisDeadZoneLock(mAxisDeadZoneMutex);
  return mAxisDeadZone[axisType];
}
//=============================================================================
bool TeleopSource::setAxisInvertedForAllAxes(bool axisInverted) {
  //Lock access to axis inverted
  boost::lock_guard<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisInverted[i] = axisInverted;
  }
  return true;
}
//=============================================================================
bool TeleopSource::setAxisInverted(bool axisInverted, TeleopAxisType axisType) {
  //Lock access to axis inverted
  boost::lock_guard<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
  mAxisInverted[axisType] = axisInverted;
  return true;
}
//=============================================================================
bool TeleopSource::getAxisInverted(TeleopAxisType axisType) {
  //Lock access to axis inverted
  boost::lock_guard<boost::mutex> axisInvertedLock(mAxisInvertedMutex);
  return mAxisInverted[axisType];
}
//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
