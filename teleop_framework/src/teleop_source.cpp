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
#include <cstdio>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Method definitions
//=============================================================================
TeleopSource::TeleopSource(TeleopSourceCallback* callback)
  : mCallback(callback), mListenTimeout(LISTEN_TIMEOUT_DEFAULT) {
  //Initialise array members
  for (int i = 0; i < TELEOP_AXIS_TYPE_COUNT; i++) {
    mAxisDeadZone[i] = AXIS_DEAD_ZONE_DEFAULT;
    mAxisInverted[i] = false;
  }
}
//=============================================================================
TeleopSource::~TeleopSource() {
  //Stop listening thread
  stop();
}
//=============================================================================
bool TeleopSource::start() {
  //Sanity check callback here (rather than throwing a constructor exception)
  if (NULL == mCallback) {
    fprintf(stderr, "TeleopSource::start: invalid callback\n");
    return false;
  }

  //Lock access to thread state
  boost::lock_guard<boost::recursive_mutex> threadLock(mThreadMutex);

  //Check if running
  if (!isRunning()) {
    //Prepare to listen
    if (!prepareToListen()) {
      fprintf(stderr, "TeleopSource::start: error in prepareToListen()\n");
      return false;
    }

    //Create thread which executes listen loop
    mThread = boost::thread(&TeleopSource::listenLoop, this);
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::stop() {
  //Lock access to thread state
  boost::lock_guard<boost::recursive_mutex> threadLock(mThreadMutex);

  //Check if running
  if (!isRunning()) {
    return true;
  }

  //Interrupt
  mThread.interrupt();

  //Wait for thread to finish
  mThread.join();

  //Done listening
  if (!doneListening()) {
    fprintf(stderr, "TeleopSource::stop: error in doneListening()\n");
    return false;
  }

  //Return result
  return true;
}
//=============================================================================
bool TeleopSource::isRunning() {
  //Lock access to thread state
  boost::lock_guard<boost::recursive_mutex> threadLock(mThreadMutex);

  //Check if thread has same ID as default thread (which is "Not-A-Thread")
  return (boost::thread::id() != mThread.get_id());
}
//=============================================================================
void TeleopSource::listenLoop() {
  TeleopState teleopState;    //latest teleop state
  ListenResult listenResult;  //listen result
  bool error = false;         //true on error

  //Loop until interrupted or error occurs
  while (!error && !boost::this_thread::interruption_requested()) {

    //Lock access to listen timeout
    boost::unique_lock<boost::mutex> listenTimeoutLock(mListenTimeoutMutex);

    //Listen for events
    listenResult = listen(mListenTimeout, &teleopState);

    //Unlock access to listen timeout
    listenTimeoutLock.unlock();

    //Deal with result
    switch (listenResult) {
      case LISTEN_RESULT_ERROR:
        //Error
        fprintf(stderr, "TeleopSource::listenLoop: error in listen()\n");
        error = true;
        break;
      case LISTEN_RESULT_UNCHANGED:
        //Do nothing this time around
        break;
      case LISTEN_RESULT_CHANGED:
      {
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
        mCallback->callback(&teleopState, false, false);
        break;
      }
      default:
        //Invalid result
        fprintf(stderr, "TeleopSource::listenLoop: invalid result from listen() (%d)\n",
                listenResult);
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

  //On termination call the callback one more time with the latest status
  mCallback->callback(&teleopState, true, error);
}
//=============================================================================
bool TeleopSource::setListenTimeout(int listenTimeout) {
  if (LISTEN_TIMEOUT_MIN > listenTimeout || LISTEN_TIMEOUT_MAX < listenTimeout) {
    fprintf(stderr, "TeleopSource::setListenTimeout: invalid listen timeout (%d)\n",
            listenTimeout);
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
    fprintf(stderr, "TeleopSource::setAxisDeadZoneForAllAxes: invalid axis dead zone (%f)\n",
            axisDeadZone);
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
    fprintf(stderr, "TeleopSource::setAxisDeadZone: invalid axis dead zone (%f)\n", axisDeadZone);
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
bool TeleopSource::setAxisInverted(bool axisInverted,
                                   TeleopAxisType axisType) {
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
