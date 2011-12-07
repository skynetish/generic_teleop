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
#ifndef INCLUDE_TELEOP_SOURCE_HPP
#define INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/**
 * This class provides a framework for generic handling of tele-operation
 * sources.  The start() and stop() methods start and stop a listening thread
 * which listens for teleop device events and reports them using the updated()
 * callback.  When the listening thread is about to stop, for any reason, the
 * stopping() callback is called.  The listening thread may stop due to a call
 * to stop(), an error, or destruction of the teleop source object.
 *
 * Both the start() and stop() methods are non-blocking.  The start() method
 * starts the listening thread if it is not already running, and does nothing
 * otherwise.  The stop() method interrupts the listening thread if it is
 * running, and does nothing otherwise.  To wait for the listening thread to
 * stop, the waitForStopped() method can be used.  The stopping() callback is
 * called before the waitForStopped() method returns.
 *
 * Destruction of the teleop source object from within the updated() and
 * stopping() callbacks is not permitted, and the result of such destruction
 * is undefined.  The waitForStopped() method will always fail if called from
 * either callback method.
 *
 * Sub-classes for each teleop source type must implement three
 * listening-related virtual methods: one to perform the actual listening
 * (listen()), one for preparation (listenPrepare()) and one for cleanup
 * (listenCleanup()).  They must also call the preDestroy() method from their
 * destructors.  This method ensures that the listening thread is properly and
 * permanently stopped, before returning.  This call needs to be in the
 * sub-class destructors, since the virtual listening-related methods needed to
 * cleanly stop the listening thread are not available by the time the teleop
 * source super-class destructor is initiated.  Failure by a sub-class to call
 * the preDestroy() method in its destructor results in undefined behaviour.
 *
 * The framework ensures that the three listening-related methods are always
 * called in the following order, zero or more times, and for each sequence the
 * methods are called from the same thread.  First, the framework performs one
 * successful call to listenPrepare() -- a failed call to listenPrepare()
 * aborts the sequence.  Second, the framework performs zero or more successful
 * calls to listen() -- a failed call to listen() triggers the next step in the
 * sequence.  Third, the framework performs one call to listenCleanup() --
 * regardless of the result, listenCleanup() should strive to cleanup as much
 * as possible.
 *
 * The class is non-copyable as a precaution, since many teleop sources will
 * use members or resources which are difficult to share.  This could be
 * delegated to the individual sub-classes, but it's safer to do it here,
 * especially since copying a teleop source is probably not very useful in most
 * situations.
 *
 * Messages and errors are simply printed on stdout and stderr, respectively.
 */
class TeleopSource : public boost::noncopyable {

public:

  /**
   * Callback class which reports teleop state updates and listening thread
   * termination.  TeleopSource users should sub-class this to handle events.
   */
  class TeleopSourceCallback {

  public:

    /**
     * Callback used to report an updated teleop state.  This is called from
     * within the listening loop of the teleop source listening thread.  As
     * such, this method should return quickly, to avoid losing teleop events.
     * An alternative could be to create a new thread for each call, but this
     * could cause performance, buffering and synchronisation problems.  The
     * waitForStopped() method will always fail if called from this callback.
     * Teleop source object destruction from within this callback results in
     * undefined behaviour.
     *
     *   @param teleopState [in] - the latest teleop state
     *   @param stopping [in] - true if listening thread is stopping
     *   @param error [in] - true if there were errors while listening
     */
    virtual void updated(const TeleopState* const teleopState, bool stopping, bool error) = 0;

    /**
     * Callback used to report that the listening thread is stopping.  This is
     * called from the teleop source listening thread, after termination of the
     * listening loop.  The listening thread may be stopping due to a call to
     * stop(), an error, or destruction of the teleop source object.  If the
     * waitForStopped() method has been invoked, this callback is called before
     * that method returns.  The waitForStopped() method will always fail if
     * called from this callback.  Teleop source object destruction from within
     * this callback results in undefined behaviour.().
     *
     *   @param error [in] - true if stopping due to an error
     */
    virtual void stopping(bool error) = 0;

  }; //class

  /** Return values for listen method */
  typedef enum {
    LISTEN_RESULT_ERROR,
    LISTEN_RESULT_UNCHANGED,
    LISTEN_RESULT_CHANGED
  } ListenResult;

  /**@{ Listen timeout in milliseconds - check for interruption this often */
  static const int LISTEN_TIMEOUT_DEFAULT;
  static const int LISTEN_TIMEOUT_MIN;
  static const int LISTEN_TIMEOUT_MAX;
  /**@}*/

  /**@{ Axis dead zone - values smaller than this are set to 0.0 */
  static const TeleopAxisValue AXIS_DEAD_ZONE_DEFAULT;
  static const TeleopAxisValue AXIS_DEAD_ZONE_MIN;
  static const TeleopAxisValue AXIS_DEAD_ZONE_MAX;
  /**@}*/

  /**
   * Constructor.
   *
   *   @param callback [in] - callback to use to report status
   */
  explicit TeleopSource(TeleopSourceCallback* callback);

  /**
   * Destructor.  Virtual since sub-classes may be destroyed using pointers to
   * this (parent) class.
   */
  virtual ~TeleopSource();

  /**
   * Start listening thread.  If listening thread is already running this has
   * no effect.  This method is non-blocking.
   *
   *   @return true on success (or already running)
   */
  bool start();

  /**
   * Request that the listening thread be stopped.  If the listening thread is
   * not running this has no effect.  This method is non-blocking.
   *
   *   @return true on success (or already stopped)
   */
  bool stop();

  /**
   * Block until teleop source is stopped.  This method will always fail if
   * called from the listening thread, so it can't be used from the update() or
   * stopping() callbacks.  This method returns after the stopping() callback
   * is called.
   *
   *   @return true on success
   */
  bool waitForStopped();

  /**
   * Check if listening thread is running.
   *
   *   @return true if running.
   */
  bool isRunning();

  /**
   * Set listen timeout, which specifies how often to check for interruption.
   *
   *   @param listenTimeout [in] - listen timeout in milliseconds
   *
   *   @return true on success
   */
  bool setListenTimeout(int listenTimeout);

  /**
   * Get listen timeout, which specifies how often to check for interruption.
   *
   *   @return listen timeout in milliseconds
   */
  int getListenTimeout();

  /**
   * Set axis dead zone for all axes.
   *
   *   @param deadZone [in] - axis dead zone
   *
   *   @return true on success
   */
  bool setAxisDeadZoneForAllAxes(TeleopAxisValue axisDeadZone);

  /**
   * Set axis dead zone for a given axis.
   *
   *   @param axisDeadZone [in] - axis dead zone
   *   @param axisType [in] - axis type
   *
   *   @return true on success
   */
  bool setAxisDeadZone(TeleopAxisValue axisDeadZone, TeleopAxisType axisType);

  /**
   * Get axis dead zone for a given axis.
   *
   *   @param axisType [in] - axis type
   *
   *   @return axis dead zone
   */
  TeleopAxisValue getAxisDeadZone(TeleopAxisType axisType);

  /**
   * Set axis inverted status for all axes.
   *
   *   @param axisInverted [in] - axis inverted status
   *
   *   @return true on success
   */
  bool setAxisInvertedForAllAxes(bool axisInverted);

  /**
   * Set axis inverted status for a given axis.
   *
   *   @param axisInverted [in] - axis inverted status
   *   @param axisType [in] - axis type
   *
   *   @return true on success
   */
  bool setAxisInverted(bool axisInverted, TeleopAxisType axisType);

  /**
   * Get axis inverted status for a given axis.
   *
   *   @param axisType [in] - axis type
   *
   *   @return axis inverted
   */
  bool getAxisInverted(TeleopAxisType axisType);

protected:

  /**
   * Cleanly and permanently stop the listening thread.  This ensures that all
   * subsequent calls to start() will fail.  This must (always and only) be
   * called by sub-class destructors to ensure that the framework does not try
   * to reach virtual methods after the sub-class has been destroyed.
   */
  void preDestroy();

private:

  /** Callback */
  TeleopSourceCallback* mCallback;

  /** Is running flag */
  bool mIsRunning;

  /** Destruction initiated flag */
  bool mDestructionInitiated;

  /** Listen timeout in milliseconds - check for interruption this often */
  int mListenTimeout;

  /** Axis dead zones - values smaller than this are set to 0.0 */
  TeleopAxisValue mAxisDeadZone[TELEOP_AXIS_TYPE_COUNT];

  /** Axis inverted statuses - true means axis values should be inverted */
  bool mAxisInverted[TELEOP_AXIS_TYPE_COUNT];

  /** Listening thread */
  boost::thread mListeningThread;

  /** Mutex for protecting running status */
  boost::mutex mIsRunningMutex;

  /** Mutex for protecting destruction status */
  boost::mutex mDestructionInitiatedMutex;

  /** Mutex for protecting listen timeout */
  boost::mutex mListenTimeoutMutex;

  /** Mutex for protecting axis dead zone */
  boost::mutex mAxisDeadZoneMutex;

  /** Mutex for protecting axis inverted */
  boost::mutex mAxisInvertedMutex;

  /**
   * Check if current thread is the listening thread.
   *
   *   @return true if this is a special thread
   */
  bool isListeningThread();

  /**
   * Executes main listen loop (run in listening thread).
   */
  void listeningThread();

  /**
   * Prepare to listen (open files, etc.).  Called once from listening thread,
   * initially or after a call to listenCleanup().
   *
   *   @return true on success
   */
  virtual bool listenPrepare() = 0;

  /**
   * Blocks until teleop source device events detected or the given timeout is
   * reached.  When events occur, updates the teleop state.  The timeout could
   * also be inherited, but the interface is cleaner if sub-classes don't need
   * to worry about inherited members.  Called one or more times from listening
   * thread, between a successful call to listenPrepare() and any call to
   * listenCleanup().
   *
   *   @param listenTimeout [in] - timeout value in milliseconds
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return LISTEN_ERROR on error, LISTEN_STATE_UNCHANGED on timeout or no
   *           change to state, LISTEN_STATE_CHANGED if state updated
   */
  virtual ListenResult listen(int listenTimeout, TeleopState* const teleop) = 0;

  /**
   * Done listening (close files, etc.).  Called once from listening thread
   * after a successful call to listenPrepare() (and one ore more calls to
   * listen()).  Regardless of the returned result, this method should always
   * try to clean up as much as possible.
   *
   *   @return true on success
   */
  virtual bool listenCleanup() = 0;

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================
