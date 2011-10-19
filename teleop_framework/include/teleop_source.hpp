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
 * sources.  The start() and stop() methods start and stop a listening loop
 * which runs in a separate thread.  This loop listens for teleop device events
 * and reports them using the update() callback.  Listen thread termination is
 * signalled using the stopped() callback.
 *
 * Sub-classes for each teleop source type must implement the given pure
 * virtual methods to perform the actual listening, as well as related
 * preparation and cleanup.
 *
 * The class is non-copyable as a precaution, since many teleop sources will
 * use members or resources which are difficult to share.  This could be
 * delegated to the individual sub-classes, but it's safer to do it here,
 * especially since copying a teleop source is probably not very useful in most
 * situations.
 *
 * Note that messages and errors are simply printed on stdout and stderr,
 * respectively, which means that output may be garbled if other threads within
 * the same process are printing at the same time.
 */
class TeleopSource : boost::noncopyable {

public:

  /**
   * Callback class which reports teleop state updates and listening thread
   * termination.  TeleopSource users should use a sub-class to handle events.
   */
  class TeleopSourceCallback {

  public:

    /**
     * Callback used to report an updated teleop state.  This is called from
     * the teleop source listening thread.  As such, this method should return
     * fairly quickly, to avoid losing teleop events.  An alternative could be
     * to create a new thread for each call, but this could cause performance,
     * buffering and synchronisation problems.  If "stopping" is true, the
     * listening thread is stopping its execution.  If "error" is true, there
     * has been an error.  For fatal errors, both "stopping" and "error" will
     * be true.  Note that even if stopping is true, the user should still call
     * the stop() method to ensure that cleanup is performed.  Also, note that
     * stop() cannot be called from the listening thread, which means that it
     * cannot be called from within this callback method.  It can, however, be
     * called from the stopped() method.
     *
     *   @param teleopState [in] - the latest teleop state
     *   @param stopping [in] - true if listening thread is stopping
     *   @param error [in] - true if there were errors
     */
    virtual void updated(const TeleopState* const teleopState, bool stopping, bool error) = 0;

    /**
     * Callback used to report that the teleop source has stopped (at some
     * point in the past).  This is called from its own (detached) thread.  If
     * "error" is true, the stoppage was due to an error.  Note that because
     * this runs in its own thread, there is no guarantee that another thread
     * hasn't started the teleop source again since it was stopped.  Also, this
     * method can be used to start(), stop(), or even destroy the teleop
     * source, since it runs in it's own detached thread.  In particular, this
     * callback is a good place to call the stop() method, to ensure that
     * cleanup is performed.
     *
     *   @param error [in] - true if there were errors
     */
    virtual void stopped(bool error) = 0;

  }; //class

  /** Return values for listen method */
  typedef enum {
    LISTEN_RESULT_ERROR,
    LISTEN_RESULT_UNCHANGED,
    LISTEN_RESULT_CHANGED
  } ListenResult;

  /**@{ Listen timeout in milliseconds - check for interruption this often */
  static const int LISTEN_TIMEOUT_DEFAULT = 200;
  static const int LISTEN_TIMEOUT_MIN = 0;
  static const int LISTEN_TIMEOUT_MAX = 60000;
  /**@}*/

  /**@{ Axis dead zone - values smaller than this are set to 0.0 */
  static const TeleopAxisValue AXIS_DEAD_ZONE_DEFAULT = 0.01;
  static const TeleopAxisValue AXIS_DEAD_ZONE_MIN = 0.01;
  static const TeleopAxisValue AXIS_DEAD_ZONE_MAX = 0.99;
  /**@}*/

  /**
   * Constructor.
   *
   *   @param callback [in] - callback to use to report status
   */
  TeleopSource(TeleopSourceCallback* callback);

  /**
   * Destructor.  Ideally we'd like to call stop() from the destructor.  But
   * stop() calls doneListening(), which is a virtual method (and it should be,
   * since the base class can't clean up for the sub-class).  Since C++ doesn't
   * allow virtual methods to be called from the destructor, most sub-classes
   * should (at least) call stop() from their destructors.
   */
  virtual ~TeleopSource();

  /**
   * Start listening thread which reports teleop device activity.
   *
   *   @return true on success
   */
  bool start();

  /**
   * Stop listening thread.  Cannot be called from the listening thread itself,
   * which means it cannot be called from the callback.
   *
   *   @return true on success
   */
  bool stop();

  /**
   * Check if listening thread has been started (and not stopped).  This will
   * be false before the thread has been started, and after the stop() method
   * has been called.
   *
   *   @return true if running.
   */
  bool isStarted();

  /**
   * Check if listening thread is actually executing something.  This will be
   * false before the thread has been started, and after the thread has stopped
   * executing (due to a call to stop() or a fatal error).
   *
   *   @return true if executing.
   */
  bool isExecuting();

  /**
   * Set listen timeout which specifies how often to check for interruption.
   *
   *   @param listenTimeout [in] - listen timeout in milliseconds
   *
   *   @return true on success
   */
  bool setListenTimeout(int listenTimeout);

  /**
   * Get listen timeout.
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

private:

  /** Callback */
  TeleopSourceCallback* mCallback;

  /** Listen timeout in milliseconds - check for interruption this often */
  int mListenTimeout;

  /** Axis dead zones - values smaller than this are set to 0.0 */
  TeleopAxisValue mAxisDeadZone[TELEOP_AXIS_TYPE_COUNT];

  /** Axis inverted statuses - true means axis values should be inverted */
  bool mAxisInverted[TELEOP_AXIS_TYPE_COUNT];

  /** Listening thread */
  boost::thread mThread;

  /** Listening thread is executing */
  bool mThreadExecuting;

  /** Mutex for protecting listening thread state */
  boost::recursive_mutex mThreadMutex;

  /** Mutex for protecting thread executing flag */
  boost::mutex mThreadExecutingMutex;

  /** Mutex for protecting listen timeout */
  boost::mutex mListenTimeoutMutex;

  /** Mutex for protecting axis dead zone */
  boost::mutex mAxisDeadZoneMutex;

  /** Mutex for protecting axis inverted */
  boost::mutex mAxisInvertedMutex;

  /** Mutex for ensuring that stopped is not called until execution stops */
  boost::mutex mThreadStoppedMutex;

  /**
   * Executes main listen loop (run in listening thread).
   */
  void listeningThread();

  /**
   * Calls stopped callback (run in own detached thread).
   */
  void stoppingThread(bool error);

  /**
   * Prepare to listen (open files, etc.).  The framework guarantees that this
   * will not be called multiple times unless stop() has called doneListening()
   * in between.
   *
   *   @return true on success
   */
  virtual bool prepareToListen() = 0;

  /**
   * Blocks until teleop source device events detected or the given timeout is
   * reached.  When events occur, updates the teleop state.  The timeout could
   * also be inherited, but the interface is cleaner if sub-classes don't need
   * to worry about inherited members.  Since this method is called from the
   * listening thread, modifiable class members should be used carefully, and
   * protected as needed.
   *
   *   @param timeoutMilliseconds [in] - timeout value in milliseconds
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return LISTEN_ERROR on error, LISTEN_STATE_UNCHANGED on timeout or no
   *           change to state, LISTEN_STATE_CHANGED if state updated
   */
  virtual ListenResult listen(int timeoutMilliseconds, TeleopState* const teleop) = 0;

  /**
   * Done listening (close files, etc.).  The framework guarantees that this
   * will not be called until start() has called prepareToListen(), and it will
   * not be called multiple times unless start() has called prepareToListen() in
   * between.
   *
   *   @return true on success
   */
  virtual bool doneListening() = 0;

}; //class





//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================
