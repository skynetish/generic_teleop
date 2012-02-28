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
#ifndef INCLUDE_TELEOP_SOURCE_ADAPTER_HPP
#define INCLUDE_TELEOP_SOURCE_ADAPTER_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>
#include <teleop_source.hpp>
#include <boost/thread.hpp>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================
/**
 * This class defines an adapter object used to provide high-level thread-based
 * control of a given teleop source object.
 *
 * The init() and shutdown() methods control the life cycle of the object.
 * Note that shutdown() is always called on destruction.
 *
 * The start() and stop() methods start and stop a listening thread which
 * listens for teleop device events and reports them using the updated()
 * callback.  When the listening thread is about to stop, the stopping()
 * callback is called.  The listening thread may stop due to a call to stop(),
 * an error, or destruction of the adapter object.
 *
 * To wait for the listening thread to stop, the waitForStopped() method can
 * be called; alternatively, the start() and stop() methods can be called in
 * blocking mode.  The stopping() callback is called before any of the blocking
 * methods return.  The blocking methods may return in any order.
 *
 * Both callbacks are called from the listening thread.  As such, calls to
 * init(), start(), stop(), and waitForStopped() will always fail when called
 * from either callback.  Calls to shutdown() or destruction of the adapter
 * object from either callback are not permitted, and will cause the program to
 * be aborted (to avoid undefined behaviour).
 */
class TeleopSourceAdapter {

public:

  /**
   * Callback class which reports teleop state updates and listening thread
   * termination.  Users should sub-class this to handle events.
   */
  class TeleopSourceAdapterCallback {

  public:

    /**
     * Destructor.  Virtual so destruction will invoke sub-class destructor.
     */
    virtual ~TeleopSourceAdapterCallback() {}

    /**
     * Callback used to report an updated teleop state.  This is called from
     * the listening thread.  As such, this method should return quickly, to
     * avoid delaying or even losing teleop events.  An alternative could be to
     * create a new thread for each call, but this could cause performance,
     * buffering and synchronisation problems.
     *
     *   @param teleopState [in] - the latest teleop state
     *   @param stopping [in] - true if listening thread is stopping
     *   @param error [in] - true if there were errors while listening
     */
    virtual void updated(const TeleopState* const teleopState, bool stopping, bool error) = 0;

    /**
     * Callback used to report that the listening thread is stopping.  This is
     * called from the listening thread.  The listening thread may be stopping
     * due to a call to stop(), an error, or destruction of the adapter object.
     *
     *   @param error [in] - true if stopping due to an error
     */
    virtual void stopping(bool error) = 0;

  }; //class

  /** Listen timeout in milliseconds - check for interruption this often */
  static const unsigned int LISTEN_TIMEOUT_DEFAULT;

  /** Axis dead zone default - values smaller than dead zone are set to 0.0 */
  static const TeleopAxisValue AXIS_DEAD_ZONE_DEFAULT;
  /** Axis dead zone min */
  static const TeleopAxisValue AXIS_DEAD_ZONE_MIN;
  /** Axis dead zone max */
  static const TeleopAxisValue AXIS_DEAD_ZONE_MAX;

  /**
   * Constructor.
   */
  TeleopSourceAdapter();

  /**
   * Destructor.
   */
  ~TeleopSourceAdapter();

  /**
   * Initialise object.  If object is already initialised it is shutdown and
   * reinitialised.  After a successful call to this method the teleop source
   * and callback objects should not be destroyed or shutdown until the
   * shutdown() method has been called.
   *
   *   @param teleopSource - teleop source object to use
   *   @param callback - callback object to use
   *
   *   @return true on success
   */
  bool init(TeleopSource* teleopSource, TeleopSourceAdapterCallback* callback);

  /**
   * Shutdown object.  If object is already shutdown this has no effect.  This
   * method always cleans up as much as possible, even if there are errors.
   * This method is always called on destruction.
   *
   *   @return true on success
   */
  bool shutdown();

  /**
   * Start listening thread.  If listening thread is already running this has
   * no effect, although it will block if the blocking flag is set.  Should
   * only be called between init() and shutdown().
   *
   *   @param blocking - if true block until stopped
   *
   *   @return true on success (or already running)
   */
  bool start(bool blocking);

  /**
   * Request that the listening thread be stopped.  If the listening thread is
   * not running this has no effect.  Should only be called between init() and
   * shutdown().
   *
   *   @param blocking - if true block until stopped
   *
   *   @return true on success (or already stopped)
   */
  bool stop(bool blocking);

  /**
   * Block until teleop source is stopped.  Should only be called between
   * init() and shutdown().
   *
   *   @return true on success
   */
  bool waitForStopped();

  /**
   * Check if node is initialised.
   *
   *   @return true if initialised
   */
  bool isInitialised();

  /**
   * Check if node is running.
   *
   *   @return true if running
   */
  bool isRunning();

  /**
   * Check if current thread is the listening thread.
   *
   *   @return true if this is the listening thread
   */
  bool isListeningThread();

  /**
   * Set listen timeout, which specifies how often to check for interruption.
   *
   *   @param listenTimeout [in] - listen timeout in milliseconds
   *
   *   @return true on success
   */
  bool setListenTimeout(unsigned int listenTimeout);

  /**
   * Get listen timeout, which specifies how often to check for interruption.
   *
   *   @return listen timeout in milliseconds
   */
  unsigned int getListenTimeout();

  /**
   * Set axis dead zone for all axes.
   *
   *   @param axisDeadZone [in] - axis dead zone
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

  /** Teleop source */
  TeleopSource* mTeleopSource;

  /** Callback */
  TeleopSourceAdapterCallback* mCallback;

  /** Init done flag */
  bool mIsInitialised;

  /** Is running flag */
  bool mIsRunning;

  /** Listen timeout in milliseconds - check for interruption this often */
  unsigned int mListenTimeout;

  /** Axis dead zones - values smaller than this are set to 0.0 */
  TeleopAxisValue mAxisDeadZone[TELEOP_AXIS_TYPE_COUNT];

  /** Axis inverted statuses - true means axis values should be inverted */
  bool mAxisInverted[TELEOP_AXIS_TYPE_COUNT];

  /** Listening thread */
  boost::thread mListeningThread;

  /** Mutex to protect is initialised flag */
  boost::recursive_mutex mIsInitialisedMutex;

  /** Mutex to protect is running flag */
  boost::mutex mIsRunningMutex;

  /** Mutex to protect listen timeout */
  boost::mutex mListenTimeoutMutex;

  /** Mutex to protect axis dead zone */
  boost::mutex mAxisDeadZoneMutex;

  /** Mutex to protect axis inverted */
  boost::mutex mAxisInvertedMutex;

  /**
   * Executes main listen loop (run in listening thread).
   */
  void listeningThread();

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_ADAPTER_HPP
//=============================================================================
