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




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================
/**
 * This class defines a simple low-level interface to teleop sources.
 *
 * The init() and shutdown() methods control the life cycle of the object.
 * Note that shutdown() is always called on destruction.
 *
 * The listen() method listens for teleop source events and updates the
 * provided teleop state accordingly.
 */
class TeleopSource {

public:

  /**
   * Destructor.  Virtual so destruction will invoke sub-class destructor.
   */
  virtual ~TeleopSource() {}

  /**
   * Initialise object (open files, etc.).  If object is already initialised
   * it is shutdown and reinitialised.
   *
   *   @return true on success
   */
  virtual bool init() = 0;

  /**
   * Blocks until teleop source device events detected or the given timeout is
   * reached.  When events occur, updates the teleop state.  Should only be
   * called between init() and shutdown().
   *
   *   @param listenTimeout [in] - timeout value in milliseconds
   *   @param teleopState [in/out] - the current teleop state, to be updated
   *   @param updated [out] - true if teleop state was updated
   *
   *   @return true on success
   */
  virtual bool listen(unsigned int listenTimeout, TeleopState* const teleopState, bool* updated) = 0;

  /**
   * Shutdown object (close files, etc.).  If object is already shutdown this
   * has no effect.  This method always cleans up as much as possible, even if
   * there are errors.  This method is always called on destruction.
   *
   *   @return true on success
   */
  virtual bool shutdown() = 0;

}; //class




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================
