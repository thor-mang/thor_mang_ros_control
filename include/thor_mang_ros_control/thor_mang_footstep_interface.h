//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HARDWARE_INTERFACE_FOOTSTEPS_INTERFACE_H
#define HARDWARE_INTERFACE_FOOTSTEPS_INTERFACE_H

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <hardware_interface/internal/hardware_resource_manager.h>



namespace hardware_interface
{
class ThorMangFootstepsHandle
{
public:
  ThorMangFootstepsHandle()
    : name("")
    , dynamixel_mutex(NULL)
  {}

  ThorMangFootstepsHandle(const std::string& name, boost::mutex* dynamixel_mutex)
    : name(name)
    , dynamixel_mutex(dynamixel_mutex)
  {}

  std::string getName() const { return name; }
  boost::mutex& getDynamixelMutex() const { return *dynamixel_mutex; }

private:
  std::string name;
  boost::mutex* dynamixel_mutex;
};

/** \brief Hardware interface to support reading the state of footstep parameters */
class ThorMangFootstepInterface : public HardwareResourceManager<ThorMangFootstepsHandle, ClaimResources> {};
}

#endif
