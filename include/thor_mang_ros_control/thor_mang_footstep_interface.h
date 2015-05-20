//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
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

#include <hardware_interface/internal/hardware_resource_manager.h>



namespace hardware_interface
{
class ThorMangFootstepsHandle
{
public:
  ThorMangFootstepsHandle()
		: name_(""),
			com_(0),
			support_leg_(0)
  {}

	ThorMangFootstepsHandle(const std::string& name, double* com, std::string* support_leg)
		: name_(name),
			com_(com),
			support_leg_(support_leg)
  {}

  std::string getName() const { return name; }
	double getCOM() const { return *com_; }
	std::string getSupportLeg() const { return *support_leg_; }

private:
	std::string name_;
	double* com_;
	std::string support_leg_;
};

/** \brief Hardware interface to support reading the state of footstep parameters */
class ThorMangFootstepInterface : public HardwareResourceManager<ThorMangFootstepsHandle, ClaimResources> {};
}

#endif
