/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CASSIE_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CASSIE_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief Kinematics of a Cassie robot (copied from srb model from dairlib/centroidal_models)
 */
class CassieKinematicModel : public KinematicModel {
public:
  CassieKinematicModel () : KinematicModel(2)
  {
    const double z_nominal_b = -0.7;  // Nominal z deviation of legs (aka nominal stance height)
    const double y_nominal_b =  0.15;  // Nominal stance width

    nominal_stance_.at(L) << 0.0,  y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << 0.0, -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_  << 0.4, 0.3, 0.4;
  }
};

/**
 * @brief SRB dynamics of Cassie robot (copied from dairlib/centroidal_models)
 */
class CassieDynamicModel : public SingleRigidBodyDynamics {
public:
  CassieDynamicModel()
  : SingleRigidBodyDynamics(30.0218,
                    0.91,0.55,0.89,0.00,0.00,0.00,
                    2) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CASSIE_MODEL_H_ */
