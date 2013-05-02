/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// #include "dynamics/SensorDynamics.h"
#include "sensors/ImuSensor.h"
#include "kinematics/Joint.h"
#include "kinematics/Shape.h"
#include "kinematics/Transformation.h"
#include "utils/UtilsMath.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace kinematics;

namespace sensors{
    ImuSensor::ImuSensor( const char *_name,
        const dynamics::BodyNodeDynamics *_parent ) : SingletonSensor(_name, _parent){
    }

    ImuSensor::~ImuSensor(){
    }

    Eigen::Vector4d ImuSensor::pollLatest(){
      Eigen::Vector3d w = mParentBodyNode->mOmegaBody; // angular velocity in local frame
      Eigen::Matrix4d m = mParentBodyNode->getWorldTransform(); // get the world transform

			double thetaX, thetaY, thetaZ;

			// X and Y angles from transformation
			if (m(2,0) < +1) {
				if (m(2,0) > -1) {
					thetaY = asin(-m(2,0));
					thetaZ = atan2(m(1,0),m(0,0));
					thetaX = atan2(m(2,1),m(2,2));
				} else { // r20 = -1
					// Not a unique solution:  thetaX - thetaZ = atan2(-r12,r11)
					thetaY = +M_PI/2;
					thetaZ = -atan2(-m(1,2),m(1,1));
					thetaX = 0;
				}
			} else { // r20 = +1
				// Not a unique solution:  thetaX + thetaZ = atan2(-r12,r11)
				thetaY = -M_PI/2;
				thetaZ = atan2(-m(1,2),m(1,1));
				thetaX = 0;
			}

      return Vector4d(w[0], w[1], thetaX, thetaY);
    }
}



