/*! \file       Trajectory.cpp
 *  \brief      A class for defining the basic interface for any specific type of trajectory.
 *  \details
 *  \author     Ryan Lober
 *  \version
 *  \date       Feb 2016
 *  \bug
 *  \warning
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of TGL (Trajectory Generation Library).
 *  Copyright (C) 2016 Institut des Systemes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "tgl/Trajectory.hpp"


using namespace tgl;

Trajectory::Trajectory():
internalClockResetTrigger(true)
{
}

Trajectory::Trajectory(const WaypointSet& newWptSet):
internalClockResetTrigger(true)
{
    if(!setWaypoints(newWptSet))
        LOG(ERROR) << "Could not set the waypoints you passed to the trajectory.";
}

Trajectory::~Trajectory()
{
}

TglMessage Trajectory::getDesired(  Eigen::VectorXd& desiredPos,
                                    Eigen::VectorXd& desiredVel,
                                    Eigen::VectorXd& desiredAcc,
                                    const double time_step)
{
    double tmp_time_step = time_step == TGL_USE_INTERNAL_CLOCK ? getInternalClockTime() : time_step;
    TglMessage implementationMessage = getImplementationDesired(desiredPos, desiredVel, desiredAcc, tmp_time_step);
    /*TODO:
     *  Implement Quaternion SLERP and derivation for angular velocity and acceleration.
     *  Concatenate results to desiredPos/Vel/Acc
     */
    if (implementationMessage == TGL_FINISHED) {
        resetInternalClock();
    }
    return implementationMessage;
}

TglMessage Trajectory::getDesired(  Eigen::VectorXd& desiredPos,
                                    Eigen::VectorXd& desiredVel,
                                    Eigen::VectorXd& desiredAcc,
                                    const Eigen::VectorXd& currentPos,
                                    const Eigen::VectorXd& currentVel,
                                    const Eigen::VectorXd& currentAcc,
                                    const double time_step)
{
    double tmp_time_step = time_step == TGL_USE_INTERNAL_CLOCK ? getInternalClockTime() : time_step;
    /* TODO:
     * Separate the rotation compenents from the linear components and pass those to the implementations.
     */
    TglMessage implementationMessage = getImplementationDesired(desiredPos, desiredVel, desiredAcc, currentPos, currentVel, currentAcc, tmp_time_step);
    /*TODO:
     *  Implement Quaternion SLERP and derivation for angular velocity and acceleration.
     *  Concatenate results to desiredPos/Vel/Acc
     */
     if (implementationMessage == TGL_FINISHED) {
         resetInternalClock();
     }
     return implementationMessage;
}


TglMessage Trajectory::getImplementationDesired(Eigen::VectorXd& desiredPos,
                                                Eigen::VectorXd& desiredVel,
                                                Eigen::VectorXd& desiredAcc,
                                                const double time_step)
{
    LOG(ERROR) << "The trajectory you are using has not implemented an open-loop generator!";
    return TGL_ERROR;
}

TglMessage Trajectory::getImplementationDesired(Eigen::VectorXd& desiredPos,
                                                Eigen::VectorXd& desiredVel,
                                                Eigen::VectorXd& desiredAcc,
                                                const Eigen::VectorXd& currentPos,
                                                const Eigen::VectorXd& currentVel,
                                                const Eigen::VectorXd& currentAcc,
                                                const double time_step)
{
    LOG(ERROR) << "The trajectory you are using has not implemented a closed-loop generator!";
    return TGL_ERROR;
}

TglMessage Trajectory::setWaypoints(const WaypointSet& newWptSet)
{
    wptSet = newWptSet;
    resetInternalClock();
}

TglMessage Trajectory::getWaypoints(WaypointSet& newWptSet)
{
    if (!wptSet.empty()) {
        newWptSet = wptSet;
        return TGL_OK;
    }
    return TGL_ERROR;
}

TglMessage Trajectory::resetInternalClock()
{
    internalClockResetTrigger = true;
    return TGL_OK;
}

double Trajectory::getInternalClockTime()
{
    if (internalClockResetTrigger) {
        internalClockStartTime = std::chrono::system_clock::now();
        internalClockResetTrigger = false;
    }

    return std::chrono::duration<double>(std::chrono::system_clock::now() - internalClockStartTime).count();
}
