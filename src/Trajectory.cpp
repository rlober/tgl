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

TglMessage Trajectory::getDesired(Eigen::MatrixXd& desired)
{
    return getDesired(getInternalClockTime(), desired);
}

TglMessage Trajectory::getDesired(const double time_step, Eigen::MatrixXd& desired)
{
    LOG(ERROR) << "The trajectory you are using has not implemented this function!";
    return TGL_ERROR;
}

TglMessage Trajectory::getDesired(const Eigen::MatrixXd& current, Eigen::MatrixXd& desired)
{
    LOG(ERROR) << "The trajectory you are using has not implemented this function!";
    return TGL_ERROR;
}

TglMessage Trajectory::getDesired(const double time_step, const Eigen::MatrixXd& current, Eigen::MatrixXd& desired)
{
    LOG(ERROR) << "The trajectory you are using has not implemented this function!";
    return TGL_ERROR;
}

TglMessage Trajectory::setWaypoints(const WaypointSet& newWptSet)
{
    wptSet = newWptSet;
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
