/*! \file       WaypointSet.hpp
 *  \brief      A class for defining a set of waypoints and manipulating them properly.
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

#ifndef TGL_WAYPOINTSET_H
#define TGL_WAYPOINTSET_H

// STL includes
#include <iostream>
#include <vector>
#include <map>
#include <cassert>

// Eigen includes
#include <Eigen/Dense>

// Glog includes
#include <glog/logging.h>

// TGL includes
#include "tgl/TglTools.hpp"
#include "tgl/TglTypes.hpp"
#include "tgl/Waypoint.hpp"


namespace tgl
{
using StdVectorXd = std::vector<Eigen::VectorXd>;
using StdDoubleVector = std::vector<double>;
using StdWaypointVector = std::vector<Waypoint>;
using WaypointMap = std::map<int, Waypoint>; // Key = index, Value = VectorXd Waypoint
using WaypointPair = std::pair<int, Waypoint>;

class WaypointSet {
public:
    // Constructor functions
    WaypointSet();
    WaypointSet(const StdVectorXd& wpts);
    WaypointSet(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    //Destructor
    ~WaypointSet();

    //Set WaypointSet
    TglMessage setWaypoints(const StdVectorXd& wpts);
    TglMessage setWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    //Add waypoints
    TglMessage addWaypoint(const Eigen::VectorXd& wpt);
    TglMessage addWaypoint(const Eigen::VectorXd& wpt, const double wpt_time);
    TglMessage addWaypoints(const StdVectorXd& wpts);
    TglMessage addWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    //Eigen convertors
    Eigen::MatrixXd asMatrix();

    //Getters
    Eigen::VectorXd getWaypointTimes();
    double getLastWaypointTime();
    int getWaypointDimension();
    int getNumberOfWaypoints();
    Eigen::VectorXd getWaypointAtTime(const double time_step);

private:
    TglMessage setWaypointMap(const StdWaypointVector& wptVec);

    WaypointMap wptMap;

};

} // end of namespace tgl
#endif // TGL_WAYPOINTSET_H
