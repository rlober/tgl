/*! \file       Waypoint.hpp
 *  \brief      A class for defining waypoints which consist of a coordinate vector and a time step.
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

#ifndef TGL_WAYPOINT_H
#define TGL_WAYPOINT_H

// STL includes
#include <iostream>
#include <vector>
#include <map>
#include <utility>

// Eigen includes
#include <Eigen/Dense>

// Glog includes
#include <glog/logging.h>

// TGL includes
#include "tgl/TglTools.hpp"
#include "tgl/TglTypes.hpp"


namespace tgl
{
class Waypoint {
public:
    // Constructor functions
    Waypoint();
    Waypoint(const Eigen::VectorXd& newWpt);
    Waypoint(const Eigen::VectorXd& newWpt, double newWptTime);

    // Copy and Assignment
    Waypoint& operator=(Waypoint other);

    // Operator overloads
    bool operator==(Waypoint& other);
    Waypoint operator+(Waypoint& other);
    Waypoint operator-(Waypoint& other);
    Waypoint operator*(double scalar);
    Waypoint operator/(double scalar);
    // Waypoint operator+=(Waypoint& other);
    // Waypoint operator-=(Waypoint& other);
    // Waypoint operator*=(double scalar);
    // Waypoint operator/=(double scalar);

    TglMessage set(const Eigen::VectorXd& newWpt);
    TglMessage set(const Eigen::VectorXd& newWpt, double newWptTime);
    TglMessage setTime(double newWptTime);

    Eigen::VectorXd get();
    double getTime();
    int getDimension();


private:
    Eigen::VectorXd wpt;
    double wptTime;
};

} // end of namespace tgl
#endif // TGL_WAYPOINT_H
