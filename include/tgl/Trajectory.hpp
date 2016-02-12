/*! \file       Trajectory.hpp
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

#ifndef TGL_TRAJECTORY_H
#define TGL_TRAJECTORY_H

// STL includes
#include <iostream>
#include <vector>

// Eigen includes
#include <Eigen/Dense>

// TGL includes
#include "tgl/TglTools.hpp"
#include "tgl/TglTypes.hpp"
#include "tgl/WaypointSet.hpp"





namespace tgl
{

class Trajectory {
public:
    // Constructor function
    Trajectory();
    //Destructor
    ~Trajectory();

    // Open Loop
    virtual TglMessage getDesired(Eigen::MatrixXd& desired) = 0;
    virtual TglMessage getDesired(const double time_step, Eigen::MatrixXd& desired) = 0;

    // Closed Loop
    virtual TglMessage getDesired(const Eigen::MatrixXd& current, Eigen::MatrixXd& desired) = 0;
    virtual TglMessage getDesired(const double time_step, const Eigen::MatrixXd& current, Eigen::MatrixXd& desired) = 0;

protected:
    WaypointSet wptSet;

};

} // end of namespace tgl
#endif // TGL_TRAJECTORY_H
