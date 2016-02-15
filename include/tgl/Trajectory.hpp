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
/*! \class Trajectory
 *  \brief An interface class for generic trajectories in any space.
 *
 *  This class provides the base interfaces for any type of TGL trajectory. By providing a generic interface, specific implementations can be interchanged effortlessly by simply pointing and object of this base Trjactory class at a particular implementation i.e.
    ~~~~~~~~~~~~~~{.cpp}
    tgl::Trajectory* myTraj = new tgl::CustomTrajectory();
    ~~~~~~~~~~~~~~
    Then to use the class, the base getter and setters can be called.
 */
class Trajectory {
public:

    /*! Basic constructor. Does nothing.
     */
    Trajectory();

    /*! Basic destructor. Does nothing.
     */
    virtual ~Trajectory();

    /*! Get the desired values from the trajectory. This function uses an internal clock to keep track of the time evolution. **Open Loop**
     *  \param desired a reference to an Eigen::MatrixXd which will be filled by the trajectory implementation
     *  \return A TglMessage indicating the status of the trajectory (see TglTypes.hpp)
     */
    virtual TglMessage getDesired(Eigen::MatrixXd& desired) = 0;

    /*! Get the desired values from the trajectory. **Open Loop**
     *  \param time_step the time with which to calculate the desired values.
     *  \param desired a reference to an Eigen::MatrixXd which will be filled by the trajectory implementation
     *  \return A TglMessage indicating the status of the trajectory (see TglTypes.hpp)
     */
    virtual TglMessage getDesired(const double time_step, Eigen::MatrixXd& desired) = 0;

    /*! Get the desired values from the trajectory. **Closed Loop**
     *  \param current the current state of the system being controlled by the trajectory.
     *  \param desired a reference to an Eigen::MatrixXd which will be filled by the trajectory implementation
     *  \return A TglMessage indicating the status of the trajectory (see TglTypes.hpp)
     */
    virtual TglMessage getDesired(const Eigen::MatrixXd& current, Eigen::MatrixXd& desired) = 0;

    /*! Get the desired values from the trajectory. **Closed Loop**
     *  \param time_step the time with which to calculate the desired values.
     *  \param current the current state of the system being controlled by the trajectory.
     *  \param desired a reference to an Eigen::MatrixXd which will be filled by the trajectory implementation
     *  \return A TglMessage indicating the status of the trajectory (see TglTypes.hpp)
     */
    virtual TglMessage getDesired(const double time_step, const Eigen::MatrixXd& current, Eigen::MatrixXd& desired) = 0;

protected:
    WaypointSet wptSet; /*!< The Waypoint Set for the trajectory. */

};

} // end of namespace tgl
#endif // TGL_TRAJECTORY_H
