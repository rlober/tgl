/*! \file       Waypoint.hpp
 *  \brief      A class for defining waypoints which consist of a coordinate vector and a time step.
 *  \details
 *  \author     Ryan Lober
 *  \version
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 *  \bug        Wrench waypoint consrtuctor `Waypoint(const Eigen::Wrenchd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);` has an ambiguity conflict with the `Eigen::VectorXd` constructor. Need to resolve this.
 */
/*
 *  This file is part of TGL (Trajectory Generation Library).
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
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
#include <Eigen/Lgsm>

// Glog includes
#include <glog/logging.h>

// TGL includes
#include "tgl/TglTools.hpp"
#include "tgl/TglTypes.hpp"

#ifndef TGL_WAYPOINT_TIME_NOT_SPECIFIED
#define TGL_WAYPOINT_TIME_NOT_SPECIFIED -1.0
#endif

namespace tgl
{

/*! \class Waypoint
 *  \brief A class for defining waypoints generically and in any space.
 *
 *  This class basically just provides a nice way of coupling a waypoint coordinate vector and its corresponding time step
 */
class Waypoint {
public:

    /*! Basic constructor. Does nothing.
     */
    Waypoint();

    /*! Initializing constructor. Creates a waypoint from a vector of waypoint coordinates and their associated time.
     *  \param newWpt a vector of waypoint coordinates
     *  \param newWptTime the time at which the waypoint should occur
     */
    Waypoint(const Eigen::VectorXd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    /*! Initializing constructor. Creates a waypoint from a Displacementd object which contains both position and orientation.
     *  \param newWpt a Displacementd waypoint with position and orientation
     *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
     */
    Waypoint(const Eigen::Displacementd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    /*! Initializing constructor. Creates a waypoint from a Rotation3d object which contains a quaternion orientation.
     *  \param newWpt a quaternion orientation waypoint
     *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
     */
    Waypoint(const Eigen::Rotation3d& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    //TODO: Fix this BUG.

    // /* Initializing constructor. Creates a waypoint from a Wrenchd object which contains both torque and force.
    //  *  \param newWpt a waypoint in torque and force
    //  *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
    //  */
    // Waypoint(const Eigen::Wrenchd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    //TODO: Implement KDL versions of this.

    /*! Assignment operator.
     */
    Waypoint& operator=(Waypoint other);

    /*! Equivalence operator. Operates only on the internal waypoint coordinates (e.g. Eigen::VectorXd);
     */
    bool operator==(Waypoint& other);

    /*! Addition operator. Operates only on the internal waypoint coordinates (e.g. Eigen::VectorXd);
     */
    Waypoint operator+(Waypoint& other);

    /*! Subtraction operator. Operates only on the internal waypoint coordinates (e.g. Eigen::VectorXd);
     */
    Waypoint operator-(Waypoint& other);

    /*! Multiplication operator. Operates only on the internal waypoint coordinates (e.g. Eigen::VectorXd);
     */
    Waypoint operator*(double scalar);

    /*! Divison operator. Operates only on the internal waypoint coordinates (e.g. Eigen::VectorXd);
     */
    Waypoint operator/(double scalar);

    // TODO: Implement all this shit.
    // Waypoint operator+=(Waypoint& other);
    // Waypoint operator-=(Waypoint& other);
    // Waypoint operator*=(double scalar);
    // Waypoint operator/=(double scalar);

    /*! Sets the waypoint from a vector of waypoint coordinates and their associated time.
     *  \warning This will erase any existing waypoint data.
     *  \param newWpt a vector of waypoint coordinates
     *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
     */
    TglMessage set(const Eigen::VectorXd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    /*! Sets the waypoint from a Displacementd object which contains both position and orientation.
     *  \warning This will erase any existing waypoint data.
     *  \param newWpt a Displacementd waypoint with position and orientation
     *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
     */
    TglMessage set(const Eigen::Displacementd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    /*! Sets the waypoint from a Rotation3d object which contains a quaternion orientation.
     *  \warning This will erase any existing waypoint data.
     *  \param newWpt a quaternion orientation waypoint
     *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
     */
    TglMessage set(const Eigen::Rotation3d& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    /*! Sets the waypoint from a Wrenchd object which contains both torque and force.
     *  \warning This will erase any existing waypoint data.
     *  \param newWpt a waypoint in torque and force
     *  \param newWptTime the time at which the waypoint should occur. *If this is not specified then the waypoint time will not be set.*
     */
    TglMessage set(const Eigen::Wrenchd& newWpt, double newWptTime = TGL_WAYPOINT_TIME_NOT_SPECIFIED);

    //TODO: Implement KDL versions of this.

    /*! Sets only the waypoint time. Note: This will erase the existing waypoint time.
     *  \param newWptTime the time at which the waypoint should occur
     */
    TglMessage setTime(double newWptTime);

    /*! Gets the waypoint coordinates as an Eigen::VectorXd.
        \f[
           \begin{bmatrix}
            x \\
            y \\
            z
            \end{bmatrix}
        \f]
     *  \param includeTimes put the waypoint time at the top of the returned vector waypoint column vector i.e.
         \f[
            \begin{bmatrix}
             t \\
             x \\
             y \\
             z
             \end{bmatrix}
         \f]
     *  \return A vector of waypoint coordinates.
     */
    Eigen::VectorXd get(bool includeTimes = false);

    /*! Get the waypoint quaternion if one exists.
     *  \return The waypoint quaternion.
     *  \warning If the waypoint type does not implicitly contain a rotation then an Identity quaternion will be returned.
     */
    Eigen::Rotation3d getRotation();

    /*! Get the waypoint time.
     *  \return The waypoint time.
     */
    double getTime();

    /*! Get the dimension of the waypoint coordinates. This is the DoF.
     *  \return The waypoint dimension.
     */
    int getDimension();

    /*! Get the type of object used to construct the waypoint.
     *  \return The type of waypoint used as a TglWaypointType
     */
     TglWaypointType type();

     /*! Get the type of object used to construct the waypoint.
      *  \return The type of waypoint used as a TglWaypointType
      */
     bool hasRotation();


private:

    /*! Sets the waypoint vector exposed to the trajectory implementations.
     *  \param newWpt a vector of waypoint coordinates
     *  \param newWptTime the time at which the waypoint should occur.
     */
    TglMessage setInternalVariables(const Eigen::VectorXd& newWpt, double newWptTime);

    /*! Sets the waypoint quaternion handled internally here.
     *  \param newQuat the waypoint's quaternion
     */
    TglMessage setInternalRotation(const Eigen::Rotation3d& newQuat);

    /*! Sets the waypoint type.
     *  \param newType the waypoint type
     */
    TglMessage setType(TglWaypointType newType);

    Eigen::VectorXd wpt;            /*!< The waypoint coordinate vector. */
    Eigen::Rotation3d wptRotation;  /*!< The rotation components of a waypoint. */
    double wptTime;                 /*!< The waypoint time. */
    TglWaypointType wptType;        /*!< The type of representation used to construct the waypoint. */
};

} // end of namespace tgl
#endif // TGL_WAYPOINT_H
