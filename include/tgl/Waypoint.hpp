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

    /*! Initializing constructor. Creates a waypoint from a vector of waypoint coordinates. Initializes the waypoint time to 0.0.
     *  \param newWpt a vector of waypoint coordinates
     */
    Waypoint(const Eigen::VectorXd& newWpt);

    /*! Initializing constructor. Creates a waypoint from a vector of waypoint coordinates and their associated time.
     *  \param newWpt a vector of waypoint coordinates
     *  \param newWptTime the time at which the waypoint should occur
     */
    Waypoint(const Eigen::VectorXd& newWpt, double newWptTime);

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

    /*! Sets the waypoint from a vector of waypoint coordinates. Initializes the waypoint time to 0.0. Note: This will erase any existing waypoint data.
     *  \param newWpt a vector of waypoint coordinates
     */
    TglMessage set(const Eigen::VectorXd& newWpt);

    /*! Sets the waypoint from a vector of waypoint coordinates and their associated time. Note: This will erase any existing waypoint data.
     *  \param newWpt a vector of waypoint coordinates
     *  \param newWptTime the time at which the waypoint should occur
     */
    TglMessage set(const Eigen::VectorXd& newWpt, double newWptTime);

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

    /*! Get the waypoint time.
     *  \return The waypoint time.
     */
    double getTime();

    /*! Get the dimension of the waypoint coordinates. This is the DoF.
     *  \return The waypoint dimension.
     */
    int getDimension();


private:
    Eigen::VectorXd wpt;    /*!< The waypoint coordinate vector. */
    double wptTime;         /*!< The waypoint time. */
};

} // end of namespace tgl
#endif // TGL_WAYPOINT_H
