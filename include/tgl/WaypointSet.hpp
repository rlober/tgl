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
using StdVectorXd = std::vector<Eigen::VectorXd>;           /*!< A std vector of dynamically sized Eigen double vectors. */
using StdDoubleVector = std::vector<double>;                /*!< A std vector of doubles. */
using StdWaypointVector = std::vector<Waypoint>;            /*!< A std vector of Waypoint objects. */
using WaypointMap = std::map<int, Waypoint>;                /*!< A map from a unique index to its corresponding Waypoint object. */
using wptMapIterator = std::map<int, Waypoint>::iterator;   /*!< Waypoint map iterator. */
using WaypointPair = std::pair<int, Waypoint>;              /*!< Key-value pair for inserting into a Waypoint map. */

/*! \class WaypointSet
 *  \brief This class groups a set of waypoints into a manageable unit which allows us to handle tricky operations like adding/removing and inserting waypoints to an existing set.
 *
 *  Useful functions are provided for accessing and modifying the waypoints in a safe manner.
 */
class WaypointSet {
public:

    /*! Basic constructor. Does nothing.
     */
    WaypointSet();

    /*! Initializing constructor. Creates a fully defined WaypointMap from a vector of waypoint coordinates. Initializes the waypoint times to 0.0 for each waypoint.
     *  \param wpts a StdVectorXd of waypoint coordinates
     */
    WaypointSet(const StdVectorXd& wpts);

    /*! Initializing constructor. Creates a fully defined WaypointMap from a vector of waypoint coordinates and times.
     *  \param wpts a StdVectorXd of waypoint coordinates
     *  \param wpt_times a StdDoubleVector of times for each of the waypoint coordinate vectors
     */
    WaypointSet(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    /*! Initializing constructor. Creates a fully defined WaypointMap from a vector of Waypoints.
     *  \param wptVec a StdWaypointVector of waypoints.
     */
    WaypointSet(const StdWaypointVector& wptVec);

    /*! Destuctor. Currently does nothing.
     */
    ~WaypointSet();

    /*! Sets the waypoints in the WaypointSet. Note: this is a clearing method and will erase any existing waypoints.
     *  \param wpts a StdVectorXd of waypoint coordinates
     */
    TglMessage setWaypoints(const StdVectorXd& wpts);

    /*! Sets the waypoints in the WaypointSet. Note: this is a clearing method and will erase any existing waypoints.
     *  \param wpts a StdVectorXd of waypoint coordinates
     *  \param wpt_times a StdDoubleVector of times for each of the waypoint coordinate vectors
     */
    TglMessage setWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    /*! Sets the waypoints in the WaypointSet. Note: this is a clearing method and will erase any existing waypoints.
     *  \param wptVec a StdWaypointVector of waypoints
     */
    TglMessage setWaypoints(const StdWaypointVector& wptVec);

    /*! Adds a single waypoint to the WaypointSet. The waypoint will be added to the end of the movement.
     *  \param wpt an Eigen::VectorXd representing the new waypoint coordinates
     */
    TglMessage addWaypoint(const Eigen::VectorXd& wpt);

    /*! Adds a single waypoint to the WaypointSet. The waypoint will be added at the time specified.
     *  \param wpt an Eigen::VectorXd representing the new waypoint coordinates
     *  \param wpt_time the time at which the waypoint should occur
     */
    TglMessage addWaypoint(const Eigen::VectorXd& wpt, const double wpt_time);

    /*! Adds a set of waypoints to the WaypointSet. The waypoints will be added to the end of the movement
     *  \param wpts a StdVectorXd of waypoint coordinates
     */
    TglMessage addWaypoints(const StdVectorXd& wpts);

    /*! Adds a set of waypoints to the WaypointSet. The waypoints will be added at the times specified.
     *  \param wpts a StdVectorXd of waypoint coordinates
     *  \param wpt_times a StdDoubleVector of times for each of the waypoint coordinate vectors
     */
    TglMessage addWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    /*! Returns an Eigen::MatrixXd object with the waypoint coordinates as column vectors.
        \f[
           \begin{bmatrix}
            x_0 & x_1 & \dots & x_n \\
            y_0 & y_1 & \dots & y_n \\
            z_0 & z_1 & \dots & z_n
            \end{bmatrix}
        \f]
     *  \param includeTimes put the waypoint times at the top of each waypoint column vector i.e.
         \f[
            \begin{bmatrix}
             t_0 & t_1 & \dots & t_n \\
             x_0 & x_1 & \dots & x_n \\
             y_0 & y_1 & \dots & y_n \\
             z_0 & z_1 & \dots & z_n
             \end{bmatrix}
         \f]
     *  \param useRowFormat return the matrix in row order format i.e.
        \f[
            \begin{bmatrix}
            x_0 & y_0 & z_0 \\
            x_1 & y_1 & z_1 \\
            \vdots & \vdots & \vdots \\
            x_n & y_n & z_n
            \end{bmatrix}
        \f]
     */
    Eigen::MatrixXd asMatrix(bool includeTimes = false, bool useRowFormat = false);

    /*! Get the waypoint times as a vector.
     *  \return An Eigen::VectorXd containing the waypoint times
     */
    Eigen::VectorXd getWaypointTimes();

    /*! Get the last waypoint time (equal to the total expected duration of the trajectory).
     *  \return The last waypoint vector time
     */
    double getLastWaypointTime();

    /*! Get the waypoint dimension. This is the number of DoF of the trajectory.
     *  \return The number of DoF of the trajectory
     */
    int getWaypointDimension();

    /*! Get the total number of waypoints.
     *  \return The number of waypoints in the set.
     */
    int getNumberOfWaypoints();

    /*! Get the waypoint at a specific time.
     *  \return An Eigen::VectorXd containing the waypoint coordinates
     */
    Eigen::VectorXd getWaypointAtTime(const double time_step);

private:

    /*! Sets the waypoints in the WaypointMap. Note: this is a clearing method and will erase any existing waypoints.
     *  \param wptVec a StdWaypointVector of waypoints
     */
    TglMessage setWaypointMap(const StdWaypointVector& wptVec);

    /*! Sets the fastVectors which are used to efficiently map to Eigen containers. Note: this is a clearing method and will erase the current vectors.
     */
    TglMessage fillFastWaypointVectors();


    WaypointMap wptMap;                         /*!< The waypoint map manipulared by this class. This is where we keep track of how the waypoints are arranged. */
    StdDoubleVector fastWptVector;              /*!< A contiguous vector of the waypoints flattened out. */
    StdDoubleVector fastWptTimesVector;         /*!< A contiguous vector of the waypoint times out. */
    StdDoubleVector fastWptVectorWithTimes;     /*!< A contiguous vector of the waypoint times and waypoints flattened out. */
};

} // end of namespace tgl
#endif // TGL_WAYPOINTSET_H
