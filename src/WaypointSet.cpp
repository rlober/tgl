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

#include "tgl/WaypointSet.hpp"


using namespace tgl;

/****************************************************
                   Public Functions
 ****************************************************/

WaypointSet::WaypointSet()
{
}

WaypointSet::WaypointSet(const StdVectorXd& wpts)
{
    if(!setWaypoints(wpts)){
        LOG(ERROR) << "Unable to properly set waypoints.";
    }
}

WaypointSet::WaypointSet(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    if(!setWaypoints(wpts, wpt_times)){
        LOG(ERROR) << "Unable to properly set waypoints.";
    }
}

WaypointSet::WaypointSet(const StdWaypointVector& wptVec)
{
    if(!setWaypointMap(wptVec)){
        LOG(ERROR) << "Unable to properly set waypoints.";
    }
}

WaypointSet::~WaypointSet()
{
}

TglMessage WaypointSet::setWaypoints(const StdVectorXd& wpts)
{
    StdDoubleVector wpt_times;
    for(auto i:wpts)
        wpt_times.push_back(0.0);

    return setWaypoints(wpts, wpt_times);
}

TglMessage WaypointSet::setWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    if(wpt_times.size() == wpts.size())
    {
        int expectedVectorDimension = wpts[0].size();
        bool sizeOk;
        StdWaypointVector wptVec;
        for(size_t i=0; i<wpts.size(); i++)
        {
            if(wpts[i].size() == expectedVectorDimension){
                wptVec.push_back(Waypoint(wpts[i], wpt_times[i]));
            }else{
                LOG(WARNING) << "You are trying to mix waypoints of different sizes! Be careful.";
                // TODO: Pad with zeros rather than replace with a zero vec.
                wptVec.push_back(Waypoint(Eigen::VectorXd::Zero(expectedVectorDimension), wpt_times[i]));
            }
        }
        return setWaypointMap(wptVec);
    }
    else
    {
        LOG(ERROR) << "Waypoint and Waypoint Times vectors are not the same size.";
        return TGL_ERROR;
    }
}

TglMessage WaypointSet::setWaypoints(const StdWaypointVector& wptVec)
{
    return setWaypointMap(wptVec);
}

TglMessage WaypointSet::addWaypoint(const Eigen::VectorXd& wpt)
{
    //TODO: implement
}

TglMessage WaypointSet::addWaypoint(const Eigen::VectorXd& wpt, const double wpt_time)
{
    //TODO: implement
}

TglMessage WaypointSet::addWaypoints(const StdVectorXd& wpts)
{
    //TODO: implement
}

TglMessage WaypointSet::addWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    //TODO: implement
}

Eigen::MatrixXd WaypointSet::asMatrix(bool includeTimes, bool useRowFormat)
{
    if(!wptMap.empty()){
        if (includeTimes) {
            Eigen::Map<Eigen::MatrixXd> mat(fastWptVectorWithTimes.data(), getWaypointDimension()+1, getNumberOfWaypoints());
            if (useRowFormat) {
                return mat.transpose();
            }
            return mat;
        } else {
            Eigen::Map<Eigen::MatrixXd> mat(fastWptVector.data(), getWaypointDimension(), getNumberOfWaypoints());
            if (useRowFormat) {
                return mat.transpose();
            }
            return mat;
        }
    }else{
        // Map is empty so just return a Zero mat
        LOG(WARNING) << "Waypoint set is empty.";
        return Eigen::MatrixXd(0,0);
    }
}

Eigen::VectorXd WaypointSet::getWaypointTimes()
{
    return Eigen::Map<Eigen::VectorXd>(fastWptTimesVector.data(), getNumberOfWaypoints());
}

int WaypointSet::getNumberOfWaypoints()
{
    return wptMap.size();
}

int WaypointSet::getWaypointDimension()
{
    if(!wptMap.empty())
        return wptMap.begin()->second.getDimension();

    else
        return 0;
}

Eigen::VectorXd WaypointSet::getWaypointAtTime(const double time_step)
{
    if(!wptMap.empty()){
        // Iterate through the map and compare the wpt times to the desired time.
        for(auto&& wpt : wptMap){
            if (time_step == wpt.second.getTime()) {
                return wpt.second.get();
            }
        }
        // If none of the times match return a vector of zeros.
        return Eigen::VectorXd::Zero(getWaypointDimension());
    }else{
        //If the map is empty then throw a warning and return a vector of zeros.
        LOG(ERROR) << "The WaypointSet is empty.";
        return Eigen::VectorXd::Zero(getWaypointDimension());
    }
}

/****************************************************
                   Private Functions
 ****************************************************/

TglMessage WaypointSet::setWaypointMap(const StdWaypointVector& wptVec)
{
    int wptID = 0;
    bool insertOk=true;
    for(auto wpt : wptVec){
        insertOk &= wptMap.insert(WaypointPair(wptID, wpt)).second;
        wptID++;
    }
    if (insertOk) {
        fillFastWaypointVectors();
    }


    return insertOk ? TGL_OK : TGL_ERROR;
}

TglMessage WaypointSet::fillFastWaypointVectors()
{
    if(!wptMap.empty())
    {
        fastWptVector.clear();
        fastWptVectorWithTimes.clear();
        for(wptMapIterator it = wptMap.begin(); it != wptMap.end(); ++it)
        {
            fastWptTimesVector.push_back(it->second.getTime());
            fastWptVectorWithTimes.push_back(it->second.getTime());
            for(auto dof = 0; dof < it->second.get().size(); ++dof){
                fastWptVector.push_back( it->second.get()(dof) );
                fastWptVectorWithTimes.push_back( it->second.get()(dof) );
            }
        }
        return TGL_OK;
    }else{
        LOG(WARNING) << "Waypoint set is empty.";
        return TGL_ERROR;
    }
}
