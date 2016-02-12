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

WaypointSet::WaypointSet()
{
    LOG(ERROR) << "This is a test.";
}

WaypointSet::WaypointSet(const StdVectorXd& wpts)
{
    if(!setWaypoints(wpts)){
        std::cout << "error" << std::endl;
    }
}

WaypointSet::WaypointSet(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    if(!setWaypoints(wpts, wpt_times)){
        std::cout << "error" << std::endl;
    }
}

WaypointSet::~WaypointSet()
{

}

TglMessage WaypointSet::setWaypoints(const StdVectorXd& wpts)
{
    StdDoubleVector wpt_times;
    for(auto&& i:wpts)
        wpt_times.push_back(0.0);

    return setWaypoints(wpts, wpt_times);
}

TglMessage WaypointSet::setWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    assert(wpt_times.size() == wpts.size());
    int expectedVectorDimension = wpts[0].size();
    bool sizeOk;
    StdWaypointVector wptVec;
    for(size_t i; i<wpts.size(); i++)
    {
        assert(wpts[i].size() == expectedVectorDimension);
        wptVec.push_back(Waypoint(wpts[i], wpt_times[i]));
    }
    return setWaypointMap(wptVec);
}

TglMessage WaypointSet::setWaypointMap(const StdWaypointVector& wptVec)
{
    int wptID = 0;
    bool insertOk=true;
    for(auto wpt : wptVec){
        insertOk &= wptMap.insert(WaypointPair(wptID, wpt)).second;
        wptID++;
    }

    return insertOk ? TGL_OK : TGL_ERROR;
}
