#ifndef TGL_WAYPOINTS_H
#define TGL_WAYPOINTS_H

// STL includes
#include <iostream>
#include <vector>
#include <map>

// Eigen includes
#include <Eigen/Dense>

// Glog includes
#include <glog/logging.h>

// TGL includes
#include "tgl/TglTools.hpp"
#include "tgl/TglTypes.hpp"


namespace tgl
{
using StdVectorXd = std::vector<Eigen::VectorXd>;
using StdDoubleVector = std::vector<double>;
using WaypointMap = std::map<double, Eigen::VectorXd>; // Key = time_step, Value = VectorXd Waypoint

class Waypoints {
public:
    // Constructor functions
    Waypoints();
    Waypoints(const StdVectorXd& wpts);
    Waypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);

    //Destructor
    ~Waypoints();

    //Set Waypoints
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
    TglMessage setWaypointMap(const StdVectorXd& wpts, const StdDoubleVector& wpt_times);


    WaypointMap wptMap;

};

} // end of namespace tgl
#endif // TGL_WAYPOINTS_H
