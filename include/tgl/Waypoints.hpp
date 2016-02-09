#ifndef TGL_WAYPOINTS_H
#define TGL_WAYPOINTS_H

// STL includes
#include <iostream>
#include <vector>
#include <map>

// Eigen includes
#include <Eigen/Dense>

// TGL includes
#include "tgl/TglTools.hpp"
#include "tgl/TglTypes.hpp"


namespace tgl
{
using StdVectorXd = std::vector<Eigen::VectorXd>;
using WaypointMap = std::map<double, Eigen::VectorXd>; // Key = time_step, Value = VectorXd Waypoint

class Waypoints {
public:
    // Constructor function
    Waypoints();
    //Destructor
    ~Waypoints();

protected:

private:

};

} // end of namespace tgl
#endif // TGL_WAYPOINTS_H
