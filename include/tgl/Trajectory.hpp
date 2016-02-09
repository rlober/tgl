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




namespace tgl
{

using Waypoints = std::vector<Eigen::VectorXd>;
using WaypointTimes = std::vector<double>;

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
    Waypoints wpts;
    WaypointTimes wptimes;

};

} // end of namespace tgl
#endif // TGL_TRAJECTORY_H
