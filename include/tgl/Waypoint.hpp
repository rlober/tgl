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
class Waypoint {
public:
    // Constructor functions
    Waypoint();
    Waypoint(const Eigen::VectorXd& newWpt);
    Waypoint(const Eigen::VectorXd& newWpt, double newWptTime);

    // Copy and Assignment
    Waypoint& operator=(Waypoint other);

    // Operator overloads
    bool operator==(Waypoint& other);
    Waypoint operator+(Waypoint& other);
    Waypoint operator-(Waypoint& other);
    Waypoint operator*(double scalar);
    Waypoint operator/(double scalar);
    // Waypoint operator+=(Waypoint& other);
    // Waypoint operator-=(Waypoint& other);
    // Waypoint operator*=(double scalar);
    // Waypoint operator/=(double scalar);

    TglMessage set(const Eigen::VectorXd& newWpt);
    TglMessage set(const Eigen::VectorXd& newWpt, double newWptTime);
    TglMessage setTime(double newWptTime);

    Eigen::VectorXd get();
    double getTime();
    int getDimension();


private:
    Eigen::VectorXd wpt;
    double wptTime;
};

} // end of namespace tgl
#endif // TGL_WAYPOINT_H
