#include "tgl/Waypoints.hpp"


using namespace tgl;

Waypoints::Waypoints()
{
    LOG(ERROR) << "This is a test.";
}

Waypoints::Waypoints(const StdVectorXd& wpts)
{
    if(!setWaypoints(wpts)){
        std::cout << "error" << std::endl;
    }
}

Waypoints::Waypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    if(!setWaypoints(wpts, wpt_times)){
        std::cout << "error" << std::endl;
    }
}

Waypoints::~Waypoints()
{

}

TglMessage Waypoints::setWaypoints(const StdVectorXd& wpts)
{
    return TGL_OK;
}

TglMessage Waypoints::setWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    return TGL_OK;
}
