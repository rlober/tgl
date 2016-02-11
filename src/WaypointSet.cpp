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
    return TGL_OK;
}

TglMessage WaypointSet::setWaypoints(const StdVectorXd& wpts, const StdDoubleVector& wpt_times)
{
    return TGL_OK;
}
