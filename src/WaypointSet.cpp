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
