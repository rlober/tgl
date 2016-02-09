#include "../TglTestTools.hpp"
#include "tgl/Waypoints.hpp"

using namespace tgl;

class ConstructorTest : public WaypointTest{
protected:
    TglTestMessage doRunTest(const Waypoints& wpt){
        std::cout << "Test 1" << std::endl;
        return TGL_TEST_SUCCESS;
    }
};



int main(int argc, char const *argv[]) {

    Waypoints wpt;
    /*
    *   Test Case 1
    */
    WaypointTest* test1 = new ConstructorTest;
    test1->runTest(wpt);


    if (TEST_COUNT == TEST_SUCCESS_COUNT) {
        std::cout << "All Waypoint class tests ("<< TEST_SUCCESS_COUNT <<"/"<< TEST_COUNT <<") passed!" << std::endl;
    }
    else {
        std::cout << TEST_SUCCESS_COUNT <<"/"<< TEST_COUNT << " Waypoint class tests failed:" << std::endl;
    }
    return TEST_COUNT == TEST_SUCCESS_COUNT;
}
