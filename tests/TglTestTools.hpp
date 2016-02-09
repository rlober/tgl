#ifndef TGL_TGLTESTTOOLS_HPP
#define TGL_TGLTESTTOOLS_HPP

#include "tgl/Waypoints.hpp"

namespace tgl{
enum TglTestMessage {
    TGL_TEST_FAILURE = 0,
    TGL_TEST_SUCCESS = 1
};
static int TEST_COUNT=0; // increment for each new test case
static int TEST_SUCCESS_COUNT=0; // Number of successful tests

class WaypointTest{
public:
    WaypointTest(){TEST_COUNT++;}

    void runTest(const Waypoints& wptObject){
        TEST_SUCCESS_COUNT += doRunTest(wptObject);
    }

protected:

    virtual TglTestMessage doRunTest(const Waypoints& wptObject) = 0;
};




}


#endif //TGL_TGLTESTTOOLS_HPP
