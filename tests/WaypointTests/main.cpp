#include "../TglTestTools.hpp"
#include "tgl/Waypoints.hpp"

using namespace tgl;

/*************************************************
*
*   Test definitions
*
* Example test pattern:

class BlahTest : public TglTest{
protected:
    TglTestMessage doRunTest(){
        Waypoints wpt;
        // do stuff with wpt
        if // good result
            return TGL_TEST_SUCCESS;
        else // bad result
            return TGL_TEST_FAILURE;
    }
};

*************************************************/

class ConstructorTest : public TglTest{
protected:
    TglTestMessage doRunTest(){
        Waypoints wpt;
        return TGL_TEST_SUCCESS;
    }
};

/*************************************************
*
*   main
*
*************************************************/

int main(int argc, char const *argv[])
{
    std::vector<TglTest*> testVector;
    /*****************************************/
    /*
    *   Make sure to add your test to the list
    *   e.g. testVector.push_back(new BlahTest);
    */
    testVector.push_back(new ConstructorTest);

    /*****************************************/
    return runAllTests(testVector);
}
