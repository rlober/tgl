#include "../TglTestTools.hpp"
#include "tgl/WaypointSet.hpp"

using namespace tgl;

/*************************************************
*
*   Test definitions
*
* Example test pattern:

class MyTest : public TglTest{
protected:
    TglTestMessage test(){
        WaypointSet wpts;
        // do stuff with wpts
        if // good result
            return TGL_TEST_SUCCESS;
        else // bad result
            return TGL_TEST_FAILURE;
    }
};

*************************************************/

class ConstructorTest : public TglTest{
protected:
    TglTestMessage test(){
        int nDof = 3; Eigen::VectorXd onesVec = Eigen::VectorXd::Ones(nDof);
        StdVectorXd wpt_vector = {onesVec*1.0, onesVec*2.0, onesVec*3.0};
        StdDoubleVector wpt_times = {0.0, 1.1, 2.1};
        WaypointSet wpt1;
        WaypointSet wpt2(wpt_vector);
        WaypointSet wpt3(wpt_vector, wpt_times);
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
