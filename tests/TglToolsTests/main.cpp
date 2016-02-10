#include "../TglTestTools.hpp"
#include "tgl/TglTools.hpp"
using namespace tgl;

/*************************************************
*
*   Test definitions
*
* Example test pattern:

class MyTest : public TglTest{
protected:
    TglTestMessage test(){
        Waypoints wpt;
        // do stuff with wpt
        if // good result
            return TGL_TEST_SUCCESS;
        else // bad result
            return TGL_TEST_FAILURE;
    }
};

*************************************************/

class ChronoTest : public TglTest{
protected:
    TglTestMessage test(){
        TGL_CHRONO_START
        fibonacci(20);
        TGL_CHRONO_STOP
        TGL_CHRONO_START
        fibonacci(30);
        TGL_CHRONO_STOP
        TGL_CHRONO_START
        fibonacci(40);
        TGL_CHRONO_STOP
        return TGL_TEST_SUCCESS;
    }
    long fibonacci(unsigned n)
    {
        if (n < 2) return n;
        return fibonacci(n-1) + fibonacci(n-2);
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

    testVector.push_back(new ChronoTest);

    /*****************************************/
    return runAllTests(testVector);
}
