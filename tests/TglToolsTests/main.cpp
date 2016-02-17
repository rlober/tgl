/*! \file       main.cpp
 *  \brief      Tests for the TGL tools.
 *  \details
 *  \author     Ryan Lober
 *  \version
 *  \date       Feb 2016
 *  \bug
 *  \warning
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of TGL (Trajectory Generation Library).
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
