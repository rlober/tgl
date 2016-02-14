/*! \file       main.cpp
 *  \brief      Tests for the WaypointSet class.
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
 *  Copyright (C) 2016 Institut des Systemes Intelligents et de Robotique (ISIR)
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

        Eigen::MatrixXd testMat(nDof,3); testMat << onesVec*1.0, onesVec*2.0, onesVec*3.0;
        Eigen::MatrixXd emptyMat(0,0);

        WaypointSet wpt1;
        WaypointSet wpt2(wpt_vector);
        WaypointSet wpt3(wpt_vector, wpt_times);

        bool matsOk = true;
        matsOk &= emptyMat == wpt1.asMatrix();
        if(!matsOk){LOG(ERROR) << "Unitialized waypoint asMatrix() method did not work.";}
        for(int i=0; i<3; ++i){
            matsOk &= testMat.col(i) == wpt2.asMatrix().col(i);
            if(!matsOk){LOG(ERROR) << "Initialized waypoint asMatrix() method did not work.";}
            matsOk &= testMat.col(i) == wpt3.asMatrix().col(i);
            if(!matsOk){LOG(ERROR) << "Initialized waypoint and time asMatrix() method did not work.";}
        }
        TGL_CHRONO_START
        std::cout << "wpt3.asMatrix(false, false)\n" << wpt3.asMatrix(false, false) << std::endl;
        TGL_CHRONO_STOP_US
        TGL_CHRONO_START
        std::cout << "wpt3.asMatrix(false, true)\n" << wpt3.asMatrix(false, true) << std::endl;
        TGL_CHRONO_STOP_US
        TGL_CHRONO_START
        std::cout << "wpt3.asMatrix(true, false)\n" << wpt3.asMatrix(true, false) << std::endl;
        TGL_CHRONO_STOP_US
        TGL_CHRONO_START
        std::cout << "wpt3.asMatrix(true, true)\n" << wpt3.asMatrix(true, true) << std::endl;
        TGL_CHRONO_STOP_US

        return matsOk ? TGL_TEST_SUCCESS : TGL_TEST_FAILURE;
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
