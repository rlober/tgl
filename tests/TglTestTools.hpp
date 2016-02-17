/*! \file       TglTestTools.hpp
 *  \brief      Classes and tools for performing unit tests on the TGL classes.
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

#ifndef TGL_TGLTESTTOOLS_HPP
#define TGL_TGLTESTTOOLS_HPP

#include <iostream>
#include <vector>
#include <typeinfo>

namespace tgl{
enum TglTestMessage {
    TGL_TEST_FAILURE = 0,
    TGL_TEST_SUCCESS = 1
};
static int TEST_COUNT=0; // increment for each new test case
static int TEST_SUCCESS_COUNT=0; // Number of successful tests

class TglTest{
public:
    TglTest(){TEST_COUNT++; testNumber=TEST_COUNT;}
    virtual ~TglTest()
    {
        if (testNumber==1) {
            if (TEST_COUNT == TEST_SUCCESS_COUNT) {
                std::cout << "All tests ("<< TEST_SUCCESS_COUNT <<"/"<< TEST_COUNT <<") passed!" << std::endl;
            }
            else {
                std::cout << TEST_SUCCESS_COUNT <<"/"<< TEST_COUNT << " tests failed:" << std::endl;
            }
        }
    }
    void runTest(){
        TglTestMessage tglMsg = test();
        TEST_SUCCESS_COUNT += tglMsg;
        if (tglMsg) {
            std::cout << "Test ["<< typeid(*this).name() << "] SUCCEEDED." << std::endl;
        }else{
            std::cout << "Test ["<< typeid(*this).name() << "] FAILED." << std::endl;
        }
    }
protected:
    virtual TglTestMessage test() = 0;
    int testNumber;
};

bool runAllTests(std::vector<TglTest*> testVector)
{
    for(auto test:testVector)
        test->runTest();

    for(auto test:testVector)
        delete test;

    return TEST_COUNT == TEST_SUCCESS_COUNT;
}



}


#endif //TGL_TGLTESTTOOLS_HPP
