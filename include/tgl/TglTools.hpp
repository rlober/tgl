/*! \file       TglTools.hpp
 *  \brief      Useful tools and classes when dealing with trajectories.
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

#ifndef TGL_TGLTOOLS_H
#define TGL_TGLTOOLS_H
#include <chrono>


#ifndef TGL_CHRONO_START
#define TGL_CHRONO_START TGL_CHRONO_START_LINE = __LINE__; TGL_CHRONO_START_TIME = std::chrono::high_resolution_clock::now();
#endif


#ifndef TGL_CHRONO_STOP
#define TGL_CHRONO_STOP TGL_CHRONO_STOP_TIME = std::chrono::high_resolution_clock::now(); std::cout << "|------------|\n| tgl_chrono |\n|------------|\n" << "file: "<< __FILE__ << "\nlines: "<< TGL_CHRONO_START_LINE << "-"<< __LINE__ << "\ntime_elapsed: "<< std::chrono::duration_cast<std::chrono::milliseconds>(TGL_CHRONO_STOP_TIME - TGL_CHRONO_START_TIME).count() << " ms\n\n"; TGL_CHRONO_COUNT++;
#endif

namespace tgl
{

static int TGL_CHRONO_COUNT = 0;
static int TGL_CHRONO_START_LINE = 0;
static std::chrono::high_resolution_clock::time_point TGL_CHRONO_START_TIME, TGL_CHRONO_STOP_TIME;

} // End of namespace tgl
#endif //TGL_TGLTOOLS_H
