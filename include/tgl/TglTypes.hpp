/*! \file       TglTypes.hpp
 *  \brief      Various enums to define specific flags, messages, etc.
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

#ifndef TGL_TGLTYPES_H
#define TGL_TGLTYPES_H

#include <ostream>

namespace tgl
{

/*! \brief An enum for providing useful return values from function executions.
 *
 *  ERROR, OK, and WARNING are mostly for functions which would return booleans for success or failure. START, RUNNING, and FINISHED are used for getting the status of the current trajectory's execution.
 */
enum TglMessage {
    TGL_ERROR,          // 0 can use this for conditional statement
    TGL_OK,             // 1
    TGL_WARNING,        // 2
    TGL_START,          // 3
    TGL_RUNNING,        // 4
    TGL_FINISHED        // 5
};

enum TglWaypointType {
    TGL_WPT_NONE,           // 0 can use this for conditional statement
    TGL_WPT_VECTOR_XD,      // 1
    TGL_WPT_LGSM_DISP,      // 2
    TGL_WPT_LGSM_QUAT,      // 3
    TGL_WPT_LGSM_WRENCH,    // 4
    TGL_WPT_KDL_FRAME       // 5
};

inline std::ostream& operator<<(std::ostream& os, const TglWaypointType& wptType)
{
    switch (wptType) {
        case TGL_WPT_NONE:
            os << "TGL_WPT_NONE";
            break;
        case TGL_WPT_VECTOR_XD:
            os << "TGL_WPT_VECTOR_XD";
            break;
        case TGL_WPT_LGSM_DISP:
            os << "TGL_WPT_LGSM_DISP";
            break;
        case TGL_WPT_LGSM_QUAT:
            os << "TGL_WPT_LGSM_QUAT";
            break;
        case TGL_WPT_LGSM_WRENCH:
            os << "TGL_WPT_LGSM_WRENCH";
            break;
        case TGL_WPT_KDL_FRAME:
            os << "TGL_WPT_KDL_FRAME";
            break;
    }
    return os;
}

} // End of namespace tgl
#endif //TGL_TGLTYPES_H
