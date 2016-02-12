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

#ifndef TGL_TGLTYPES_H
#define TGL_TGLTYPES_H

namespace tgl
{

enum TglMessage {
    TGL_ERROR,          // 0 can use this for conditional statement
    TGL_OK,             // 1
    TGL_WARNING,        // 2
    TGL_START,          // 3
    TGL_RUNNING,        // 4
    TGL_FINISHED        // 5
};

} // End of namespace tgl
#endif //TGL_TGLTYPES_H
