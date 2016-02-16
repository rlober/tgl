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
// STL includes
#include <chrono>


// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Lgsm>

/*! \brief Chronometer START trigger.
 *
 *  To clock a segment of code simply add this before the segment. Note: It must be followed by a STOP trigger before a new segment can be clocked. The chronometer will use the highest fidelity possible. See [std::chrono::high_resolution_clock](http://en.cppreference.com/w/cpp/chrono/high_resolution_clock) for more details.
 */
#ifndef TGL_CHRONO_START
#define TGL_CHRONO_START TGL_CHRONO_START_LINE = __LINE__; TGL_CHRONO_START_TIME = std::chrono::high_resolution_clock::now();
#endif

/*! \brief Chronometer STOP trigger (ms).
 *
 *  Add this macro to stop the code clocking. The output is written to the console and gives the file path, lines clocked and the total duration between the START and STOP triggers in milliseconds. See the other STOP triggers for different clock periods.
 */
#ifndef TGL_CHRONO_STOP
#define TGL_CHRONO_STOP TGL_CHRONO_STOP_TIME = std::chrono::high_resolution_clock::now(); std::cout << "|------------|\n| tgl_chrono |\n|------------|\n" << "file: "<< __FILE__ << "\nlines: "<< TGL_CHRONO_START_LINE << "-"<< __LINE__ << "\ntime_elapsed: "<< std::chrono::duration_cast<std::chrono::milliseconds>(TGL_CHRONO_STOP_TIME - TGL_CHRONO_START_TIME).count() << " ms\n\n"; TGL_CHRONO_COUNT++;
#endif

/*! \brief Chronometer STOP trigger (us).
 *
 *  Add this macro to stop the code clocking. The output is written to the console and gives the file path, lines clocked and the total duration between the START and STOP triggers in microseconds. See the other STOP triggers for different clock periods.
 */
#ifndef TGL_CHRONO_STOP_US
#define TGL_CHRONO_STOP_US TGL_CHRONO_STOP_TIME = std::chrono::high_resolution_clock::now(); std::cout << "|------------|\n| tgl_chrono |\n|------------|\n" << "file: "<< __FILE__ << "\nlines: "<< TGL_CHRONO_START_LINE << "-"<< __LINE__ << "\ntime_elapsed: "<< std::chrono::duration_cast<std::chrono::microseconds>(TGL_CHRONO_STOP_TIME - TGL_CHRONO_START_TIME).count() << " us\n\n"; TGL_CHRONO_COUNT++;
#endif

/*! \brief Chronometer STOP trigger (ns).
 *
 *  Add this macro to stop the code clocking. The output is written to the console and gives the file path, lines clocked and the total duration between the START and STOP triggers in nanoseconds. See the other STOP triggers for different clock periods.
 */
#ifndef TGL_CHRONO_STOP_NS
#define TGL_CHRONO_STOP_NS TGL_CHRONO_STOP_TIME = std::chrono::high_resolution_clock::now(); std::cout << "|------------|\n| tgl_chrono |\n|------------|\n" << "file: "<< __FILE__ << "\nlines: "<< TGL_CHRONO_START_LINE << "-"<< __LINE__ << "\ntime_elapsed: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(TGL_CHRONO_STOP_TIME - TGL_CHRONO_START_TIME).count() << " ns\n\n"; TGL_CHRONO_COUNT++;
#endif

namespace tgl
{

static int TGL_CHRONO_COUNT = 0;                                                /*!< A counter used to measure the number of segments clocked. TODO: use this to provide an ID for the clock results which can be pulled from a log written to file. */
static int TGL_CHRONO_START_LINE = 0;                                           /*!< The line where the START trigger was called. */
static std::chrono::high_resolution_clock::time_point TGL_CHRONO_START_TIME;    /*!< System time at the START trigger. */
static std::chrono::high_resolution_clock::time_point TGL_CHRONO_STOP_TIME;     /*!< System time at the START trigger. */

/*! \class TglTools
 *  \brief A tool class which provides some useful static conversion functions.
 *
 *  This is mostly a "faux" class full of static functions which make life just a little easier for the users. Here you will find a bunch of conversion functions to/from Eigen, KDL, and EigenLgsm.
 */
class TglTools {
public:
    TglTools ();
    ~TglTools ();

    /*! Convert an Eigen::VectorXd to an Eigen::Displacementd (Lgsm)
     *  \param inputVector an Vector of values which must be of dimension 7 and correspond to the following:
        \f[
            \begin{array}{cc}
            pos =& \begin{dcases} \begin{bmatrix} x \\ y \\ z \end{bmatrix}\end{dcases} \\
            quat =& \begin{dcases}\begin{bmatrix} qw \\ qx \\ qy \\ qz \end{bmatrix}\end{dcases}
            \end{array}
        \f]
     *  \return An Dispd object filled with the inputVector values.
     */
    static Eigen::Displacementd eigenVectorXdToDisplacementd(const Eigen::VectorXd& inputVector);

    /*! Convert an Eigen::VectorXd to an Eigen::Twistd (Lgsm)
     *  \param inputVector an Vector of values which must be of dimension 6 and correspond to the following:
        \f[
            \begin{array}{cc}
            linear =& \begin{dcases} \begin{bmatrix} x \\ y \\ z \end{bmatrix}\end{dcases} \\
            angular =& \begin{dcases}\begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}\end{dcases}
            \end{array}
        \f]
     *  \return An Twistd object filled with the inputVector values.
     */
    static Eigen::Twistd eigenVectorXdToTwistd(const Eigen::VectorXd& inputVector);

    /*! Convert an Eigen::VectorXd to an Eigen::Rotation3d (Quaternion) (Lgsm)
     *  \param inputVector an Vector of values which must be of dimension 4 and correspond to the following:
        \f[
            \begin{bmatrix} qw \\ qx \\ qy \\ qz \end{bmatrix}
        \f]
     *  \return An Dispd object filled with the inputVector values.
     */
    static Eigen::Rotation3d eigenVectorXdToRotation3d(const Eigen::VectorXd& inputVector);

    /*! Convert an Eigen::Displacementd to an Eigen::Displacementd (Lgsm)
     *  \param inputDisplacementd an Eigen Lgsm Displacementd object reference
     *  \return A VectorXd object filled with the input values.
     */
    static Eigen::VectorXd eigenDisplacementdToVectorXd(const Eigen::Displacementd& inputDisplacementd);

    /*! Convert an Eigen::Twistd to an Eigen::Twistd (Lgsm)
     *  \param inputTwistd an Eigen Lgsm Twistd object reference
     *  \return A VectorXd object filled with the input values.
     */
    static Eigen::VectorXd eigenTwistdToVectorXd(const Eigen::Twistd& inputTwistd);

    /*! Convert an Eigen::Rotation3d to an Eigen::Rotation3d (Quaternion) (Lgsm)
     *  \param inputRotation3d an Eigen Lgsm Rotation3d object reference
     *  \return A VectorXd object filled with the input values.
     */
    static Eigen::VectorXd eigenRotation3dToVectorXd(const Eigen::Rotation3d& inputRotation3d);

};

} // End of namespace tgl
#endif //TGL_TGLTOOLS_H
