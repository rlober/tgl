/*! \file       Waypoint.cpp
 *  \brief      A class for defining waypoints which consist of a coordinate vector and a time step.
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

#include "tgl/Waypoint.hpp"


using namespace tgl;

Waypoint::Waypoint()
{
}

Waypoint::Waypoint(const Eigen::VectorXd& newWpt, double newWptTime)
{
    set(newWpt, newWptTime);
}

Waypoint::Waypoint(const Eigen::Displacementd& newWpt, double newWptTime)
{
    set(newWpt, newWptTime);
}

Waypoint::Waypoint(const Eigen::Rotation3d& newWpt, double newWptTime)
{
    set(newWpt, newWptTime);
}

Waypoint& Waypoint::operator=(Waypoint other)
{
    other.set(this->get(), this->getTime());
    return *this;
}

bool Waypoint::operator==(Waypoint& other)
{
    bool retBool=true;
    retBool &= this->get() == other.get();
    return retBool;
}

Waypoint Waypoint::operator+(Waypoint& other)
{
    if (this->getDimension() == other.getDimension()) {
        return  Waypoint((this->get() + other.get()));
    }
    else {
        LOG(ERROR) << "Waypoint dimensions do not match: "<<this->getDimension()<<" ~= "<<other.getDimension()<<".";
        return Waypoint();
    }
}

Waypoint Waypoint::operator-(Waypoint& other)
{
    if (this->getDimension() == other.getDimension()) {
        return  Waypoint((this->get() - other.get()));
    }
    else {
        LOG(ERROR) << "Waypoint dimensions do not match: "<<this->getDimension()<<" ~= "<<other.getDimension()<<".";
        return Waypoint();
    }
}

Waypoint Waypoint::operator*(double scalar)
{
    return  Waypoint(this->get() * scalar);
}

Waypoint Waypoint::operator/(double scalar)
{
    if (scalar>=0.0) {
        return  Waypoint(this->get() / scalar);
    }
    else {
        LOG(ERROR) << "Divide by zero.";
        return Waypoint();
    }
}

// Waypoint Waypoint::operator+=(Waypoint& other)
// {
//
// }


// Waypoint Waypoint::operator-=(Waypoint& other)
// {
//
// }

// Waypoint Waypoint::operator*=(double scalar)
// {
//
// }


// Waypoint Waypoint::operator/=(double scalar)
// {
//
// }

TglMessage Waypoint::set(const Eigen::VectorXd& newWpt, double newWptTime)
{
    if (newWptTime!=TGL_WAYPOINT_TIME_NOT_SPECIFIED && newWptTime >= 0.0) {
        setTime(newWptTime);
    }

    if (!getDimension()) {
        wpt = newWpt;
    }else if (getDimension()==newWpt.size()) {
        wpt = newWpt;
    }else{
        LOG(ERROR) << "The new waypoint dimension ("<< newWpt.size() <<") does not match the current waypoint dimension ("<<getDimension()<<"). Doing nothing.";
        return TGL_ERROR;
    }
    return TGL_OK;
}

TglMessage Waypoint::set(const Eigen::Displacementd& newWpt, double newWptTime)
{
    //TODO: implement
    return TGL_OK;
}

TglMessage Waypoint::set(const Eigen::Rotation3d& newWpt, double newWptTime)
{
    //TODO: implement
    return TGL_OK;
}

TglMessage Waypoint::set(const Eigen::Wrenchd& newWpt, double newWptTime)
{
    //TODO: implement
    return TGL_OK;
}

TglMessage Waypoint::setTime(double newWptTime)
{
    wptTime = newWptTime;
    return TGL_OK;
}

Eigen::VectorXd Waypoint::get(bool includeTimes)
{
    if (includeTimes) {
        Eigen::VectorXd v(getDimension() + 1); v << getTime(), wpt;
        return v;
    }
    else {
        return wpt;
    }
}
double Waypoint::getTime()
{
    return wptTime;
}

int Waypoint::getDimension()
{
    return wpt.size();
}
