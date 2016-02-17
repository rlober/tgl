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

/****************************************************
                   Public Functions
 ****************************************************/

Waypoint::Waypoint():
wptType(TGL_WPT_NONE)
{
}

Waypoint::Waypoint(const Eigen::VectorXd& newWpt, double newWptTime):
wptType(TGL_WPT_VECTOR_XD)
{
    this->set(newWpt, newWptTime);
}

Waypoint::Waypoint(const Eigen::Displacementd& newWpt, double newWptTime):
wptType(TGL_WPT_LGSM_DISP)
{
    this->set(newWpt, newWptTime);
}

Waypoint::Waypoint(const Eigen::Rotation3d& newWpt, double newWptTime):
wptType(TGL_WPT_LGSM_QUAT)
{
    this->set(newWpt, newWptTime);
}


// TODO: Bug here.
// Waypoint::Waypoint(const Eigen::Wrenchd& newWpt, double newWptTime)
// {
//     set(newWpt, newWptTime);
// }

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
    if(this->type()==TGL_WPT_NONE){this->setType(TGL_WPT_VECTOR_XD);}

    if (this->type()==TGL_WPT_VECTOR_XD) {
        return this->setInternalVariables(newWpt, newWptTime);
    }
    else {
        LOG(ERROR) << "You can not set a waypoint of type: " << this->type() << " with an Eigen::VectorXd.";
        return TGL_ERROR;
    }
}

TglMessage Waypoint::set(const Eigen::Displacementd& newWpt, double newWptTime)
{
    if(this->type()==TGL_WPT_NONE){this->setType(TGL_WPT_LGSM_DISP);}

    if (this->type()==TGL_WPT_LGSM_DISP) {
        Eigen::VectorXd newVectorXdWpt = newWpt.getTranslation();
        if (this->setInternalRotation(newWpt.getRotation())) {
            return this->setInternalVariables(newVectorXdWpt, newWptTime);
        }
    }
    else {
        LOG(ERROR) << "You can not set a waypoint of type: " << this->type() << " with an Eigen::VectorXd.";
        return TGL_ERROR;
    }
}

TglMessage Waypoint::set(const Eigen::Rotation3d& newWpt, double newWptTime)
{
    if(this->type()==TGL_WPT_NONE){this->setType(TGL_WPT_LGSM_QUAT);}

    if (this->type()==TGL_WPT_LGSM_QUAT) {
        setTime(newWptTime);
        return this->setInternalRotation(newWpt);
    }
    else {
        LOG(ERROR) << "You can not set a waypoint of type: " << this->type() << " with an Eigen::VectorXd.";
        return TGL_ERROR;
    }
}

TglMessage Waypoint::set(const Eigen::Wrenchd& newWpt, double newWptTime)
{
    if(this->type()==TGL_WPT_NONE){this->setType(TGL_WPT_LGSM_WRENCH);}

    if (this->type()==TGL_WPT_LGSM_WRENCH) {
        Eigen::VectorXd newVectorXdWpt = newWpt;
        return this->setInternalVariables(newVectorXdWpt, newWptTime);
    }
    else {
        LOG(ERROR) << "You can not set a waypoint of type: " << this->type() << " with an Eigen::VectorXd.";
        return TGL_ERROR;
    }
}

TglMessage Waypoint::setTime(double newWptTime)
{
    if (newWptTime >= 0.0) {
        wptTime = newWptTime;
        return TGL_OK;
    }else{
        wptTime = 0.0;
        return TGL_ERROR;
    }
}

Eigen::VectorXd Waypoint::get(bool includeTimes)
{
    if (includeTimes) {
        Eigen::VectorXd v(this->getDimension() + 1); v << this->getTime(), wpt;
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

Eigen::Rotation3d Waypoint::getRotation()
{
    if (this->hasRotation()) {
        return wptRotation;
    }
    else {
        LOG(ERROR) << "Waypoints of type: " << this->type() << " do not have a quaternion component.";
        return Eigen::Rotation3d::Identity();
    }
}

int Waypoint::getDimension()
{
    return wpt.size();
}

TglWaypointType Waypoint::type()
{
    return wptType;
}

bool Waypoint::hasRotation()
{
    if (this->type() == TGL_WPT_LGSM_DISP ||
        this->type() == TGL_WPT_LGSM_QUAT) {
        return true;
    }
    else {
        return false;
    }
}


/****************************************************
                   Private Functions
 ****************************************************/

TglMessage Waypoint::setInternalVariables(const Eigen::VectorXd& newWpt, double newWptTime)
{
    setTime(newWptTime);
    if (!this->getDimension()) {
        wpt = newWpt;
    }else if (this->getDimension()==newWpt.size()) {
        wpt = newWpt;
    }else{
        LOG(ERROR) << "The new waypoint dimension ("<< newWpt.size() <<") does not match the current waypoint dimension ("<<this->getDimension()<<"). Doing nothing.";
        return TGL_ERROR;
    }
    return TGL_OK;
}

TglMessage Waypoint::setInternalRotation(const Eigen::Rotation3d& newQuat)
{
    wptRotation = newQuat;
    return TGL_OK;
}

TglMessage Waypoint::setType(TglWaypointType newType)
{
    if (this->type()!=TGL_WPT_NONE) {
        LOG(ERROR) << "You can't change the type of waypoint (currently: "<< this->type() <<") once it has been set.";
        return TGL_ERROR;
    }
    else {
        wptType = newType;
        return TGL_OK;
    }
}
