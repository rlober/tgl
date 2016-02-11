#include "tgl/Waypoint.hpp"


using namespace tgl;

Waypoint::Waypoint()
{
}

Waypoint::Waypoint(const Eigen::VectorXd& newWpt)
{
    set(newWpt);
}

Waypoint::Waypoint(const Eigen::VectorXd& newWpt, double newWptTime)
{
    set(newWpt);
    setTime(newWptTime);
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
    set(newWpt);
    setTime(newWptTime);
}

TglMessage Waypoint::set(const Eigen::VectorXd& newWpt)
{
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

TglMessage Waypoint::setTime(double newWptTime)
{
    wptTime = newWptTime;
    return TGL_OK;
}

Eigen::VectorXd Waypoint::get()
{
    return wpt.eval();
}
double Waypoint::getTime()
{
    return wptTime;
}

int Waypoint::getDimension()
{
    return wpt.size();
}
