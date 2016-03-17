#include "Keypoint.hpp"

Keypoint::Keypoint(int id, vpPoint point) {
    this->id = id;
    this->point = point;
}

Keypoint::Keypoint(int id, double x, double y, double z) {
    this->id = id;
    this->point.set_oX(x);
    this->point.set_oY(y);
    this->point.set_oZ(z);
}

Keypoint::~Keypoint() {}

vpPoint Keypoint::getPoint() {
    return this->point;
}

double Keypoint::getX() {
    return this->point.get_oX();
}

double Keypoint::getY() {
    return this->point.get_oY();
}

double Keypoint::getZ() {
    return this->point.get_oZ();
}

int Keypoint::getID() {
    return this->id;
}

void Keypoint::setPoint(vpPoint point) {
    this->point = point;
}

void Keypoint::setX(const double x) {
    this->point.set_oX(x);
}

void Keypoint::setY(const double y) {
    this->point.set_oY(y);
}

void Keypoint::setZ(const double z) {
    this->point.set_oZ(z);
}
