#include "Quarternion.hpp"

Quarternion::Quarternion(float x, float y, float z, float w) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

Quarternion::~Quarternion() {}

float Quarternion::getX() {
    return this->x;
}

float Quarternion::getY() {
    return this->y;
}

float Quarternion::getZ() {
    return this->z;
}

float Quarternion::getW() {
    return this->w;
}