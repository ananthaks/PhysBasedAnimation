//
// Created by Anantha Srinivas on 08/02/18.
//

#include "Spring.h"

Spring::Spring() : mSpringConstant(0.f), mRestLength(0.f), mCurrLength(0.f) {}

Spring::Spring(float kConstant, float restLen) : mSpringConstant(kConstant), mRestLength(restLen), mCurrLength(0.f) {}

Spring::Spring(const Spring& spring) : mSpringConstant(spring.getSpringConstant()), mRestLength(spring.getRestLength()), mCurrLength(spring.getCurrLength()) {}

float Spring::getRestLength() const {
    return mRestLength;
}

void Spring::setRestLength(float restLength) {
    mRestLength = restLength;
}

float Spring::getSpringConstant() const {
    return mSpringConstant;
}

void Spring::setSpringConstant(float springConstant) {
    mSpringConstant = springConstant;
}

void Spring::setEndPoints(const std::vector<Eigen::Vector3f> &endPoints) {
    Eigen::Vector3f point1 = Eigen::Vector3f(endPoints[0]);
    Eigen::Vector3f point2 = Eigen::Vector3f(endPoints[1]);
    mEndPoints.push_back(point1);
    mEndPoints.push_back(point2);

    mCurrLength = (point1 - point2).size();

    recompute();
}

std::vector<Eigen::Vector3f>& Spring::getEndPoints() {
    return mEndPoints;
}

float Spring::getCurrLength() const {
    return mCurrLength;
}

const Eigen::Vector3f& Spring::getSpringForce() {
    return mForce;
}

void Spring::recompute() {

}


