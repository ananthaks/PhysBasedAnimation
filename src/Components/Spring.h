//
// Created by Anantha Srinivas on 08/02/18.
//

#pragma once

#include <vector>
#include "Dense"

class Spring {

private:

    float mSpringConstant;
    float mRestLength;
    float mCurrLength;

    std::vector<Eigen::Vector3f> mEndPoints;

    Eigen::Vector3f mForce;

public:

    Spring();

    Spring(float kConstant, float restLen);

    Spring(const Spring& spring);

    float getRestLength() const;

    void setRestLength(float restLength);

    float getSpringConstant() const;

    void setSpringConstant(float springConstant);

    void setEndPoints(const std::vector<Eigen::Vector3f> &endPoints);

    std::vector<Eigen::Vector3f>& getEndPoints();

    float getCurrLength() const;

    const Eigen::Vector3f& getSpringForce();

private:
     void recompute();


};

