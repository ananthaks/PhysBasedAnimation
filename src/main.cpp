#include <iostream>
#include "Dense"
#include "utility/FileHelper.h"
#include "components/Spring.h"
#include <string>

#include <Partio.h>
#include "core/Particle.cpp"

using Eigen::MatrixXd;

using T = double;
constexpr int dim = 3;

static char *inputFile = "E:\\Git\\PhysBasedAnimation\\actual_input.txt";
static char *outputFile = "E:\\Git\\PhysBasedAnimation\\actual_output.txt";


void computeForce(std::vector<float> &result) {

    std::vector<float> inputs;

    bool isSuccess = FileHelper::readFloats(inputFile, inputs);

    if(!isSuccess) {
        return;
    }

    int count = 27;

    int numCases = inputs.size() / count;

    for(int i = 0; i < numCases; ++i) {

        float springConstant = inputs[i * count];
        float dampConstant = inputs[i * count + 1];
        float restLength = inputs[i * count + 2];

        Eigen::Vector3f point1 = Eigen::Vector3f(inputs[i * count + 3], inputs[i * count + 4], inputs[i * count + 5]);
        Eigen::Vector3f point2 = Eigen::Vector3f(inputs[i * count + 6], inputs[i * count + 7], inputs[i * count + 8]);
        Eigen::Vector3f point3 = Eigen::Vector3f(inputs[i * count + 9], inputs[i * count + 10], inputs[i * count + 11]);
        Eigen::Vector3f point4 = Eigen::Vector3f(inputs[i * count + 12], inputs[i * count + 13], inputs[i * count + 14]);

        Eigen::Vector3f velocity1 = Eigen::Vector3f(inputs[i * count + 15], inputs[i * count + 16], inputs[i * count + 17]);
        Eigen::Vector3f velocity2 = Eigen::Vector3f(inputs[i * count + 18], inputs[i * count + 19], inputs[i * count + 20]);
        Eigen::Vector3f velocity3 = Eigen::Vector3f(inputs[i * count + 21], inputs[i * count + 22], inputs[i * count + 23]);
        Eigen::Vector3f velocity4 = Eigen::Vector3f(inputs[i * count + 24], inputs[i * count + 25], inputs[i * count + 26]);

        Spring spring1 = Spring(springConstant, dampConstant, restLength);
        spring1.setEndPoints(point1, point2);
        spring1.setPointVelocities(velocity1, velocity2);
        spring1.recompute();

        Spring spring2 = Spring(springConstant, dampConstant, restLength);
        spring2.setEndPoints(point2, point3);
        spring2.setPointVelocities(velocity2, velocity3);
        spring2.recompute();

        Spring spring3 = Spring(springConstant, dampConstant, restLength);
        spring3.setEndPoints(point3, point4);
        spring3.setPointVelocities(velocity3, velocity4);
        spring3.recompute();

        Spring spring4 = Spring(springConstant, dampConstant, restLength);
        spring4.setEndPoints(point4, point1);
        spring4.setPointVelocities(velocity4, velocity1);
        spring4.recompute();

        Spring spring5 = Spring(springConstant, dampConstant, restLength);
        spring5.setEndPoints(point4, point2);
        spring5.setPointVelocities(velocity4, velocity2);
        spring5.recompute();

        // Evaluating total Spring Force at each segment's end points - Force is same in magnitude for both ends, only direction is opposite

        // Incoming is positive and outgoing is negative
        Eigen::Vector3f force1 =  -spring1.getSpringForce() + spring4.getSpringForce();
        Eigen::Vector3f force2 =  spring1.getSpringForce() - spring2.getSpringForce() + spring5.getSpringForce();
        Eigen::Vector3f force3 = -spring3.getSpringForce() + spring2.getSpringForce();
        Eigen::Vector3f force4 = -spring4.getSpringForce() - spring5.getSpringForce() + spring3.getSpringForce();

        result.push_back(force1[0]);
        result.push_back(force1[1]);
        result.push_back(force1[2]);

        result.push_back(force2[0]);
        result.push_back(force2[1]);
        result.push_back(force2[2]);

        result.push_back(force3[0]);
        result.push_back(force3[1]);
        result.push_back(force3[2]);

        result.push_back(force4[0]);
        result.push_back(force4[1]);
        result.push_back(force4[2]);

        // Evaluating total Damping Force at each segment's end points

        // Incoming is positive and outgoing is negative
        Eigen::Vector3f damp1 =  -spring1.getDampingForce() + spring4.getDampingForce();
        Eigen::Vector3f damp2 =  spring1.getDampingForce() - spring2.getDampingForce() + spring5.getDampingForce();
        Eigen::Vector3f damp3 = -spring3.getDampingForce() + spring2.getDampingForce();
        Eigen::Vector3f damp4 = -spring4.getDampingForce() - spring5.getDampingForce() + spring3.getDampingForce();

        result.push_back(damp1[0]);
        result.push_back(damp1[1]);
        result.push_back(damp1[2]);

        result.push_back(damp2[0]);
        result.push_back(damp2[1]);
        result.push_back(damp2[2]);

        result.push_back(damp3[0]);
        result.push_back(damp3[1]);
        result.push_back(damp3[2]);

        result.push_back(damp4[0]);
        result.push_back(damp4[1]);
        result.push_back(damp4[2]);

    }

    isSuccess = FileHelper::printFloats(outputFile, result);

    if(!isSuccess) {
        return;
    }

}

void printResults(std::vector<float> &results) {
    int count = results.size() / 3;
    for(int i = 0; i < count; ++i) {
        std::cout << " " << results[i * 3] << " " << results[i * 3 + 1] << " " << results[i * 3 + 2] << std::endl;
    }
}

template <class T, int dim>
void writePartio(const std::string& particleFile)
{
    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute posH, vH, mH;
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    posH = parts->addAttribute("position", Partio::VECTOR, 3);
    vH = parts->addAttribute("v", Partio::VECTOR, 3);
    for (int i=0; i<3; i++){
        int idx = parts->addParticle();
        float* m = parts->dataWrite<float>(mH, idx);
        float* p = parts->dataWrite<float>(posH, idx);
        float* v = parts->dataWrite<float>(vH, idx);
        m[0] = (T)(i + 1);
        for (int k = 0; k < 3; k++)
            p[k] = (T)(i + 1);
        for (int k = 0; k < 3; k++)
            v[k] = (T)(i + 100);
    }

    Partio::write(particleFile.c_str(), *parts);
    parts->release();
}


int main()
{

    std::string file="test.bgeo";

    //writePartio<T,dim>(file);

    //std::vector<float> results;
    //computeForce(results);
    //printResults(results);

}