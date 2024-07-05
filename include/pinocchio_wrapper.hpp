#ifndef PINOCCHIO_WRAPPER_HPP
#define PINOCCHIO_WRAPPER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/regressor.hpp"

using namespace std;

class pinWrapper
{
    public:
        pinWrapper();
        ~pinWrapper();

        void getCrba(Eigen::VectorXd& q, Eigen::MatrixXd& D);
        void getJointTorqueRegressor(Eigen::VectorXd& q, Eigen::VectorXd& v, Eigen::VectorXd& a, Eigen::MatrixXd& Y);
        void forwardKinematics(Eigen::VectorXd& q);
        void setUrdf(char filename[]);

        pinocchio::Model *modelPtr_;
        pinocchio::Data *dataPtr_;

        int nq_;
        int nv_;
        bool initialized_;

};

#endif