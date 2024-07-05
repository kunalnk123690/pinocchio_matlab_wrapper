#include "pinocchio_wrapper.hpp"

pinWrapper::pinWrapper()
{
    initialized_ = false;
    modelPtr_ = new pinocchio::Model;
    dataPtr_ = NULL;
    nq_ = 0;
    nv_ = 0;
}

pinWrapper::~pinWrapper()
{
    initialized_ = false;
    modelPtr_ = NULL;
    dataPtr_ = NULL;
    nq_ = 0;
    nv_ = 0;
}

void pinWrapper::setUrdf(char filename[])
{
    pinocchio::urdf::buildModel(filename, *modelPtr_);
    dataPtr_ = new pinocchio::Data(*modelPtr_);

    initialized_ = true;
    nq_ = modelPtr_->nq;
    nv_ = modelPtr_->nv;
}



void pinWrapper::getCrba(Eigen::VectorXd& q, Eigen::MatrixXd& D)
{
    pinocchio::crba(*modelPtr_, *dataPtr_, q);
    D.setZero(nv_, nv_);
    D = dataPtr_->M.selfadjointView<Eigen::Upper>();
}

void pinWrapper::getJointTorqueRegressor(Eigen::VectorXd& q, Eigen::VectorXd& v, Eigen::VectorXd& a, Eigen::MatrixXd& Y)
{
    Y = pinocchio::computeJointTorqueRegressor(*modelPtr_, *dataPtr_, q, v, a);
}

void pinWrapper::forwardKinematics(Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(*modelPtr_, *dataPtr_, q);
}
