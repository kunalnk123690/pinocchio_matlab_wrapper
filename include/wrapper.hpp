#include "mex.hpp"
#include "mexAdapter.hpp"
#include <iostream>
#include <Eigen/Core>
#include <string>
#include <mutex>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

using matlab::mex::ArgumentList;
using namespace std;

class MexFunction : public matlab::mex::Function
{

    public:
        void operator()(ArgumentList outputs, ArgumentList inputs);
        void setUrdf(std::string filename, bool is_floating);
        void exitFunction();
        void displayError(std::string errormsg);



    private:
        // mutex mtx_;
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr_;
        matlab::data::ArrayFactory factory_;
        bool initialized_;
        pinocchio::Model *modelPtr_;
        pinocchio::Data *dataPtr_;
        unsigned long nq_;
        unsigned long nv_;

        

};