#include "wrapper.hpp"


void MexFunction::operator()(ArgumentList outputs, ArgumentList inputs) 
{
    static char helpstr[] = 
    "USAGE:\n"
    "  [result] = pin(command, [parameters]);\n"
    "EXAMPLES:\n"
    "  pin('load', '../urdf/rrbot.urdf');\n"
    "COMMANDS AND PARAMETERS:\n"
    "  pin('load', file)      		                : initialize and load model file\n"
    "  pin('crba', q)      		                    : Joint space inertia matrix\n"
    "  pin('computeJointTorqueRegressor', q, dq, ddq)  : Joint regressor matrix Y(q,dq,ddq)*PI=Tau\n"
    "  pin('computeFrameJacobian', q, 'link')       : Computes frame jacobian of frame named link in the urdf\n"
    "  pin('forwardKinematics', q)                  : Performs forward kinematics with joint configurations q \n"
    "  pin('getJointJacobianTimeVariation', q, v, 'joint') : Computes time derrivative of the jacobian\n"
    "  pin(exit) or pin(quit)                       : terminate\n\n";


    matlabPtr_ = getEngine();

    if( inputs.size()<1 )
    {
        std::cout << helpstr;
        return;
    }

    if( inputs[0].getType()!= matlab::data::ArrayType::CHAR )
    {
        displayError("Must be string input\n");
        return;
    }


    matlab::data::CharArray commandStr = inputs[0];
    if( commandStr.toAscii().compare("load") == 0 )
    {
        // check for repeated initialization
        if( initialized_ )
        {
            displayError("Model already initialized\n");
            return;
        }

        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("urdf path must be a string\n");
            return;
        }        

        matlab::data::CharArray urdfStr = inputs[1];
        setUrdf( urdfStr.toAscii() );
    }


    //---------------------------- getJointId ----------------------------//
    else if( commandStr.toAscii().compare("getJointId") == 0 )
    {

        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("input must be a string\n");
            return;
        }

        matlab::data::CharArray fieldStr = inputs[1];
        if( modelPtr_->existJointName( fieldStr.toAscii() ) )
        {
            int JOINT_ID = modelPtr_->getJointId( fieldStr.toAscii() );
            outputs[0] = factory_.createScalar( JOINT_ID );
        }
        else
        {
            displayError("invalid joint name\n");
            return;
        }

    }



    //---------------------------- getBodyId ----------------------------//
    else if( commandStr.toAscii().compare("getBodyId") == 0 )
    {

        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("input must be a string\n");
            return;
        }

        matlab::data::CharArray fieldStr = inputs[1];
        if( modelPtr_->existBodyName( fieldStr.toAscii() ) )
        {
            int BODY_ID = modelPtr_->getBodyId( fieldStr.toAscii() );
            outputs[0] = factory_.createScalar( BODY_ID );
        }
        else
        {
            displayError("invalid body name\n");
        }

    }


    //---------------------------- getFrameId ----------------------------//
    else if( commandStr.toAscii().compare("getFrameId") == 0 )
    {

        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("input must be a string\n");
            return;
        }

        matlab::data::CharArray fieldStr = inputs[1];
        if( modelPtr_->existFrame( fieldStr.toAscii() ) )
        {
            int BODY_ID = modelPtr_->getFrameId( fieldStr.toAscii() );
            outputs[0] = factory_.createScalar( BODY_ID );
        }
        else
        {
            displayError("invalid body name\n");
        }

    }


    //---------------------------- crba ----------------------------//
    else if( commandStr.toAscii().compare("crba") == 0 )
    {

        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::DOUBLE )
        {
            displayError("input must be a double vector\n");
            return;
        }

        // check argument 
        if( inputs[1].getDimensions().at(0) != modelPtr_->nq )
        {
            displayError("invalid q dimensions\n");
            return;
        }
        
        else
        {
            Eigen::VectorXd q;
            q.setZero(nq_);

            Eigen::MatrixXd D;
            D.setZero(nv_, nv_);
            double *out_;
            out_ = D.data();

            matlab::data::TypedArray<double> q_ = inputs[1];

            memcpy(q.data(), q_.release().get(), sizeof(double)*nq_);

            D = pinocchio::crba(*modelPtr_, *dataPtr_, q).selfadjointView<Eigen::Upper>();

            outputs[0] = factory_.createArray( {nv_, nv_}, out_, out_+D.size(), matlab::data::InputLayout::COLUMN_MAJOR );
            
        }

    }


    //---------------------------- computeFrameJacobian ----------------------------//
    else if( commandStr.toAscii().compare("computeFrameJacobian") == 0 )
    {

        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }
        
        // check argument 
        if( inputs.size()<3 || inputs[1].getType()!= matlab::data::ArrayType::DOUBLE || inputs[2].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("second input must be double and third input must be a string\n");
            return;
        }
        
        matlab::data::CharArray fieldStr = inputs[2];
        Eigen::VectorXd q;
        q.setZero(nq_);
        matlab::data::TypedArray<double> q_ = std::move(inputs[1]);
        Eigen::MatrixXd J;
        J.setZero(6, nv_);
        memcpy(q.data(), q_.release().get(), sizeof(double)*nq_);

        if( modelPtr_->existFrame(fieldStr.toAscii()) )
        {
            pinocchio::FrameIndex FRAME_ID = modelPtr_->getFrameId(fieldStr.toAscii());
            pinocchio::forwardKinematics(*modelPtr_, *dataPtr_, q);
            pinocchio::computeFrameJacobian(*modelPtr_, *dataPtr_, q, FRAME_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
            double *out_;
            out_ = J.data();
            outputs[0] = factory_.createArray( {6, nv_}, out_, out_+J.size(), matlab::data::InputLayout::COLUMN_MAJOR );
        }
        else
        {
            displayError("invalid input\n");
        }

    }



    //---------------------------- data.oMf.pose/framePose ----------------------------//
    else if ( commandStr.toAscii().compare("data.oMf.pose") == 0 )
    {
        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }

        // check argument 
        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("second input must be string\n");
            return;
        }

        Eigen::Vector3d p;
        Eigen::Matrix3d R;
        p.setZero();
        R.setZero();
        matlab::data::CharArray fieldStr = inputs[1];

        if( modelPtr_->existFrame(fieldStr.toAscii()) )
        {
            pinocchio::FrameIndex FRAME_ID = modelPtr_->getFrameId(fieldStr.toAscii());
            p = dataPtr_->oMf[FRAME_ID].translation();
            R = dataPtr_->oMf[FRAME_ID].rotation();
            double *out_p, *out_R;
            out_p = p.data();
            out_R = R.data();
            outputs[0] = factory_.createArray( {3, 1}, out_p, out_p+p.size(), matlab::data::InputLayout::COLUMN_MAJOR );
            outputs[1] = factory_.createArray( {3, 3}, out_R, out_R+R.size(), matlab::data::InputLayout::COLUMN_MAJOR );
        }
        else
        {
            displayError("invalid frame\n");
        }        

    }


    //---------------------------- getJointJacobianTimeVariation ----------------------------//
    else if ( commandStr.toAscii().compare("getJointJacobianTimeVariation") == 0 )
    {
        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }

        // check argument 
        if( inputs.size()<4 || inputs[1].getType()!= matlab::data::ArrayType::DOUBLE || inputs[2].getType()!= matlab::data::ArrayType::DOUBLE || inputs[3].getType()!= matlab::data::ArrayType::CHAR )
        {
            displayError("second and third input must be double and fourth input must be string\n");
            return;
        }

        Eigen::MatrixXd Jdot;
        Jdot.setZero(6, nv_);

        Eigen::VectorXd q, dq;
        q.setZero(nq_);
        dq.setZero(nv_);

        matlab::data::TypedArray<double> q_ = std::move(inputs[1]);
        matlab::data::TypedArray<double> dq_ = std::move(inputs[2]);
        memcpy(q.data(), q_.release().get(), sizeof(double)*(nq_));
        memcpy(dq.data(), dq_.release().get(), sizeof(double)*(nv_));

        matlab::data::CharArray fieldStr = inputs[3];

        if( modelPtr_->existFrame(fieldStr.toAscii()) )
        {
            pinocchio::JointIndex JOINT_ID = modelPtr_->getJointId(fieldStr.toAscii());
            pinocchio::forwardKinematics(*modelPtr_, *dataPtr_, q);
            pinocchio::computeJointJacobiansTimeVariation(*modelPtr_, *dataPtr_, q, dq);
            pinocchio::getJointJacobianTimeVariation(*modelPtr_, *dataPtr_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
            double *out_;
            out_ = Jdot.data();
            outputs[0] = factory_.createArray( {6, nv_}, out_, out_+Jdot.size(), matlab::data::InputLayout::COLUMN_MAJOR );
        }
        else
        {
            displayError("invalid joint name\n");
        }        

    }

    
    //---------------------------- forwardKinematics ----------------------------//
    else if ( commandStr.toAscii().compare("forwardKinematics") == 0 )
    {
        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }

        // check argument 
        if( inputs.size()<2 || inputs[1].getType()!= matlab::data::ArrayType::DOUBLE )
        {
            displayError("second input must be a double vector\n");
            return;
        }

        matlab::data::TypedArray<double> q_ = std::move(inputs[1]);
		if( q_.getDimensions().at(0)*q_.getDimensions().at(1) != nq_ ){
            displayError("invalid q dimensions\n");
            return;
        }

        else{
            Eigen::VectorXd q;
            q.setZero(nq_);
            memcpy(q.data(), q_.release().get(), sizeof(double)*(nq_));
            pinocchio::forwardKinematics(*modelPtr_, *dataPtr_, q);
        }


    }



    //---------------------------- computeJointTorqueRegressor ----------------------------//
    else if ( commandStr.toAscii().compare("computeJointTorqueRegressor") == 0 )
    {
        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }

        // check argument 
        if( inputs.size()<4 || inputs[1].getType()!= matlab::data::ArrayType::DOUBLE || inputs[2].getType()!= matlab::data::ArrayType::DOUBLE || inputs[3].getType()!= matlab::data::ArrayType::DOUBLE )
        {
            displayError("second input must be a double vector\n");
            return;
        }

        matlab::data::TypedArray<double> q_ = std::move(inputs[1]);
        matlab::data::TypedArray<double> dq_ = std::move(inputs[2]);
        matlab::data::TypedArray<double> ddq_ = std::move(inputs[3]);
		if( q_.getDimensions().at(0)*q_.getDimensions().at(1) != nq_ || dq_.getDimensions().at(0)*dq_.getDimensions().at(1) != nv_ || ddq_.getDimensions().at(0)*ddq_.getDimensions().at(1) != nv_ ){
            displayError("invalid input dimensions\n");
            return;
        }

        else{
            Eigen::VectorXd q, dq, ddq;
            q.setZero(nq_);
            dq.setZero(nv_);
            ddq.setZero(nv_);
            memcpy(q.data(), q_.release().get(), sizeof(double)*(nq_));
            memcpy(dq.data(), dq_.release().get(), sizeof(double)*(nv_));
            memcpy(ddq.data(), ddq_.release().get(), sizeof(double)*(nv_));
            
            Eigen::MatrixXd Y;
            Y = pinocchio::computeJointTorqueRegressor(*modelPtr_, *dataPtr_, q, dq, ddq);
            double *out_;
            out_ = Y.data();
            outputs[0] = factory_.createArray( {(unsigned long) Y.rows(), (unsigned long) Y.cols()}, out_, out_+Y.size(), matlab::data::InputLayout::COLUMN_MAJOR );

        }


    }



    //---------------------------- getDynamicParameters ----------------------------//
    else if ( commandStr.toAscii().compare("getDynamicParameters") == 0 )
    {
        if( !initialized_ )
        {
            displayError("Load the model first!\n");
            return;
        }
        
        // Number of inertial parameters
        int n = 10;

        // check argument 
        int nbodies = modelPtr_->nbodies;
        matlab::data::TypedArray<double> in = std::move(inputs[1]);
        double ID;
        memcpy(&ID, in.release().get(), sizeof(double));
        int bodyID = (int) ID;
        
        if( bodyID >= nbodies || bodyID < 0 )
        {
            displayError("Invalid BodyID\n");
            return;
        }        
        else{
            double *out_;
            out_ = modelPtr_->inertias[bodyID].toDynamicParameters().data();
            outputs[0] = factory_.createArray( {10, 1}, out_, out_+modelPtr_->inertias[bodyID].toDynamicParameters().size(), matlab::data::InputLayout::COLUMN_MAJOR );
        }

    }

    //---------------------------- terminate ----------------------------//
    else if( commandStr.toAscii().compare("exit") == 0 || commandStr.toAscii().compare("quit") == 0 )
    {
        exitFunction();
        return;
    }    
    //---------------------------- undefined command ----------------------------//
    else
	{
		displayError("undefined command string\n");
	}
    
    
}


void MexFunction::setUrdf(std::string filename)
{
    modelPtr_ = new pinocchio::Model;
    pinocchio::urdf::buildModel(filename, *modelPtr_);
    dataPtr_ = new pinocchio::Data(*modelPtr_);


    initialized_ = true;
    nq_ = modelPtr_->nq;
    nv_ = modelPtr_->nv;

    std::cout << "Model loaded! " << "nq: " << nq_ << "\n";
}



void MexFunction::exitFunction()
{
    if( initialized_ )
    {
        initialized_ = false;
        modelPtr_ = NULL;
        dataPtr_ = NULL;
        nq_ = 0;
        nv_ = 0;
        
        // unlock mex so MATLAB can remove it from memory
        mexUnlock();
    }
}


void MexFunction::displayError(std::string errorMsg)
{
    matlabPtr_->feval(u"error", 0, std::vector<matlab::data::Array>({factory_.createScalar(errorMsg)}));
}