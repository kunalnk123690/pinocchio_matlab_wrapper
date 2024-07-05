#include "pinocchio_wrapper.hpp"


#include "mex.h"
#include <mutex>
#include <thread>
#include <Eigen/Core>


using namespace std;
mutex mtx;

// Define the pinocchio wrapper class pointer
pinWrapper *pin = new pinWrapper();

// exit function
void exitFunction()
{
    if( pin->initialized_ )
    {
        // clear globals
        pin->~pinWrapper();

        // unlock mex so MATLAB can remove it from memory
        mexUnlock();
    }
}



// entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    static char helpstr[] = 
    "USAGE:\n"
    "  [result] = pin(command, [parameters]);\n"
    "EXAMPLES:\n"
    "  pin('load', '../urdf/rrbot.urdf');\n"
    "  myxpos = mjx('getData', 'xpos');\n"
    "  mjx('setData', 'xpos', myxpos);\n"
    "COMMANDS AND PARAMETERS:\n"
    "  pin('load', file)      		                : initialize and load model file\n"
    "  pin('crba', q)      		                    : Joint space inertia matrix\n"
    "  pin('computeJointTorqueRegressor', q, v, a)  : Joint regressor matrix Y(q,dq,ddq)*PI=Tau\n"
    "  pin(exit) or pin(quit)                       : terminate\n\n";


    
    char filename[10000], command[100], fieldname[100];

    // no inputs: print help, return
    if( !nrhs )
    {
        mexPrintf(helpstr);
        return;
    }

    // get command string
    if( mxGetClassID(prhs[0])!=mxCHAR_CLASS )
    {
        mexPrintf("first argument must be command string\n");
        return;
    }

    mxGetString(prhs[0], command, 100);
    
    //---------------------------- load urdf ----------------------------//
    if( !strcmp(command, "load") )
    {
        // check for repeated initialization
        if( pin->initialized_ )
        {
            mexErrMsgTxt("already initialized\n");
            return;
        }

        // get filename
        if( nrhs<2 || mxGetClassID(prhs[1])!=mxCHAR_CLASS )
        {
            mexErrMsgTxt("second argument must be filename\n");
            return;
        }
        

        mxGetString(prhs[1], filename, 10000);
        
        // load and compile model        
        pin = new pinWrapper();
        pin->setUrdf(filename);

        cout << "Model Loaded!\n";

        // finish initialization
        mexAtExit(exitFunction);
        mexLock();
		return;

    }


    //---------------------------- getJointID ----------------------------//
    else if ( !strcmp(command, "getJointID") )
    {
        lock_guard<mutex> guard(mtx);

        if( !pin->initialized_ )
        {
            mexErrMsgTxt("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( nrhs<2 || mxGetClassID(prhs[1])!=mxCHAR_CLASS )
        {
            mexErrMsgTxt("input must be a string\n");
            return;
        }

        mxGetString(prhs[1], fieldname, 100);
        if( pin->modelPtr_->existJointName((string) fieldname) )
        {
            double JOINT_ID = (double) pin->modelPtr_->getJointId((string) fieldname);
            plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
            memcpy(mxGetPr(plhs[0]), &JOINT_ID, sizeof(double));
        }
        else
        {
            mexErrMsgTxt("invalid joint name\n");
        }

    }


    //---------------------------- getFrameId ----------------------------//
    else if ( !strcmp(command, "getFrameId") )
    {
        lock_guard<mutex> guard(mtx);

        if( !pin->initialized_ )
        {
            mexErrMsgTxt("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( nrhs<2 || mxGetClassID(prhs[1])!=mxCHAR_CLASS )
        {
            mexErrMsgTxt("input must be a string\n");
            return;
        }

        mxGetString(prhs[1], fieldname, 100);
        if( pin->modelPtr_->existFrame((string) fieldname) )
        {
            double FRAME_ID = (double) pin->modelPtr_->getFrameId((string) fieldname);
            plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
            memcpy(mxGetPr(plhs[0]), &FRAME_ID, sizeof(double));
        }
        else
        {
            mexErrMsgTxt("invalid frame name\n");
        }

    }


    //---------------------------- getBodyId ----------------------------//
    else if ( !strcmp(command, "getBodyId") )
    {
        lock_guard<mutex> guard(mtx);

        if( !pin->initialized_ )
        {
            mexErrMsgTxt("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( nrhs<2 || mxGetClassID(prhs[1])!=mxCHAR_CLASS )
        {
            mexErrMsgTxt("input must be a string\n");
            return;
        }

        mxGetString(prhs[1], fieldname, 100);
        if( pin->modelPtr_->existBodyName((string) fieldname) )
        {
            double BODY_ID = (double) pin->modelPtr_->getBodyId((string) fieldname);
            plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
            memcpy(mxGetPr(plhs[0]), &BODY_ID, sizeof(double));
        }
        else
        {
            mexErrMsgTxt("invalid body name\n");
        }

    }



    //---------------------------- crba ----------------------------//
    else if ( !strcmp(command, "crba") )
    {
        lock_guard<mutex> guard(mtx);

        if( !pin->initialized_ )
        {
            mexErrMsgTxt("Load the model first!\n");
            return;
        }        
        
        int nq = pin->nq_;
        int nv = pin->nv_;
        
        // check argument 
        if( !mxIsNumeric(prhs[1]) )
        {
            mexErrMsgTxt("input must be a double vector\n");
            return;
        }

        // check data dimensions
		const mwSize* sz = mxGetDimensions(prhs[1]);
		if( mxGetNumberOfDimensions(prhs[1])!=2 || sz[0]!=nq || sz[1]!=1)
        {
            mexErrMsgTxt("invalid q dimensions\n");
            return;
        }

        else
        {
            Eigen::VectorXd q;
            q.setZero(nq);
            memcpy(q.data(), mxGetPr(prhs[1]), sizeof(double)*nq);
            
            Eigen::MatrixXd D;
            pin->getCrba(q, D);
            
            plhs[0] = mxCreateDoubleMatrix(nv, nv, mxREAL);
            memcpy(mxGetPr(plhs[0]), D.data(), sizeof(double)*nv*nv);
        }

    }




    //---------------------------- forwardKinematics ----------------------------//
    else if ( !strcmp(command, "forwardKinematics") )
    {
        lock_guard<mutex> guard(mtx);

        if( !pin->initialized_ )
        {
            mexErrMsgTxt("Load the model first!\n");
            return;
        }        
        
        int nq = pin->nq_;
        
        // check argument 
        if( !mxIsNumeric(prhs[1]) )
        {
            mexErrMsgTxt("input must be a double vector\n");
            return;
        }

        // check data dimensions
		const mwSize* sz = mxGetDimensions(prhs[1]);
		if( mxGetNumberOfDimensions(prhs[1])!=2 || sz[0]!=nq || sz[1]!=1)
        {
            mexErrMsgTxt("invalid q dimensions\n");
            return;
        }

        else
        {
            Eigen::VectorXd q;
            q.setZero(nq);
            memcpy(q.data(), mxGetPr(prhs[1]), sizeof(double)*nq);
            
            pin->forwardKinematics(q);
            
        }

    }




    //---------------------------- computeJointTorqueRegressor ----------------------------//
    else if(!strcmp(command, "computeJointTorqueRegressor"))
    {
        lock_guard<mutex> guard(mtx);
        int nq = pin->nq_;
        int nv = pin->nv_;

        if( !pin->initialized_ )
        {
            mexErrMsgTxt("Load the model first!\n");
            return;
        }        
        
        // check argument 
        if( !mxIsNumeric(prhs[1]) || !mxIsNumeric(prhs[2]) || !mxIsNumeric(prhs[3]) )
        {
            mexErrMsgTxt("input must be a double vector\n");
            return;
        }
        
        // check data dimensions
		const mwSize* sz1 = mxGetDimensions(prhs[1]);
        const mwSize* sz2 = mxGetDimensions(prhs[2]);
        const mwSize* sz3 = mxGetDimensions(prhs[3]);
        if( mxGetNumberOfDimensions(prhs[1])!=2 || sz1[0]!=nq || sz1[1]!=1 || sz2[0]!=nv || sz2[1]!=1 || sz3[0]!=nv || sz3[1]!=1)
        {
            mexErrMsgTxt("invalid input dimensions\n");
            return;
        }
        else
        {
            Eigen::VectorXd q, v, a;
            q.setZero(nq);
            v.setZero(nv);
            a.setZero(nv);
            memcpy(q.data(), mxGetPr(prhs[1]), sizeof(double)*nq);
            memcpy(v.data(), mxGetPr(prhs[2]), sizeof(double)*nv);
            memcpy(a.data(), mxGetPr(prhs[3]), sizeof(double)*nv);
            
            Eigen::MatrixXd Y;
            pin->getJointTorqueRegressor(q, v, a, Y);
            
            plhs[0] = mxCreateDoubleMatrix(Y.rows(), Y.cols(), mxREAL);
            memcpy(mxGetPr(plhs[0]), Y.data(), sizeof(double)*Y.rows()*Y.cols());
        }
    }






    //---------------------------- terminate ----------------------------//
    else if( !strcmp(command, "exit") || !strcmp(command, "quit") )
    {
        exitFunction();
        return;
    }


    //---------------------------- undefined command ----------------------------//
    else
	{
		mexWarnMsgTxt ("undefined command string\n");
	}

}