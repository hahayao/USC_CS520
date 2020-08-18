#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

//define IK method
//0 - Damped Least Squares
//1 - PseudoInverse
int IKmethod = 0;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.
    
    //rotation for local and global
    vector<Mat3<real>> localRotation(fk.getNumJoints());
    vector<Mat3<real>> globalRotation(fk.getNumJoints());
    //transformation for local and global
    vector<Vec3<real>> localTransformation(fk.getNumJoints());
    vector<Vec3<real>> globalTransformation(fk.getNumJoints());
    
    real currentEuler[3];
    Mat3<real> temp1, temp2;
    for(int i = 0; i < fk.getNumJoints(); i++)
    {
        //local rotation
        for(int j = 0; j < 3; j++)
        {
            currentEuler[j] = eulerAngles[i * 3 + j];
        }
        
        temp1 = Euler2Rotation(currentEuler, fk.getJointRotateOrder(i));
        //local joint orientation rotation
        for(int j = 0; j < 3; j++)
        {
            currentEuler[j] = fk.getJointOrient(i)[j];
            
        }
        temp2 = Euler2Rotation(currentEuler, fk.getJointRotateOrder(i));
        //cout<<"TEST"<<endl;
        
        localRotation[i] = temp2 * temp1;
        
        //local transformation
        for(int j = 0; j < 3; j++)
        {
            localTransformation[i][j] = fk.getJointRestTranslation(i)[j];
        }
    }
    
    for(int i = 0; i < fk.getNumJoints(); i++)
    {
        int currentID = fk.getJointUpdateOrder(i);
        int currentParentID = fk.getJointParent(currentID);
        //root joint
        if(currentParentID == -1)
        {
            globalRotation[currentID] = localRotation[currentID];
            globalTransformation[currentID] = localTransformation[currentID];
        }
        //xParent = localTransform * xLocal
        else
        {
            //globalRotation[currentID] = globalRotation[currentParentID] * localRotation[currentID];
            multiplyAffineTransform4ds(globalRotation[currentParentID], globalTransformation[currentParentID], localRotation[currentID], localTransformation[currentID], globalRotation[currentID], globalTransformation[currentID]);
        }
    }
    
    for(int i = 0; i < numIKJoints; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            handlePositions[3 * i + j] = globalTransformation[IKJointIDs[i]][j];
        }
    }
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs.
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .
    
    int n = getFKInputDim(); // input dimension is n, euler angles
    int m = getFKOutputDim(); // output dimension is m, handle position

    // first, call trace_on to ask ADOL-C to begin recording how function f is implemented

    trace_on(adolc_tagID); // start tracking computation with ADOL-C

    vector<adouble> input(n); // define the input of the function f
    for(int i = 0; i < n; i++)
    {
        input[i] <<= 0.0;
    }

    vector<adouble> handlePos(m); // define the output of the function f

    // The computation of f goes here:
    forwardKinematicsFunction(numIKJoints, IKJointIDs, * fk, input, handlePos);

    vector<double> output(m);
    for(int i = 0; i < m; i++)
    {
        handlePos[i] >>= output[i];
    }

    // Finally, call trace_off to stop recording the function f.
    trace_off(); // ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
//  // You may find the following helpful:
    int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!
    
    int thetaNum = getFKInputDim();
    int handleNum = getFKOutputDim();
    double alpha = 0.001;
    
    vector<double> IKHandle_Pos(handleNum);
    //store value in IKHandle_Pos
    ::function(1, handleNum, thetaNum, jointEulerAngles->data(), IKHandle_Pos.data());
    //print Jacobian
    
    double jacobianMatrix[thetaNum * handleNum]; // We store the matrix in row-major order. handleNum * thetaNum
    //double * jacobianMatrixEachRow[] = { &jacobianMatrix[0], &jacobianMatrix[thetaNum] }; // pointer array where each pointer points to one row of the jacobian matrix
    
    double * jacobianMatrixEachRow[handleNum];
    for(int i = 0; i < handleNum; i++)
    {
        jacobianMatrixEachRow[i] = &jacobianMatrix[i * thetaNum];
    }
    
    ::jacobian(adolc_tagID, handleNum, thetaNum, jointEulerAngles->data(), jacobianMatrixEachRow); // each row is the gradient of one output component of the function
    
    //using formula for J, Ax = B
    //Matrix A
    Eigen::MatrixXd J(handleNum, thetaNum); // define a 3x3 Eigen column-major matrix
    // assign values to J
    for(int rowID = 0; rowID < handleNum; rowID++)
      for(int colID = 0; colID < thetaNum; colID++)
        J(rowID,colID) = jacobianMatrix[thetaNum * rowID + colID];
    
    //Damped Least Square
    if(IKmethod == 0)
    {
        Eigen::MatrixXd Ident(thetaNum, thetaNum);
        Ident.setIdentity();
        
        Eigen::MatrixXd A(thetaNum, thetaNum); // define a 3x3 Eigen column-major matrix
        A = J.transpose() * J + alpha * Ident;
        
        //Matrix B
        //delta B = delta Position
        Eigen::VectorXd deltaB(handleNum); // define a 3x1 Eigen column vector
        for(int i = 0; i < handleNum; i++)
            deltaB(i) = targetHandlePositions->data()[i] - IKHandle_Pos.data()[i];
        
        Eigen::VectorXd B(thetaNum);
        B = J.transpose() * deltaB;
        
        Eigen::VectorXd x = A.ldlt().solve(B);
        
        //jointEulerAngles + delta(theta)
        for(int i = 0; i < numJoints; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                jointEulerAngles[i][j] += x(3 * i + j);
            }
        }
    }
    //PseudoInverse
    else
    {
        //A * A-1 = I
        //Matrix A
        Eigen::MatrixXd A(handleNum, handleNum);
        A = J * J.transpose();
        
        Eigen::MatrixXd Ident(handleNum, handleNum);
        Ident.setIdentity();
        
        Eigen::MatrixXd AInverse(handleNum, handleNum);
        AInverse = A.ldlt().solve(Ident);
        
        //JDagger
        Eigen::MatrixXd JDagger(thetaNum, handleNum);
        JDagger = J.transpose() * AInverse;
        
        //delta B = delta Position
        Eigen::VectorXd deltaB(handleNum); // define a 3x1 Eigen column vector
        for(int i = 0; i < handleNum; i++)
            deltaB(i) = targetHandlePositions->data()[i] - IKHandle_Pos.data()[i];
        
        Eigen::VectorXd x = JDagger * deltaB;
        
        //jointEulerAngles + delta(theta)
        for(int i = 0; i < numJoints; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                jointEulerAngles[i][j] += x(3 * i + j);
            }
        }
    }
    
    
    
}

