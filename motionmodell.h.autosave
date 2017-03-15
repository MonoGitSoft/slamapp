#ifndef MOTIONMODELL_H
#define MOTIONMODELL_H

#include<Eigen/Dense>

using namespace Eigen;

class MotionModell {
public:
    virtual MotionModell() = 0;
    virtual VectorXd DeadReckoningPose() = 0;
    virtual MatrixXd GetMotionCov() = 0; // return the cov matrix of deadreckoning pose
    virtual void ResetMotionCov() = 0; //reset only the cov matrix (zero matrix)
    virtual void ResetDeadReckoning() = 0; // reset pose and its cov matrix
    virtual MatrixXd JacobianOfPrediction(VectorXd robotPose) = 0; // dx dy dfi in 2dim

};

#endif // MOTIONMODELL_H
