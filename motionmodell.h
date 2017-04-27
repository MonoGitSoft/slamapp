#ifndef MOTIONMODELL_H
#define MOTIONMODELL_H

#include<Eigen/Dense>

using namespace Eigen;

class MotionModell {
public:
    MotionModell() {}
    virtual VectorXd DeadReckoningPose() = 0; // not for deadreckoning is for the other metod
    virtual MatrixXd GetMotionCov(VectorXd robotPose) = 0; // return the cov matrix of deadreckoning pose
    virtual void ResetMotionCov() = 0; //reset only the cov matrix (zero matrix)
    virtual void ResetDeadReckoning() = 0; // reset pose and its cov matrix
    virtual MatrixXd JacobianOfMotion(VectorXd robotPose) = 0; // dx dy dfi in 2dim
    virtual void SetPose(VectorXd newPose) = 0;

};

#endif // MOTIONMODELL_H
