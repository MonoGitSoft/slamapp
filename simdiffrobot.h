#ifndef SIMDIFFROBOT_H
#define SIMDIFFROBOT_H

#include<motionmodell.h>


class DifferencialRobotSim : public MotionModell {
    VectorXd robotPose;
    MatrixXd poseCov;
public:
    DifferencialRobotSim () : robotPose(2), poseCov(2,2) {}
    DifferencialRobotSim (Index dim) : robotPose(dim), poseCov(dim,dim) {}
    VectorXd DeadReckoningPose() {
    }
};

#endif // SIMDIFFROBOT_H
