#ifndef SIMDIFFROBOT_H
#define SIMDIFFROBOT_H

#include<motionmodell.h>
#include<math.h>
#include<cstdlib>

#define res 100

#define kr 0.1;
#define kl 0.1;


/*inline void FiNorm(VectorXd& pose)
{
   if(pose(2) > M_PI) {
       pose(2) = -(2*M_PI - pose(2));
   }
   if(pose(2) < -M_PI) {
       pose(2) = 2*M_PI + pose(2);
   }
}*/

class DifferencialRobotSim : public MotionModell {
    VectorXd deadRecPose;
    VectorXd slamPose;
    MatrixXd poseCov;
    MatrixXd jacobiPose;
    MatrixXd jacobiError;
    MatrixXd sumD;
    double R;
    double b;
    double ds;
    double dsl;
    double dsr;
    double dfi;
    VectorXd dPose;
public:
    DifferencialRobotSim (): deadRecPose(3), slamPose(3), dPose(3) ,poseCov(3,3), sumD(2,2) ,R(500), b(40)
    , jacobiPose(3,3), jacobiError(3,2){
        dsr = 2*(R - b*0.5)*M_PI/res;
        dsl = 2*(R + b*0.5)*M_PI/res*0.99;
        ds = (dsr + dsl)*0.5;
        dfi = (dsr - dsl)/b;
        dPose << 0,0,dfi;
        sumD << 0.05*dsr , 0 , 0 , 0.05*dsl;
    }

    VectorXd DeadReckoningPose() {
        dPose[0] = ds*cos(deadRecPose(2) + dfi*0.5);
        dPose[1] = ds*sin(deadRecPose(2) + dfi*0.5);
        deadRecPose = deadRecPose + dPose;
        return deadRecPose;
    }

    MatrixXd JacobianOfPrediction(VectorXd robotPose) {
        jacobiPose << 1 , 0 , -ds*sin(robotPose(2) + dfi*0.5),
                      0 , 1 , ds*cos(robotPose(2) + dfi*0.5),
                      0 , 0 , 1;
        return jacobiPose;
    }

    void JacobianOfError() {
        jacobiError << 0.5*cos(slamPose(2) + dfi*0.5) - 0.5*ds/b*sin(slamPose(2) + dfi*0.5), 0.5*cos(slamPose(2) + dfi*0.5) + 0.5*ds/b*sin(slamPose(2) + dfi*0.5),
                       0.5*sin(slamPose(2) + dfi*0.5) + 0.5*ds/b*cos(slamPose(2) + dfi*0.5), 0.5*sin(slamPose(2) + dfi*0.5) - 0.5*ds/b*cos(slamPose(2) + dfi*0.5),
                                                       1/b                                   ,                                 -1/b;
    }

    MatrixXd GetMotionCov(VectorXd robotPose) {
        JacobianOfError();
        JacobianOfPrediction(robotPose);
        poseCov = jacobiPose*poseCov*jacobiPose.transpose() + jacobiError*sumD*jacobiError.transpose();
        return poseCov;
    }

    void ResetDeadReckoning() {
        deadRecPose << 0 , 0 , 0;
        poseCov << 0, 0 , 0 ,
                   0, 0 , 0 ,
                   0, 0 , 0 ;
    }

    void ResetMotionCov() {
        poseCov << 0, 0 , 0 ,
                   0, 0 , 0 ,
                   0, 0 , 0 ;
    }
};

#endif // SIMDIFFROBOT_H
