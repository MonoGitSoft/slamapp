#ifndef SIMDIFFROBOT_H
#define SIMDIFFROBOT_H

#include<motionmodell.h>
#include<math.h>
#include<cstdlib>
#include<vector>
#include<fstream>

#define res 360

#define DEBUG ON

#define kr 0.1;
#define kl 0.1;


inline void FiNorm(VectorXd& pose)
{
   if(pose(2) > M_PI) {
       pose(2) = -(2*M_PI - pose(2));
   }
   if(pose(2) < -M_PI) {
       pose(2) = 2*M_PI + pose(2);
   }
}

using namespace std;

class DifferencialRobotSim : public MotionModell {
    VectorXd deadRecPose;
    VectorXd realPose;
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
    double stepsl;
    double stepsr;
    VectorXd dPose;
    vector<Vector2d> simRoute;
    vector<Vector2d> realRoute;
    vector<MatrixXd> simPoseCov;
    int step;
    int numOfStep;
public:
    DifferencialRobotSim (): deadRecPose(3), slamPose(3), dPose(3),realPose(3) ,poseCov(3,3), sumD(2,2) ,R(400), b(40)
    , jacobiPose(3,3), jacobiError(3,2),simRoute(), numOfStep(), step(){
        dsr = 2*(R - b*0.5)*M_PI/res;
        dsl = 2*(R + b*0.5)*M_PI/res;
        sumD << 0.01*dsr , 0 , 0 , 0.01*dsl; //cov matrix for odometry
    }

    void SavePoses() {
        ofstream savePoses;
        savePoses.open ("pose.m");
        savePoses<<"# name: pose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<step*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : simRoute) {
            savePoses<<i<<endl;
        }
        savePoses<<"# name: cov"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<step*2<<endl
                 <<"# columns: 2"<<endl;
        for(auto i : simPoseCov) {
            savePoses<<i<<endl;
        }
        savePoses<<"# name: realpose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<step*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : realRoute) {
            savePoses<<i<<endl;
        }
        savePoses.close();
    }

    VectorXd GetRealPose(){
        VectorXd temp(3);
        temp<< cos(-2*M_PI/res*step + M_PI/2)*R, sin(-2*M_PI/res*step + M_PI/2)*R - R, -2*M_PI/res;
        return temp;

    }

    VectorXd DeadReckoningPose() {
        double r = rand() % 1000;
        r = 0.95 + r*0.0001;
        double sr = dsr*r;
        r = rand() % 1000;
        r = 0.95 + r*0.0001;
        double sl = dsl*r;
        ds = (sr + sl)*0.5;
        dfi = (sr - sl)/b;
        dPose << ds*cos(deadRecPose(2) + dfi*0.5), ds*sin(deadRecPose(2) + dfi*0.5), dfi;
        deadRecPose = deadRecPose + dPose;
        FiNorm(deadRecPose);
        simRoute.push_back(Vector2d(deadRecPose(0),deadRecPose(1)));
        realRoute.push_back(Vector2d(cos(-2*M_PI/res*step + M_PI/2)*R,sin(-2*M_PI/res*step + M_PI/2)*R - R));
        step++;
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
        MatrixXd tempCov(2,2);
        tempCov<<poseCov(0),poseCov(1),poseCov(3),poseCov(4);
        simPoseCov.push_back(tempCov);
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
