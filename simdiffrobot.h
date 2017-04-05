#ifndef SIMDIFFROBOT_H
#define SIMDIFFROBOT_H

#include<motionmodell.h>
#include<math.h>
#include<cstdlib>
#include<vector>
#include<fstream>

#define res 720

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
    const double R;

    DifferencialRobotSim (): deadRecPose(3), slamPose(3), dPose(3),realPose(3) ,poseCov(3,3), sumD(2,2) ,R(800), b(40)
    , jacobiPose(3,3), jacobiError(3,2),simRoute(), numOfStep(), step(){
        dsr = 2*(R - b*0.5)*M_PI/res;
        dsl = 2*(R + b*0.5)*M_PI/res;
        sumD << 0.02*dsr , 0 , 0 , 0.02*dsl; //cov matrix for odometry
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


    VectorXd DeadReckoningPose() {
        double r = rand() % 2000;
        r = 0.90 + r*0.0001;
        double sr = dsr*r;
        r = rand() % 2000;
        r = 0.905 + r*0.0001;
        double sl = dsl*r;
        ds = (sr + sl)*0.5;
        dfi = (sr - sl)/b;
        double s = (dsr + dsl)*0.5;
        double fi = (dsr - dsl)/b;
        dPose << s*cos(deadRecPose(2) + fi*0.5), s*sin(deadRecPose(2) + fi*0.5), fi;
        deadRecPose = deadRecPose + dPose;
        FiNorm(deadRecPose);
        simRoute.push_back(Vector2d(deadRecPose(0),deadRecPose(1)));
        dPose << ds*cos(realPose(2) + dfi*0.5), ds*sin(realPose(2) + dfi*0.5), dfi;
        realPose = realPose + dPose;
        realRoute.push_back(Vector2d(realPose(0), realPose(1)));
        step++;
        ds = s;
        dfi = fi;
        return deadRecPose;
    }

    MatrixXd JacobianOfMotion(VectorXd robotPose) {
        jacobiPose << 1 , 0 , -ds*sin(robotPose(2) + dfi*0.5),
                      0 , 1 , ds*cos(robotPose(2) + dfi*0.5),
                      0 , 0 , 1;
        return jacobiPose;
    }

    void JacobianOfError(VectorXd robotPose) {
        jacobiError << 0.5*cos(robotPose(2) + dfi*0.5) - 0.5*ds/b*sin(robotPose(2) + dfi*0.5), 0.5*cos(robotPose(2) + dfi*0.5) + 0.5*ds/b*sin(robotPose(2) + dfi*0.5),
                       0.5*sin(robotPose(2) + dfi*0.5) + 0.5*ds/b*cos(robotPose(2) + dfi*0.5), 0.5*sin(robotPose(2) + dfi*0.5) - 0.5*ds/b*cos(robotPose(2) + dfi*0.5),
                                                       1/b                                   ,                                 -1/b;
    }

    MatrixXd GetMotionCov(VectorXd robotPose) {
        JacobianOfError(robotPose);
        JacobianOfMotion(robotPose);
        poseCov = jacobiPose*poseCov*jacobiPose.transpose() + jacobiError*sumD*jacobiError.transpose();
        MatrixXd tempCov(2,2);
        tempCov<<poseCov(0),poseCov(1),poseCov(3),poseCov(4);
        simPoseCov.push_back(tempCov);
        return poseCov;
    }

    VectorXd GetRealPose() {
        VectorXd ret(2);
        ret<<realPose(0), realPose(1);
        return ret;
    }

    void ResetDeadReckoning() {
        deadRecPose << 0 , 0 , 0;
        realPose << 0,0,0;
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
