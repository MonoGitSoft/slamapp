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

enum Commands {FWD , LEFT ,RIGHT};

class Command {
public:
    Commands command;
    double param;
    Command(Commands command, double param): command(command), param(param) { }
};


class DifferencialRobotSim : public MotionModell {
    VectorXd deadRecPose;
    VectorXd plotPose;
    VectorXd dPose;
    MatrixXd poseCov;
    MatrixXd jacobiPose;
    MatrixXd jacobiError;
    vector<Vector2d> simRoute;
    vector<Vector2d> realRoute;
    vector<MatrixXd> simPoseCov;
    MatrixXd sumD;
    double b;
    double ds;
    double dsl;
    double dsr;
    double dfi;
    vector<Command> commands;
    double odoerr;
    int commandIter;
public:
    const double R;
    VectorXd realPose;
    DifferencialRobotSim():  deadRecPose(3), dPose(3),realPose(3) ,poseCov(3,3), sumD(2,2) ,R(2000), b(40)
      , jacobiPose(3,3), jacobiError(3,2),simRoute() ,commands(),odoerr(0.04), commandIter(0), plotPose() {

        commands.push_back(Command(FWD,0));
        for(int j = 0; j < 4; j++) {
            for(int i = 0; i < 25; i++) { // 20
                commands.push_back(Command(FWD,100));
            }
            commands.push_back(Command(RIGHT,M_PI/8));
            commands.push_back(Command(RIGHT,M_PI/8));
            commands.push_back(Command(RIGHT,M_PI/8));
            commands.push_back(Command(RIGHT,M_PI/8));
        }
       /*for(int i = 0; i < 2; i++) {
           for(int j = 0; j < 1;j++) {
                commands.push_back(Command(FWD,100));
           }
           commands.push_back(Command(RIGHT,M_PI/2));
       }*/

       sumD<<0,0,0,0;
       ResetDeadReckoning();
    }

    DifferencialRobotSim(const DifferencialRobotSim& other)  = default;

    VectorXd DeadReckoningPose() {
        Execute();
        return realPose;
    }

    void SetPose(VectorXd newPose) {
        realPose = newPose;
    }

    void SavePoses() {
        ofstream savePoses;
        savePoses.open ("robotpose.m");
        savePoses<<"# name: pose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<simRoute.size()*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : simRoute) {
            savePoses<<i<<endl;
        }
        savePoses<<"# name: cov"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<simPoseCov.size()*2<<endl
                 <<"# columns: 2"<<endl;
        for(auto i : simPoseCov) {
            savePoses<<i<<endl;
        }
        savePoses<<"# name: realpose"<<endl
                 <<"# type: matrix"<<endl
                 <<"# rows: "<<realRoute.size()*2<<endl
                 <<"# columns: 1"<<endl;
        for(auto i : realRoute) {
            savePoses<<i<<endl;
        }
        savePoses.close();
    }

    void JacobianOfError(VectorXd robotPose) {
        jacobiError << 0.5*cos(robotPose(2) + dfi*0.5) - 0.5*ds/b*sin(robotPose(2) + dfi*0.5), 0.5*cos(robotPose(2) + dfi*0.5) + 0.5*ds/b*sin(robotPose(2) + dfi*0.5),
                       0.5*sin(robotPose(2) + dfi*0.5) + 0.5*ds/b*cos(robotPose(2) + dfi*0.5), 0.5*sin(robotPose(2) + dfi*0.5) - 0.5*ds/b*cos(robotPose(2) + dfi*0.5),
                                                       1/b                                   ,                                 -1/b;
    }

    MatrixXd JacobianOfMotion(VectorXd robotPose) {
        jacobiPose << 1 , 0 , -ds*sin(robotPose(2) + dfi*0.5),
                      0 , 1 , ds*cos(robotPose(2) + dfi*0.5),
                      0 , 0 , 1;
        return jacobiPose;
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
        return deadRecPose;
    }

    VectorXd GetDeadPose() {
        VectorXd ret(2);
        ret<<deadRecPose(0), deadRecPose(1);
        return ret;
    }

    void ForwardKin(double dsrParam,double dslParam) {
        dsr = dsrParam;
        dsl = dslParam;
        ds = 0.5*(dsr + dsl);
        dfi = (dsr - dsl)/b;
        VectorXd tempFi(3);
        tempFi<< 0,0,deadRecPose(2) + dfi;
        FiNorm(tempFi);
        dPose << ds*cos(tempFi(2)), ds*sin(tempFi(2)), dfi;
        deadRecPose += dPose;
        FiNorm(deadRecPose);
        simRoute.push_back(Vector2d(deadRecPose(0),deadRecPose(1)));
        sumD<<abs(dsr)*odoerr,0
                , 0,abs(dsr)*odoerr;
    }

    void SimPose() {
        int sigma = 1000*odoerr*abs(dsr);
        double r = (double)(rand() % sigma)*0.001 - odoerr*abs(dsr)*0.5;
        double sr = dsr + r;
        sigma = 1000*odoerr*abs(dsl);
        r = (double)(rand() % sigma)*0.001 - odoerr*abs(dsl)*0.5;
        double sl = dsl + r;
        double s = (sr + sl)*0.5;
        double fi = (sr - sl)/b;
        VectorXd tempFi(3);
        tempFi << 0,0,realPose(2) + fi;
        FiNorm(tempFi);
        dPose << s*cos(tempFi(2)), s*sin(tempFi(2)), fi;
        realPose += dPose;
        FiNorm(realPose);
        realRoute.push_back(Vector2d(realPose(0), realPose(1)));
    }

    void GoForward(double param) {
        ForwardKin(param,param);
        SimPose();
    }

    void TurnLeft(double param) {
        double sr = (param*b)*0.5;
        ForwardKin(sr,-sr);
        SimPose();
    }

    void TurnRight(double param) {
        double sr = -(param*b)*0.5;
        ForwardKin(sr,-sr);
        SimPose();
    }

    void Execute() {
        Command task = commands[commandIter];
        if(task.param != 0) {
            switch(task.command) {
                case FWD : GoForward(task.param); break;
                case LEFT: TurnLeft(task.param); break;
                case RIGHT: TurnRight(task.param); break;
            }
        }
        commandIter++;
        if(commandIter == commands.size()) {
            commandIter = 0;
        }
    }

    VectorXd GetFullDeadPose() {
        return deadRecPose;
    }

    void ResetDeadReckoning() {
        deadRecPose << 0 , 0 , 0;
        realPose << 0,0,0;
        poseCov << 0, 0 , 0 ,
                   0, 0 , 0 ,
                   0, 0 , 0 ;
        commandIter = 0;
    }

    void ResetMotionCov() {
        poseCov << 0, 0 , 0 ,
                   0, 0 , 0 ,
                   0, 0 , 0 ;
    }
};

#endif // SIMDIFFROBOT_H
