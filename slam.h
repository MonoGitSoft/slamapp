#ifndef SLAM_H
#define SLAM_H

#include <featurebase.h>
#include <motionmodell.h>

class Slam {
protected:
    MotionModell &robot;
    Enviroment &enviroment;
public:
    Slam(MotionModell &robot, Enviroment &enviroment): robot(robot), enviroment(enviroment) {
    }
    virtual void Step() = 0;
};

#endif // SLAM_H
