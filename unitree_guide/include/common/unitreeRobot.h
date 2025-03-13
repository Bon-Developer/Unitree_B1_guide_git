/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: Unitree의 4족 로봇을 위한 클래스 정의
함수
- QuadrupedRobot class: 로봇의 다리 운동과 위치 추적을 위한 클래스 정의
- getX, getVecXP: 로봇의 현재 위치 및 자세를 LowlevelState 객체를 통해 반환
- getQ, getQd, getTau: 역기구학 계산을 수행하며, 주어진 발의 위치나 속도에 대응하여 관절 각도, 각속도 반환. 관절 각도를 이용하여 발의 힘 반환
- getFootPosition: 로봇의 다리 위치 계산
- getFootVelocity: 로봇의 다리 속도 계산
- getFeet2BPositions: 로봇의 다리 위치를 몸체 좌표계 기준으로 반환
- getFeet2BVelocities: 로봇의 다리 속도를 몸체 좌표계 기준으로 반환
- getJaco: 다리에 대한 자코비안 행렬 반환
- getRobVelLimitX, Y, Yaw: 로봇의 이동 및 회전 속도 제한 설정
- getFeetPosIdeal, getRobMass, getPcb, getRobInertial: 로봇의 이상적인 발 위치, 질량, 중심 위치, 관성 행렬 반환
- A1Robot, Go1Robot class: 특정 로봇 모델에 맞게 구현된 클래스 

Release v0.2(250313)
- B1Robot class 추가, B1Robot 함수 정의 필요
***********************************************************************/

#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

class QuadrupedRobot{
public:
    QuadrupedRobot(){};
    ~QuadrupedRobot(){}

    Vec3 getX(LowlevelState &state);
    Vec34 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    // Forward Kinematics
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    Vec3 getFootVelocity(LowlevelState &state, int id);
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);

    Mat3 getJaco(LowlevelState &state, int legID);
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}

protected:
    QuadrupedLeg* _Legs[4];
    Vec2 _robVelLimitX;
    Vec2 _robVelLimitY;
    Vec2 _robVelLimitYaw;
    Vec34 _feetPosNormalStand;
    double _mass;
    Vec3 _pcb;
    Mat3 _Ib;
};

class A1Robot : public QuadrupedRobot{
public:
    A1Robot();
    ~A1Robot(){}
};

class Go1Robot : public QuadrupedRobot{
public:
    Go1Robot();
    ~Go1Robot(){};
};

class B1Robot : public QuadrupedRobot{
public:
    B1Robot();
    ~B1Robot(){};
};

#endif  // UNITREEROBOT_H