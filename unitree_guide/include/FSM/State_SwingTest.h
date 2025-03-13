/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: FSM에서 스윙 테스트 상태를 정의
함수
- positionCtrl: 목표 위치 posGoal을 기반으로 로봇 발 위치 제어
- torqueCtrl: 목표 위치를 기반으로 발을 이동하는 토크 계산 
***********************************************************************/


#ifndef STATE_SWINGTEST_H
#define STATE_SWINGTEST_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"

class State_SwingTest : public FSMState{
public:
    State_SwingTest(CtrlComponents *ctrlComp);
    ~State_SwingTest(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void _positionCtrl();
    void _torqueCtrl();

    Vec34 _initFeetPos, _feetPos;
    Vec3  _initPos, _posGoal;
    Vec12 _targetPos;
    float _xMin, _xMax;
    float _yMin, _yMax;
    float _zMin, _zMax;
    Mat3 _Kp, _Kd;
};

#endif  // STATE_SWINGTEST_H