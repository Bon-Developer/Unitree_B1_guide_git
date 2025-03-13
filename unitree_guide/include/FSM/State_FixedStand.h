/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 로봇이 고정된 자세로 서 있는 상태를 정의, 로봇이 움직이지 않고 일정한 자세를 유지하는 상태

Release v0.2(250313)
- targetPos B1이 서 있는 상태의 각 조인트 위치 입력
***********************************************************************/

#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:

    // A1 targetPos
    // float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
    //                         0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    // B1 targetPos
    float _targetPos[12] = {0.12, 0.83, -1.56, -0.12, 0.83, -1.56, 
                            0.12, 0.83, -1.56, -0.12, 0.83, -1.56};
    float _startPos[12];
    float _duration = 1000;   //seps
    float _percent = 0;       //%
};

#endif  // FIXEDSTAND_H