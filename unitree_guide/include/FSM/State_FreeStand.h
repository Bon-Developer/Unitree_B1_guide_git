/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 로봇이 자유롭게 서 있는 상태를 정의, 목표 자세와 높이를 유지하면서 바닥에 서 있는 상태
함수
- calcOP: 목표 발 위치 계산, 현재 자세(롤, 피치, 요)와 높이 기반으로 목표 발 위치 계산
- 허용 가능한 최대/최소 롤, 피치, 요, 높이 정의
***********************************************************************/


#ifndef FREESTAND_H
#define FREESTAND_H

#include "FSM/FSMState.h"

class State_FreeStand : public FSMState{
public:
    State_FreeStand(CtrlComponents *ctrlComp);
    ~State_FreeStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    Vec3 _initVecOX;
    Vec34 _initVecXP;
    float _rowMax, _rowMin;
    float _pitchMax, _pitchMin;
    float _yawMax, _yawMin;
    float _heightMax, _heightMin;

    Vec34 _calcOP(float row, float pitch, float yaw, float height);
    void _calcCmd(Vec34 vecOP);
};

#endif  // FREESTAND_H