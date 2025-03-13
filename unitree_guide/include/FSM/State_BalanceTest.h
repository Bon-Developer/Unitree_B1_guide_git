/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 로봇의 균형을 테스트하는 FSM 상태 구현, 몸의 위치, 속도, 자세 조정하는 역할 수행
함수
- State_BalanceTest: 로봇의 균형 유지에 필요한 센서 데이터, 제어 시스템, 로봇 모델 초기화
***********************************************************************/



#ifndef BALANCETEST_H
#define BALANCETEST_H

#include "FSM/FSMState.h"

class State_BalanceTest : public FSMState{
public:
    State_BalanceTest(CtrlComponents *ctrlComp);
    ~State_BalanceTest(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void calcTau();

    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    VecInt4 *_contact;

    RotMat _Rd, _RdInit;
    Vec3 _pcd, _pcdInit;
    double _kpw;
    Mat3 _Kpp, _Kdp, _Kdw;
    Vec3 _ddPcd, _dWbd;

    Vec12 _q, _tau;
    Vec3 _posBody, _velBody;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec34 _posFeet2BGlobal;
    Vec34 _forceFeetGlobal, _forceFeetBody;

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;
};

#endif  // BALANCETEST_H