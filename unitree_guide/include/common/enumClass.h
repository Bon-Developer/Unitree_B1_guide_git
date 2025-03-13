/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 로봇 제어 플랫폼, 로봇 유형, 사용자 명령, 프레임 유형, 웨이브 상태, FSM 모드, 상태명 분류
RobotType에 B1 추가asdasdasdasdasadasdasdasdasdasd
***********************************************************************/



#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

//250312 B1 추가
enum class RobotType{
    A1,
    Go1,
    B1 //추가
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // trotting
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    L1_X,       // balanceTest
    L1_A,       // swingTest
    L1_Y        // stepTest
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCETEST,
    SWINGTEST,
    STEPTEST
};

#endif  // ENUMCLASS_H