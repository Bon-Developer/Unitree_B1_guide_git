/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: ROS 기반의 로봇 제어 상태 중 하나인 State_move_base 정의하는 FSM 상태 클래스, 로봇이 move_base 노드로부터 이동 명령을 받아 trotting 상태를 유지하며 움직이는 역할
     cmd_vel 메시지를 구독하여 속도 명령을 받아오고, 속도가 0이면 FIXEDSTAND, 이동 중이면 TROTTING 상태 유지
     실제 로봇을 원격 제어할 경우 사용할 것으로 예상
     하지만 시뮬레이션 환경에서 cmd_vel을 사용하는 경우 활용가능
     이 코드를 활용하려면 다른 헤더 파일에 이 코드를 활성화시켜아함
***********************************************************************/


#ifdef COMPILE_WITH_MOVE_BASE

#ifndef STATE_MOVE_BASE_H
#define STATE_MOVE_BASE_H

#include "FSM/State_Trotting.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

class State_move_base : public State_Trotting{
public:
    State_move_base(CtrlComponents *ctrlComp);
    ~State_move_base(){}
    FSMStateName checkChange();
private:
    void getUserCmd();
    void initRecv();
    void twistCallback(const geometry_msgs::Twist& msg);
    ros::NodeHandle _nm;
    ros::Subscriber _cmdSub;
    double _vx, _vy;
    double _wz;
};

#endif  // STATE_MOVE_BASE_H

#endif  // COMPILE_WITH_MOVE_BASE