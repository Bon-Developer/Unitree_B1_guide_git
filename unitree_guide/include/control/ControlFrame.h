/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 로봇 제어 프레임워크를 담당하며, FSM을 기반으로 로봇의 상태를 관리하고, 로봇 제어 컴포넌트를 FSM과 연계하여 제어하는 역할
함수
- ControlFrame class: FSM 기반의 로봇 제어 흐름 관리, 로봇 제어 컴포넌트를 저장하고 FSM과 연결, run() 함수에서 FSM을 실행하여 로봇의 동작 조정
(ctrlComp: 제어 컴포넌트)
- run: FSM이 현재 상태에서 실행해야할 작업 수행(ex)현재 걷기 모드라면 다음 걸음을 어떻게 해야할지 결정)
- FSM(Finite State Machine): 상태 간 전환을 관리, run 함수가 실행될 때마다 FSM 업데이트되어 적절한 동작 수행  
***********************************************************************/


#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

class ControlFrame{
public:
	ControlFrame(CtrlComponents *ctrlComp);
	~ControlFrame(){
		delete _FSMController;
	}
	void run();
private:
	FSM* _FSMController;
	CtrlComponents *_ctrlComp;
};

#endif  //CONTROLFRAME_H