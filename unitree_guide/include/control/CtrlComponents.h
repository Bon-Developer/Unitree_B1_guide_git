/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 로봇 제어 구성 요소 관리, 센서 데이터, 로봇 모델, 보행 패턴, 균형 제어, 상태 추정기 등 다양한 기능을 하나로 통합 지원
함수
- LowlevelCmd: lowlevel 명령 저장
- LowlevelState: lowlevel 센서 데이터 저장
- contact: 각 다리가 지면과 접촉 여부
- phase: 각 다리의 보행 위상
- *contact, *phase: 초기 상태 공중에 떠 있는 상태, 보행 패턴의 초기 위상
- sendRecv(): IOinterface를 통해 low level 명령어와 센서 데이터를 수신
- runWaveGen(): 보행 주기 및 다리 접촉 상태를 계산하고, 보행 패턴 생성
- setAllStacne, setAllSwing, setStarWave: 로봇이 다리를 어떤 상태로 유지할지 결정(모두 지면 / 모두 공중 / 보행 패턴 시작)
- geneObj: 센서 데이터를 기반으로 로봇 상태 추정, 균형 유지를 위한 제어기 생성
***********************************************************************/

#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

    void runWaveGen(){
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj(){
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);

#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        balCtrl->setPyPlot(plot);
        estimator->setPyPlot(plot);
#endif  // COMPILE_DEBUG
    }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};

#endif  // CTRLCOMPONENTS_H