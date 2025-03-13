/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
/**********************************************************************
Release v0.1(250313)
설명: 균형 제어 알고리즘을 구현하는 클래스, 균형을 유지하며 움직일 수 있도록 힘(Force) 계산하고, 다리별 최적 접촉력을 구하는 QP 기반 최적화 문제 해결
함수
- BalanceCtrl class: 로봇의 질량과 관성 정보를 바탕으로 균형 유지에 필요한 힘 계산 준비
- BalanceCtrl 생성자: 총 질량, 관성 행렬(3x3), 6x6 매트릭스로 힘-토크 변환 행렬, 균형 제어 가중치
- calF: 자코비안 행렬을 이용하여 균형을 유지하는 힘 계산
- calMatirxA: 로봇 다리 위치와 현재 회전 상태를 기반으로 힘-토크 관계 행렬(A행렬) 설정
- calVectorBd: 가속도 및 각속도 변화율을 이용하여 힘을 균형 있게 배분하기 위한 목표 벡터를 계산
- calConstraints: 접촉하는 다리 개수에 따라 QP 제약 조건 설정
- solveQP: QP를 이용한 최적 다리 힘 계산
***********************************************************************/


#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include "common/mathTypes.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/unitreeRobot.h"

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

class BalanceCtrl{
public:
    BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta);
    BalanceCtrl(QuadrupedRobot *robModel);
    Vec34 calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact);
#ifdef COMPILE_DEBUG
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG
private:
    void calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact);
    void calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM);
    void calConstraints(VecInt4 contact);
    void solveQP();

    Mat12 _G, _W, _U;
    Mat6 _S;
    Mat3 _Ib;
    Vec6 _bd;
    Vec3 _g;
    Vec3 _pcb;
    Vec12 _F, _Fprev, _g0T;
    double _mass, _alpha, _beta, _fricRatio;
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;
    Eigen::Matrix<double, 6 , 12> _A;
    Eigen::Matrix<double, 5 , 3 > _fricMat;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

#ifdef COMPILE_DEBUG
    PyPlot *_testPlot;
#endif  // COMPILE_DEBUG
};

#endif  // BALANCECTRL_H