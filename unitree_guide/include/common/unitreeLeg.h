/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: Unitree 로봇의 다리 구현을 위한 클래스 정의, 다리 부분의 운동학 및 동작 제어
함수
- QuadrupedLeg class: 다리 식별자, 각 링크의 길이, 엉덩이부터 몸체까지의 위치 벡터를 매개변수로 받음
- calcPEe2H, calcPEe2B: 주어진 관절 각도를 사용하여 엔드이펙터의 위치를 엉덩이 좌표 또는 몸체 좌표로 계산
- calcVEe: 엔드이펙터의 속도 계산
- calcQ, calcQd: 주어진 위치 또는 속도에서 관절 각도 또는 각속도 계산
- calcTau: 관절 각도와 힘을 사용하여 토크 계산
- caclJaco: 관절 각도에 대한 엔드이펙터의 자코비안 행렬 계산
- q1_ik, q2_ik, q3_ik: 각각 역기구학 계산
- A1Leg, Go1Leg class: QuadrupedLeg 클래스를 상속받는 로봇 모델 다리 구현, 특정 로봇 모델의 다리 길이와 초기 설정을 생성자로 저장

Release v0.2(250313)
- unitree_ros_master의 robot description 기준으로 B1 class도 정의
- A1, Go1의 const.xacro 확인 결과 각 수치는 kinetic value의 thigh_offset / thigh_length / calf_length로 정의된 것으로 확인
- B1도 동일하게 const.xacro에서 kinetic value로 정의
***********************************************************************/

#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg(){}
    Vec3 calcPEe2H(Vec3 q);
    Vec3 calcPEe2B(Vec3 q);
    Vec3 calcVEe(Vec3 q, Vec3 qd);
    Vec3 calcQ(Vec3 pEe, FrameType frame);
    Vec3 calcQd(Vec3 q, Vec3 vEe);
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
    Vec3 calcTau(Vec3 q, Vec3 force);
    Mat3 calcJaco(Vec3 q);
    Vec3 getHip2B(){return _pHip2B;}
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;
    const Vec3 _pHip2B;
};

class A1Leg : public QuadrupedLeg{
public:
    A1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
    ~A1Leg(){}
};

class Go1Leg : public QuadrupedLeg{
public:
    Go1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B){}
    ~Go1Leg(){}
};

class B1Leg : public QuadrupedLeg{
public:
    B1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.12675, 0.35, 0.35, pHip2B){}
    ~B1Leg(){}
};

#endif  // UNITREELEG_H