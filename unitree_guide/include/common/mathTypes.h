/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: 수학적 연산과 데이터 구조에 필요한 벡터 및 행렬을 정의하기 위한 Eigen 라이브러리를 사용하는 C++ 헤어 파일
함수
- Vec2~6: 2,3,4,6차원의 실수 벡터를 나타냄(위치, 속도, 가속도 등을 표현할 때 사용)
- Quat: 4차원의 쿼터니안 벡터
- VecInt4: 4차원 정수 벡터
- Vec12, Vec18: 12차원, 18차원의 실수 벡터
- VecX: 동적 크기를 가진 벡터
- RotMat: 3x3 회전 행렬로, 3D 공간에서의 객체 회전을 나타냄
- HomoMat: 4x4 동차 변환 행렬로 , 3D 변환을 표현
- Mat2, Mat3, Mat6: 2x2, 3x3, 6x6 행렬로 변환
- I3, I12, I18: 3x3, 12x12, 18x18 단위 행렬 변환
- vec12ToVec34:12차원을 3x4 행렬 변환
- vec34ToVec12: 3x4 행렬을 12차원으로 다시 변환
***********************************************************************/

#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <eigen3/Eigen/Dense>

/************************/
/******** Vector ********/
/************************/
// 2x1 Vector
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// 3x1 Vector
using Vec3 = typename Eigen::Matrix<double, 3, 1>;

// 4x1 Vector
using Vec4 = typename Eigen::Matrix<double, 4, 1>;

// 6x1 Vector
using Vec6 = typename Eigen::Matrix<double, 6, 1>;

// Quaternion
using Quat = typename Eigen::Matrix<double, 4, 1>;

// 4x1 Integer Vector
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;

// 12x1 Vector
using Vec12 = typename Eigen::Matrix<double, 12, 1>;

// 18x1 Vector
using Vec18 = typename Eigen::Matrix<double, 18, 1>;

// Dynamic Length Vector
using VecX = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/******** Matrix ********/
/************************/
// Rotation Matrix
using RotMat = typename Eigen::Matrix<double, 3, 3>;

// Homogenous Matrix
using HomoMat = typename Eigen::Matrix<double, 4, 4>;

// 2x2 Matrix
using Mat2 = typename Eigen::Matrix<double, 2, 2>;

// 3x3 Matrix
using Mat3 = typename Eigen::Matrix<double, 3, 3>;

// 3x3 Identity Matrix
#define I3 Eigen::MatrixXd::Identity(3, 3)

// 3x4 Matrix, each column is a 3x1 vector
using Vec34 = typename Eigen::Matrix<double, 3, 4>;

// 6x6 Matrix
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

// 12x12 Matrix
using Mat12 = typename Eigen::Matrix<double, 12, 12>;

// 12x12 Identity Matrix
#define I12 Eigen::MatrixXd::Identity(12, 12)

// 18x18 Identity Matrix
#define I18 Eigen::MatrixXd::Identity(18, 18)

// Dynamic Size Matrix
using MatX = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

/************************/
/****** Functions *******/
/************************/
inline Vec34 vec12ToVec34(Vec12 vec12){
    Vec34 vec34;
    for(int i(0); i < 4; ++i){
        vec34.col(i) = vec12.segment(3*i, 3);
    }
    return vec34;
}

inline Vec12 vec34ToVec12(Vec34 vec34){
    Vec12 vec12;
    for(int i(0); i < 4; ++i){
        vec12.segment(3*i, 3) = vec34.col(i);
    }
    return vec12;
}

#endif  // MATHTYPES_H