/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
Release v0.1(250313)
설명: Low pass filter 헤더 파일로 LPFilter 함수는 c++로 정의되어 있음
***********************************************************************/


#ifndef LOWPASSFILTER
#define LOWPASSFILTER

class LPFilter{
public:
    LPFilter(double samplePeriod, double cutFrequency);
    ~LPFilter();
    void addValue(double newValue);
    double getValue();
    void clear();
private:
    double _weight;
    double _pastValue;
    bool _start;
};

#endif  // LOWPASSFILTER