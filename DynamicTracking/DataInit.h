#ifndef _DATA_INIT_H
#define _DATA_INIT_H

#include "DataProcess.h"

//粒子状态（移动机器人位姿），第一、二、三行为机器人的x y th值
extern vector<vector<double>> PF_ParticleRobot;
//第一行：本次观测得权重；第二行：本次最终的权重
extern vector<vector<double>> PF_W;
//标签数据，用于构建标签阵列
extern vector<string> referenceTag_EPC;
extern vector<vector<double>> referenceTag;
//天线高度和角度
extern double antennaHRightError;
extern double antennaAlphaRightError;
extern double antennaHLeftError;
extern double antennaAlphaLeftError;
//左右天线高度
extern double myVisionRightAntennaHigh;
extern double myVisionLeftAntennaHigh;
//移动机器人本体的遮挡区域
//左天线
extern vector<double> leftShadowUnreadablePointH;
extern vector<double> leftShadowUnreadablePointAlpha;
extern vector<double> leftShadowReadablePointH;
extern vector<double> leftShadowReadablePointAlpha;
//右天线
extern vector<double> rightShadowUnreadablePointH;
extern vector<double> rightShadowUnreadablePointAlpha;
extern vector<double> rightShadowReadablePointH;
extern vector<double> rightShadowReadablePointAlpha;

extern double robotXtAssume;
extern double robotYtAssume;
extern double robotThtAssume;

//初始化标签信息
void EPC_Init();
//初始化天线
void AntennaInit();
//初始化移动机器人本体的遮挡区域
void ShadowInit();

#endif // !_DATA_INIT_H

