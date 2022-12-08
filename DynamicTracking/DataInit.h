#ifndef _DATA_INIT_H
#define _DATA_INIT_H

#include "DataProcess.h"

//����״̬���ƶ�������λ�ˣ�����һ����������Ϊ�����˵�x y thֵ
extern vector<vector<double>> PF_ParticleRobot;
//��һ�У����ι۲��Ȩ�أ��ڶ��У��������յ�Ȩ��
extern vector<vector<double>> PF_W;
//��ǩ���ݣ����ڹ�����ǩ����
extern vector<string> referenceTag_EPC;
extern vector<vector<double>> referenceTag;
//���߸߶ȺͽǶ�
extern double antennaHRightError;
extern double antennaAlphaRightError;
extern double antennaHLeftError;
extern double antennaAlphaLeftError;
//�������߸߶�
extern double myVisionRightAntennaHigh;
extern double myVisionLeftAntennaHigh;
//�ƶ������˱�����ڵ�����
//������
extern vector<double> leftShadowUnreadablePointH;
extern vector<double> leftShadowUnreadablePointAlpha;
extern vector<double> leftShadowReadablePointH;
extern vector<double> leftShadowReadablePointAlpha;
//������
extern vector<double> rightShadowUnreadablePointH;
extern vector<double> rightShadowUnreadablePointAlpha;
extern vector<double> rightShadowReadablePointH;
extern vector<double> rightShadowReadablePointAlpha;

extern double robotXtAssume;
extern double robotYtAssume;
extern double robotThtAssume;

//��ʼ����ǩ��Ϣ
void EPC_Init();
//��ʼ������
void AntennaInit();
//��ʼ���ƶ������˱�����ڵ�����
void ShadowInit();

#endif // !_DATA_INIT_H

