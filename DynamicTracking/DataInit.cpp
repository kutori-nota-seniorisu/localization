#include "DataInit.h"

//����״̬���ƶ�������λ�ˣ�����һ����������Ϊ�����˵�x y thֵ
vector<vector<double>> PF_ParticleRobot(3, vector<double>(PF_Count));
//��һ�У����ι۲��Ȩ�أ��ڶ��У��������յ�Ȩ��
vector<vector<double>> PF_W(2, vector<double>(PF_Count));
//��ǩ��Ϣ
vector<string> referenceTag_EPC(referenceTagNum);
vector<vector<double>> referenceTag(referenceTagNum, vector<double>(3));
//���߸߶ȺͽǶ�
double antennaHRightError;
double antennaAlphaRightError;
double antennaHLeftError;
double antennaAlphaLeftError;
//�������߸߶�
double myVisionRightAntennaHigh;
double myVisionLeftAntennaHigh;
//�ƶ������˱�����ڵ�����
//������
vector<double> leftShadowUnreadablePointH(4);
vector<double> leftShadowUnreadablePointAlpha(4);
vector<double> leftShadowReadablePointH(4);
vector<double> leftShadowReadablePointAlpha(4);
//������
vector<double> rightShadowUnreadablePointH(4);
vector<double> rightShadowUnreadablePointAlpha(4);
vector<double> rightShadowReadablePointH(4);
vector<double> rightShadowReadablePointAlpha(4);

double robotXtAssume;
double robotYtAssume;
double robotThtAssume;

//��ʼ����ǩ��Ϣ
void EPC_Init()
{
	//�������е�EPC�����ѿ�������36��
	for (int i = 0; i < referenceTagNum; ++i)
		referenceTag_EPC[i] = "1037-9654-FFFF-FFFF-FFFF-00" + to_string(i + 10);

	//��ǩ���ݣ����ڹ�����ǩ����
	fstream fin;
	fin.open("TAG_POS.txt", ios::in);
	for (int i = 0; i < referenceTagNum; ++i)
		for (int j = 0; j < 3; ++j)
			fin >> referenceTag[i][j];
	fin.close();
}
//��ʼ������
void AntennaInit()
{
	fstream fin;
	//�������߲���
	fin.open("AtennaR_POS.txt", ios::in);
	vector<double> antennaR_POS(3);
	for (double& x : antennaR_POS)
		fin >> x;
	fin.close();

	fin.open("AtennaL_POS.txt", ios::in);
	vector<double> antennaL_POS(3);
	for (double& x : antennaL_POS)
		fin >> x;
	fin.close();

	fin.open("AGV_relative.txt", ios::in);
	vector<double> AGV_Relative(4);
	for (double& x : AGV_Relative)
		fin >> x;
	fin.close();

	//�����������ƶ�����������ϵ�µ����꣬���ѷ���relative��[3][0]�����ĸ����ǽǶ�
	vector<double> antennaR_Robot =
	{
		(antennaR_POS[2] - AGV_Relative[2]) * cos(-AGV_Relative[3]) - (antennaR_POS[0] - AGV_Relative[0]) * sin(-AGV_Relative[3]),//�������µ�x����
		(antennaR_POS[2] - AGV_Relative[2]) * sin(-AGV_Relative[3]) + (antennaR_POS[0] - AGV_Relative[0]) * cos(-AGV_Relative[3]) //�������µ�y����
	};

	vector<double> antennaL_Robot =
	{
		(antennaL_POS[2] - AGV_Relative[2]) * cos(-AGV_Relative[3]) - (antennaL_POS[0] - AGV_Relative[0]) * sin(-AGV_Relative[3]),//�������µ�x����
		(antennaL_POS[2] - AGV_Relative[2]) * sin(-AGV_Relative[3]) + (antennaL_POS[0] - AGV_Relative[0]) * cos(-AGV_Relative[3]) //�������µ�y����
	};

	//���߸߶ȺͽǶȣ����ɶ���
	double antennaHRight = sqrt(antennaR_Robot[0] * antennaR_Robot[0] + antennaR_Robot[1] * antennaR_Robot[1]) * 100;
	antennaHRightError = antennaHRight - 0;
	double antennaAlphaRight = atan(abs(antennaR_Robot[0] / antennaR_Robot[1])) + PI / 2; //45 / 180 * PI
	antennaAlphaRightError = antennaAlphaRight + 0 / 180 * PI;
	double antennaHLeft = sqrt(antennaL_Robot[0] * antennaL_Robot[0] + antennaL_Robot[1] * antennaL_Robot[1]) * 100;
	antennaHLeftError = antennaHLeft + 0;
	double antennaAlphaLeft = atan(abs(antennaL_Robot[0] / antennaL_Robot[1])) + PI / 2; //45 / 180 * PI
	antennaAlphaLeftError = antennaAlphaLeft + 0 / 180 * PI;

	//�������߸߶�
	myVisionRightAntennaHigh = antennaR_POS[1] * 100;
	myVisionLeftAntennaHigh = antennaL_POS[1] * 100;
}
//��ʼ���ƶ������˱�����ڵ�����
void ShadowInit()
{
	//������
	vector<vector<double>> leftShadowUnreadablePointCor = { {120, 20}, {-120, 20}, {-120, -120}, {120, -120} };
	for (int i = 0; i < leftShadowUnreadablePointH.size(); ++i)
		leftShadowUnreadablePointH[i] = sqrt(pow(leftShadowUnreadablePointCor[i][0], 2) + pow(leftShadowUnreadablePointCor[i][1], 2));
	for (int i = 0; i < leftShadowUnreadablePointAlpha.size(); ++i)
		leftShadowUnreadablePointAlpha[i] = atan2(leftShadowUnreadablePointCor[i][1], leftShadowUnreadablePointCor[i][0]);

	vector<vector<double>> leftShadowReadablePointCor = { {120, -20}, {-120, -20}, {-120, -120}, {120, -120} };
	for (int i = 0; i < leftShadowReadablePointH.size(); ++i)
		leftShadowReadablePointH[i] = sqrt(pow(leftShadowReadablePointCor[i][0], 2) + pow(leftShadowReadablePointCor[i][1], 2));
	for (int i = 0; i < leftShadowReadablePointAlpha.size(); ++i)
		leftShadowReadablePointAlpha[i] = atan2(leftShadowReadablePointCor[i][1], leftShadowReadablePointCor[i][0]);

	//������
	vector<vector<double>> rightShadowUnreadablePointCor = { {120, -20}, {-120, -20}, {-120, 120}, {120, 120} };
	for (int i = 0; i < rightShadowUnreadablePointH.size(); ++i)
		rightShadowUnreadablePointH[i] = sqrt(pow(rightShadowUnreadablePointCor[i][0], 2) + pow(rightShadowUnreadablePointCor[i][1], 2));
	for (int i = 0; i < rightShadowUnreadablePointAlpha.size(); ++i)
		rightShadowUnreadablePointAlpha[i] = atan2(rightShadowUnreadablePointCor[i][1], rightShadowUnreadablePointCor[i][0]);

	vector<vector<double>> rightShadowReadablePointCor = { {120, 20}, {-120, 20}, {-120, 120}, {120, 120} };
	for (int i = 0; i < rightShadowReadablePointH.size(); ++i)
		rightShadowReadablePointH[i] = sqrt(pow(rightShadowReadablePointCor[i][0], 2) + pow(rightShadowReadablePointCor[i][1], 2));
	for (int i = 0; i < rightShadowReadablePointAlpha.size(); ++i)
		rightShadowReadablePointAlpha[i] = atan2(rightShadowReadablePointCor[i][1], rightShadowReadablePointCor[i][0]);
}