#include "DataInit.h"

//粒子状态（移动机器人位姿），第一、二、三行为机器人的x y th值
vector<vector<double>> PF_ParticleRobot(3, vector<double>(PF_Count));
//第一行：本次观测得权重；第二行：本次最终的权重
vector<vector<double>> PF_W(2, vector<double>(PF_Count));
//标签信息
vector<string> referenceTag_EPC(referenceTagNum);
vector<vector<double>> referenceTag(referenceTagNum, vector<double>(3));
//天线高度和角度
double antennaHRightError;
double antennaAlphaRightError;
double antennaHLeftError;
double antennaAlphaLeftError;
//左右天线高度
double myVisionRightAntennaHigh;
double myVisionLeftAntennaHigh;
//移动机器人本体的遮挡区域
//左天线
vector<double> leftShadowUnreadablePointH(4);
vector<double> leftShadowUnreadablePointAlpha(4);
vector<double> leftShadowReadablePointH(4);
vector<double> leftShadowReadablePointAlpha(4);
//右天线
vector<double> rightShadowUnreadablePointH(4);
vector<double> rightShadowUnreadablePointAlpha(4);
vector<double> rightShadowReadablePointH(4);
vector<double> rightShadowReadablePointAlpha(4);

double robotXtAssume;
double robotYtAssume;
double robotThtAssume;

//初始化标签信息
void EPC_Init()
{
	//建立所有的EPC，不难看出共有36个
	for (int i = 0; i < referenceTagNum; ++i)
		referenceTag_EPC[i] = "1037-9654-FFFF-FFFF-FFFF-00" + to_string(i + 10);

	//标签数据，用于构建标签阵列
	fstream fin;
	fin.open("TAG_POS.txt", ios::in);
	for (int i = 0; i < referenceTagNum; ++i)
		for (int j = 0; j < 3; ++j)
			fin >> referenceTag[i][j];
	fin.close();
}
//初始化天线
void AntennaInit()
{
	fstream fin;
	//导入天线参数
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

	//计算天线在移动机器人坐标系下的坐标，不难发现relative的[3][0]即第四个数是角度
	vector<double> antennaR_Robot =
	{
		(antennaR_POS[2] - AGV_Relative[2]) * cos(-AGV_Relative[3]) - (antennaR_POS[0] - AGV_Relative[0]) * sin(-AGV_Relative[3]),//机器人下的x坐标
		(antennaR_POS[2] - AGV_Relative[2]) * sin(-AGV_Relative[3]) + (antennaR_POS[0] - AGV_Relative[0]) * cos(-AGV_Relative[3]) //机器人下的y坐标
	};

	vector<double> antennaL_Robot =
	{
		(antennaL_POS[2] - AGV_Relative[2]) * cos(-AGV_Relative[3]) - (antennaL_POS[0] - AGV_Relative[0]) * sin(-AGV_Relative[3]),//机器人下的x坐标
		(antennaL_POS[2] - AGV_Relative[2]) * sin(-AGV_Relative[3]) + (antennaL_POS[0] - AGV_Relative[0]) * cos(-AGV_Relative[3]) //机器人下的y坐标
	};

	//天线高度和角度，勾股定理
	double antennaHRight = sqrt(antennaR_Robot[0] * antennaR_Robot[0] + antennaR_Robot[1] * antennaR_Robot[1]) * 100;
	antennaHRightError = antennaHRight - 0;
	double antennaAlphaRight = atan(abs(antennaR_Robot[0] / antennaR_Robot[1])) + PI / 2; //45 / 180 * PI
	antennaAlphaRightError = antennaAlphaRight + 0 / 180 * PI;
	double antennaHLeft = sqrt(antennaL_Robot[0] * antennaL_Robot[0] + antennaL_Robot[1] * antennaL_Robot[1]) * 100;
	antennaHLeftError = antennaHLeft + 0;
	double antennaAlphaLeft = atan(abs(antennaL_Robot[0] / antennaL_Robot[1])) + PI / 2; //45 / 180 * PI
	antennaAlphaLeftError = antennaAlphaLeft + 0 / 180 * PI;

	//左右天线高度
	myVisionRightAntennaHigh = antennaR_POS[1] * 100;
	myVisionLeftAntennaHigh = antennaL_POS[1] * 100;
}
//初始化移动机器人本体的遮挡区域
void ShadowInit()
{
	//左天线
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

	//右天线
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