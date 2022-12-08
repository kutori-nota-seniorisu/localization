#include <iostream>
#include <fstream>
#include <cmath>
#include <numeric>
#include <malloc.h>
#include <mat.h>
#include <windows.h>
#include <graphics.h>
#include "DataProcess.h"
#include "DataInit.h"
using namespace std;

int main()
{
	//初始化窗口640x480
	initgraph(640, 480);
	setbkcolor(WHITE);
	cleardevice();

	//系统数据导入
	//RFID和机器人数据
	fstream fin;
	fin.open("myData - leftAntenna.txt", ios::in);
	MyDataResult myDataResultLeft;
	while (1)
	{
		int n;
		string s;
		double d;
		//到达文件结尾需再次读入，eof才会变为true
		fin >> n;
		if (fin.eof())
			break;
		myDataResultLeft.textData.no.push_back(n);
		fin >> s;
		myDataResultLeft.textData.epcData.push_back(s);
		fin >> n;
		myDataResultLeft.data.no.push_back(n);
		fin >> d;
		myDataResultLeft.data.phase.push_back(d);
		fin >> d;
		myDataResultLeft.data.myThree.push_back(d);
		fin >> d;
		myDataResultLeft.data.myFour.push_back(d);
		fin >> d;
		myDataResultLeft.data.Xcoordinate.push_back(d);
		fin >> d;
		myDataResultLeft.data.Ycoordinate.push_back(d);
		fin >> d;
		myDataResultLeft.data.Zcoordinate.push_back(d);
		fin >> d;
		myDataResultLeft.data.readerTime.push_back(d);
		fin >> d;
		myDataResultLeft.data.windowsTime.push_back(d);
		fin >> d;
		myDataResultLeft.data.myTen.push_back(d);
		fin >> n;
		myDataResultLeft.data.RSSI.push_back(n);
	}
	fin.close();

	fin.open("myData - rightAntenna.txt", ios::in);
	MyDataResult myDataResultRight;
	while (1)
	{
		int n;
		string s;
		double d;
		fin >> n;
		if (fin.eof())
			break;
		myDataResultRight.textData.no.push_back(n);
		fin >> s;
		myDataResultRight.textData.epcData.push_back(s);
		fin >> n;
		myDataResultRight.data.no.push_back(n);
		fin >> d;
		myDataResultRight.data.phase.push_back(d);
		fin >> d;
		myDataResultRight.data.myThree.push_back(d);
		fin >> d;
		myDataResultRight.data.myFour.push_back(d);
		fin >> d;
		myDataResultRight.data.Xcoordinate.push_back(d);
		fin >> d;
		myDataResultRight.data.Ycoordinate.push_back(d);
		fin >> d;
		myDataResultRight.data.Zcoordinate.push_back(d);
		fin >> d;
		myDataResultRight.data.readerTime.push_back(d);
		fin >> d;
		myDataResultRight.data.windowsTime.push_back(d);
		fin >> d;
		myDataResultRight.data.myTen.push_back(d);
		fin >> n;
		myDataResultRight.data.RSSI.push_back(n);
	}
	fin.close();

	//初始化标签信息
	EPC_Init();
	//初始化天线
	AntennaInit();

	//绘制 - 未完成
	setcolor(RED);
	setfillcolor(RED);
	for (int i = 0; i < referenceTagNum; ++i)
		fillcircle(referenceTag[i][0] + 200, referenceTag[i][1] + 200, 5);

	/*移动机器人的真实位姿*/

	//视觉数据，用于获取机器人真实位姿
	fin.open("AGV11.txt", ios::in);
	vector<vector<double>> myVisionCurrent_AGV(4, vector<double>(4547));
	for (int i = 0; i < myVisionCurrent_AGV.size(); ++i)
		for (int j = 0; j < myVisionCurrent_AGV[0].size(); ++j)
			fin >> myVisionCurrent_AGV[i][j];
	fin.close();

	//// 没用到
	////读取，并对测试数据进行分类
	////用于验证的数据（机器人真实位置与位姿）
	////提取左右天线读取标签的epc、该epc标签的读取次数、标签的数量
	////左天线
	//vector<double> epcPhaseLeft(myDataResultLeft.data.phase);
	//for (double& x : epcPhaseLeft)
	//	//将相位转化为弧度
	//	x = x / 180 * PI;
	////textData中存放的是读取的次数和每一次读取到的epc，下面一段代码只是一个提取操作
	////修改了初始化方式，该变量中存储左天线读取到的标签信息
	//vector<string> epcDataLeft(myDataResultLeft.textData.epcData);
	////用来统计每个标签被读取的次数
	//vector<int> referenceTagCountLeft(referenceTagNum);
	////该变量为一个二维向量
	////向量的行：某一标签被读取的次数，列：某一标签序号
	////向量的每个元素为该标签在第几次被读取到
	//vector<vector<int>> referenceTagNumLeft;
	//for (int i = 0; i < epcDataLeft.size(); ++i)
	//	for (int j = 0; j < referenceTagNum; ++j)
	//		//判断已知标签与天线读取标签是否相同
	//		if (referenceTag_EPC[j] == epcDataLeft[i])
	//		{
	//			++referenceTagCountLeft[j];
	//			//由于不知道大小，故采用若空间过小则重新分配的方法
	//			//出于节省空间的想法，但在时间上有一定消耗
	//			if (referenceTagNumLeft.size() < referenceTagCountLeft[j])
	//			{
	//				vector<int> v(referenceTagNum, -1);
	//				referenceTagNumLeft.push_back(v);
	//			}
	//			// -1是为了在vector规定的内存里
	//			//把哪一个标签的哪一次读到是总体上读取的第几次记录到二维数组中
	//			referenceTagNumLeft[referenceTagCountLeft[j] - 1][j] = i;
	//			break;
	//		}
	////右天线
	//vector<double> epcPhaseRight(myDataResultRight.data.phase);
	//for (double& x : epcPhaseRight)
	//	//将相位转化为弧度
	//	x = x / 180 * PI;
	////修改了初始化方式，该变量中存储右天线读取到的标签信息
	//vector<string> epcDataRight(myDataResultRight.textData.epcData);
	////用来统计每个标签被读取的次数
	//vector<int> referenceTagCountRight(referenceTagNum);
	////该变量为一个二维向量
	////向量的行：某一标签被读取的次数，列：某一标签序号
	////向量的每个元素为该标签在第几次被读取到
	//vector<vector<int>> referenceTagNumRight;
	//for (int i = 0; i < epcDataRight.size(); ++i)
	//	for (int j = 0; j < referenceTagNum; ++j)
	//		//判断已知标签与天线读取标签是否相同
	//		if (referenceTag_EPC[j] == epcDataRight[i])
	//		{
	//			++referenceTagCountRight[j];
	//			//由于不知道大小，故采用若空间过小则重新分配的方法
	//			//出于节省空间的想法，但在时间上有一定消耗
	//			if (referenceTagNumRight.size() < referenceTagCountRight[j])
	//			{
	//				vector<int> v(referenceTagNum, -1);
	//				referenceTagNumRight.push_back(v);
	//			}
	//			// -1是为了在vector规定的内存里
	//			referenceTagNumRight[referenceTagCountRight[j] - 1][j] = i;
	//			break;
	//		}

	/*时间*/
	// 左天线数据无用
	//vector<double> readerTimeLeft = myDataResultLeft.data.readerTime;
	vector<double> readerTimeRight = myDataResultRight.data.readerTime;

	//// 没用到
	////相位预处理
	////左天线
	////行数是读取的最多次数，列数是标签总数
	//vector<vector<double>> phaseMeasurementAdjustLeft(referenceTagNumLeft.size(), vector<double>(referenceTagNum));
	//for (int j = 0; j < referenceTagNum; ++j)
	//	if (referenceTagCountLeft[j] > 1)
	//	{
	//		//左边的零是减过1，0表示被读取过一次，j表示第j个标签。故下面一行代码是每个标签被第一次读取的相位
	//		phaseMeasurementAdjustLeft[0][j] = epcPhaseLeft[referenceTagNumLeft[0][j]];
	//		for (int i = 1; i < referenceTagCountLeft[j]; ++i)
	//		{
	//			//预处理
	//			//把后一次与前一次的相位差调整在-pi/2~pi/2内
	//			double dif = epcPhaseLeft[referenceTagNumLeft[i][j]] - phaseMeasurementAdjustLeft[i - 1][j];
	//			if ((dif > PI * indexLb) && (dif < PI * indexUb))
	//				phaseMeasurementAdjustLeft[i][j] = epcPhaseLeft[referenceTagNumLeft[i][j]] - PI;
	//			else if ((-dif > PI * indexLb) && (-dif < PI * indexUb))
	//				phaseMeasurementAdjustLeft[i][j] = epcPhaseLeft[referenceTagNumLeft[i][j]] + PI;
	//			else
	//				phaseMeasurementAdjustLeft[i][j] = epcPhaseLeft[referenceTagNumLeft[i][j]];
	//		}
	//	}
	////这步处理有何作用？
	//if (accumulate(referenceTagCountLeft.begin(), referenceTagCountLeft.end(), 0) > 0)
	//	for (auto& vec : phaseMeasurementAdjustLeft)
	//		for (auto& x : vec)
	//			x = 2 * PI - x;
	////行数是读取的最多次数，列数是标签总数
	////下面一段代码将前后两次读取的相位差值缩小到PI内
	//vector<vector<double>> phasePsoLeft(referenceTagNumLeft.size(), vector<double>(referenceTagNum));
	//for (int j = 0; j < referenceTagNum; ++j)
	//	if (referenceTagCountLeft[j] > 1)
	//	{
	//		phasePsoLeft[0][j] = phaseMeasurementAdjustLeft[0][j];
	//		for (int i = 1; i < referenceTagCountLeft[j]; i++)
	//		{
	//			//四舍五入为最近的整数，若小数部分为0.5，则舍入为距离0更远的整数，如round(1.5)=2，round(-1.5)=-2
	//			int k = round((phaseMeasurementAdjustLeft[i][j] - phasePsoLeft[i - 1][j]) / PI);
	//			//但这一步还是有些疑惑
	//			phasePsoLeft[i][j] = phaseMeasurementAdjustLeft[i][j] - PI * k;
	//		}
	//	}
	////右天线
	////行数是读取的最多次数，列数是标签总数
	//vector<vector<double>> phaseMeasurementAdjustRight(referenceTagNumRight.size(), vector<double>(referenceTagNum));
	//for (int j = 0; j < referenceTagNum; ++j)
	//	if (referenceTagCountRight[j] > 1)
	//	{
	//		//左边的零是减过1，0表示被读取过一次，j表示第j个标签。故下面一行代码是每个标签被第一次读取的相位
	//		phaseMeasurementAdjustRight[0][j] = epcPhaseRight[referenceTagNumRight[0][j]];
	//		for (int i = 1; i < referenceTagCountRight[j]; ++i)
	//		{
	//			//预处理
	//			//把后一次与前一次的相位差调整在-pi/2~pi/2内
	//			double dif = epcPhaseRight[referenceTagNumRight[i][j]] - phaseMeasurementAdjustRight[i - 1][j];
	//			if ((dif > PI * indexLb) && (dif < PI * indexUb))
	//				phaseMeasurementAdjustRight[i][j] = epcPhaseRight[referenceTagNumRight[i][j]] - PI;
	//			else if ((-dif > PI * indexLb) && (-dif < PI * indexUb))
	//				phaseMeasurementAdjustRight[i][j] = epcPhaseRight[referenceTagNumRight[i][j]] + PI;
	//			else
	//				phaseMeasurementAdjustRight[i][j] = epcPhaseRight[referenceTagNumRight[i][j]];
	//		}
	//	}
	////这步处理有何作用？
	//for (auto& vec : phaseMeasurementAdjustRight)
	//	for (auto& x : vec)
	//		x = 2 * PI - x;
	////行数是读取的最多次数，列数是标签总数
	////下面一段代码将前后两次读取的相位差值缩小到PI内
	//vector<vector<double>> phasePsoRight(referenceTagNumRight.size(), vector<double>(referenceTagNum));
	//for (int j = 0; j < referenceTagNum; ++j)
	//	if (referenceTagCountRight[j] > 1)
	//	{
	//		phasePsoRight[0][j] = phaseMeasurementAdjustRight[0][j];
	//		for (int i = 1; i < referenceTagCountRight[j]; ++i)
	//		{
	//			//四舍五入为最近的整数
	//			int k = round((phaseMeasurementAdjustRight[i][j] - phasePsoRight[i - 1][j]) / PI);
	//			//但这一步还是有些疑惑
	//			phasePsoRight[i][j] = phaseMeasurementAdjustRight[i][j] - PI * k;
	//		}
	//	}

	//计算出视觉的时间差
	vector<double> visionTimeDiff(myVisionCurrent_AGV[0].size());
	for (int i = 0; i < myVisionCurrent_AGV[0].size() - visionStartPoint + 1; ++i)
		visionTimeDiff[i + visionStartPoint - 1] = 1.0 / 120 * (i + 1);
	//去除NaN
	for (int i = 0; i < myVisionCurrent_AGV[0].size();)
		if (isnan(myVisionCurrent_AGV[0][i]))
		{
			for (auto& vec : myVisionCurrent_AGV)
				vec.erase(vec.begin() + i, vec.begin() + i + 1);
			visionTimeDiff.erase(visionTimeDiff.begin() + i, visionTimeDiff.begin() + i + 1);
		}
		else
			++i;

	//纠正视觉所获得的移动机器人朝向角数据
	myVisionCurrent_AGV[2][0] = myVisionCurrent_AGV[2][0];
	for (int i = 1; i < myVisionCurrent_AGV[2].size(); ++i)
	{
		int k = round((myVisionCurrent_AGV[2][i] - myVisionCurrent_AGV[2][i - 1]) / PI);
		myVisionCurrent_AGV[2][i] = myVisionCurrent_AGV[2][i] - PI * k;
	}

	//计算出里程计的时间差
	vector<double> odometerTimeDiff(readerTimeRight.size() - odometerRightStartPoint + 1);
	for (int i = 0; i < odometerTimeDiff.size(); ++i)
		odometerTimeDiff[i] = (readerTimeRight[odometerRightStartPoint - 1 + i] - readerTimeRight[odometerRightStartPoint - 1]) / 1000000;

	//通过插值法获得移动机器人位姿点
	vector<vector<double>> mobileRobotPoseVisionRight(odometerTimeDiff.size() + odometerRightStartPoint - 1, vector<double>(4));
	for (int i = 0; i < odometerRightStartPoint - 1; ++i)
		for (int j = 0; j < myVisionCurrent_AGV.size(); ++j)
			mobileRobotPoseVisionRight[i][j] = myVisionCurrent_AGV[j][0];
	for (int j = 0; j < odometerTimeDiff.size(); ++j)
	{
		//先找到 里程计的时间 在 视觉时间序列中 的 前后两个最近 的位姿点
		bool preFlag = false;
		int pointPre = findpre(visionTimeDiff, odometerTimeDiff, j, preFlag);
		bool psoFlag = false;
		int pointPso = findpso(visionTimeDiff, odometerTimeDiff, j, psoFlag);
		if (psoFlag)
			for (int i = 0; i < 4; ++i)
				mobileRobotPoseVisionRight[j + odometerRightStartPoint - 1][i] = (odometerTimeDiff[j] - visionTimeDiff[pointPso]) / (visionTimeDiff[pointPre] - visionTimeDiff[pointPso]) * myVisionCurrent_AGV[i][pointPre] + (odometerTimeDiff[j] - visionTimeDiff[pointPre]) / (visionTimeDiff[pointPso] - visionTimeDiff[pointPre]) * myVisionCurrent_AGV[i][pointPso];
	}

	//// 没用到
	////计算出里程计的时间差
	//odometerTimeDiff.clear();
	//odometerTimeDiff.resize(readerTimeLeft.size() - odometerLeftStartPoint + 1);
	//for (int i = 0; i < odometerTimeDiff.size(); ++i)
	//	odometerTimeDiff[i] = (readerTimeLeft[odometerLeftStartPoint - 1 + i] - readerTimeLeft[odometerLeftStartPoint - 1]) / 1000000;
	////通过插值法获得移动机器人位姿点
	//vector<vector<double>> mobileRobotPoseVisionLeft(odometerTimeDiff.size() + odometerLeftStartPoint - 1, vector<double>(4));
	//for (int i = 0; i < odometerLeftStartPoint - 1; ++i)
	//	for (int j = 0; j < myVisionCurrent_AGV.size(); ++j)
	//		mobileRobotPoseVisionLeft[i][j] = myVisionCurrent_AGV[j][0];
	//for (int j = 0; j < odometerTimeDiff.size(); ++j)
	//{
	//	//先找到 里程计的时间 在 视觉时间序列中 的 前后两个最近 的位姿点
	//	bool preFlag = false;
	//	int pointPre = findpre(visionTimeDiff, odometerTimeDiff, j, preFlag);
	//	bool psoFlag = false;
	//	int pointPso = findpso(visionTimeDiff, odometerTimeDiff, j, psoFlag);
	//	if (psoFlag)
	//		for (int i = 0; i < 4; ++i)
	//			mobileRobotPoseVisionLeft[j + odometerLeftStartPoint - 1][i] = (odometerTimeDiff[j] - visionTimeDiff[pointPso]) / (visionTimeDiff[pointPre] - visionTimeDiff[pointPso]) * myVisionCurrent_AGV[i][pointPre] + (odometerTimeDiff[j] - visionTimeDiff[pointPre]) / (visionTimeDiff[pointPso] - visionTimeDiff[pointPre]) * myVisionCurrent_AGV[i][pointPso];
	//}

	int nFlag = 0;
	ofstream fout("out2.txt", ios::out);
	//robotXt,robotYt,robotTht是机器人的真实位置与位姿
	//vector<double> robotXt(Times);
	//vector<double> robotYt(Times);
	//vector<double> robotTht(Times);
	//同理，应为左右天线真实位置
	//vector<vector<double>> antennaLeft(2, vector<double>(Times));
	//vector<vector<double>> antennaRight(2, vector<double>(Times));
	//获取机器人真实位置与位姿
	for (int i = 0; i < Times; ++i)
	{
		if (i == 0)
		{
			double robotXt = mobileRobotPoseVisionRight[nFlag][0];
			double robotYt = mobileRobotPoseVisionRight[nFlag][1];
			double robotTht = mobileRobotPoseVisionRight[nFlag][2];
			double antennaLeftX = robotXt - antennaHLeftError * cos(PI - antennaAlphaLeftError - robotTht);
			double antennaLeftY = robotYt + antennaHLeftError * sin(PI - antennaAlphaLeftError - robotTht);
			double antennaRightX = robotXt + antennaHRightError * cos(-antennaAlphaRightError + robotTht);
			double antennaRightY = robotYt + antennaHRightError * sin(-antennaAlphaRightError + robotTht);

			fout << "robot_x: " << robotXt << ", robot_y: " << robotYt << '\n';

			//机器人真实位置
			setcolor(BLUE);
			setfillcolor(BLUE);
			fillcircle(robotXt + 200, robotYt + 200, 5);
			//机器人左右天线
			setcolor(BLACK);
			setfillcolor(BLACK);
			fillcircle(antennaLeftX + 200, antennaLeftY + 200, 3);
			fillcircle(antennaRightX + 200, antennaRightY + 200, 3);
			Sleep(100);
		}
		else
		{
			int nFlagPre = nFlag;
			while (1)
			{
				++nFlag;
				if (nFlag < myDataResultRight.data.readerTime.size())
				{
					double timeDiff = myDataResultRight.data.readerTime[nFlag] - myDataResultRight.data.readerTime[nFlagPre];
					if (timeDiff >= 200 * 1000)
						break;
				}
				else
					//跳出当前循环
					break;
			}
			//跳出大循环
			if (nFlag >= myDataResultRight.data.readerTime.size())
				break;
			double robotXt = mobileRobotPoseVisionRight[nFlag][0];
			double robotYt = mobileRobotPoseVisionRight[nFlag][1];
			double robotTht = mobileRobotPoseVisionRight[nFlag][2];
			double antennaLeftX = robotXt - antennaHLeftError * cos(PI - antennaAlphaLeftError - robotTht);
			double antennaLeftY = robotYt + antennaHLeftError * sin(PI - antennaAlphaLeftError - robotTht);
			double antennaRightX = robotXt + antennaHRightError * cos(-antennaAlphaRightError + robotTht);
			double antennaRightY = robotYt + antennaHRightError * sin(-antennaAlphaRightError + robotTht);

			fout << "robot_x: " << robotXt << ", robot_y: " << robotYt << '\n';

			//机器人真实位置
			setcolor(BLUE);
			setfillcolor(BLUE);
			fillcircle(robotXt + 200, robotYt + 200, 5);
			//机器人左右天线
			setcolor(BLACK);
			setfillcolor(BLACK);
			fillcircle(antennaLeftX + 200, antennaLeftY + 200, 3);
			fillcircle(antennaRightX + 200, antennaRightY + 200, 3);
			Sleep(100);
		}
	}
	//getchar();

	/*执行粒子滤波定位算法*/
	ShadowInit();

	//右天线的标号，为方便索引故减一
	int numberFlag = 0;
	//左天线的标号，为方便索引故减一
	int numberFlagVice = 0;

	//协方差矩阵
	//vector<vector<vector<double>>> PF_CenterVar(Times, vector<vector<double>>(3, vector<double>(3)));

	//仅作为某种参考，可以不使用
	//vector<bool> phaseFlag(Times);
	//vector<double> neffRatioPre(Times);
	//vector<int> PF_ReSample(Times);
	//vector<double> time(Times);
	//vector<vector<double>> robotEstimationError(5, vector<double>(Times));

	for (int i = 0; i < Times; ++i)
	{
		auto startTime = clock();
		if (i == 0)
		{
			robotXtAssume = myDataResultRight.data.Xcoordinate[numberFlag];
			robotYtAssume = myDataResultRight.data.Ycoordinate[numberFlag];
			robotThtAssume = myDataResultRight.data.Zcoordinate[numberFlag];

			//找到静止条件下，所能够读到的标签，并依据这些标签的坐标，直接对粒子进行初始化
			//右天线读取的标签数据
			vector<string> epcDataRight(myDataResultRight.textData.epcData.begin(), myDataResultRight.textData.epcData.begin() + odometerRightStartPoint);

			//右天线读取的信号强度数据
			vector<int> RSSI_DataRightCurrent(myDataResultRight.data.RSSI.begin(), myDataResultRight.data.RSSI.begin() + odometerRightStartPoint);

			//右天线读取的标签数量，考虑信号强度
			vector<int> referenceTagCountRightCurrent(referenceTagNum);
			for (int j = 0; j < epcDataRight.size(); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					if ((referenceTag_EPC[k] == epcDataRight[j]) && (RSSI_DataRightCurrent[j] >= -53))
					{
						//数量加1
						++referenceTagCountRightCurrent[k];
						break;
					}

			//列数就是标签的位置，colRight中存储的是被读取且信号强度满足条件的标签序号
			vector<int> colRight;
			for (vector<int>::iterator it = referenceTagCountRightCurrent.begin(); it != referenceTagCountRightCurrent.end(); ++it)
				if (*it)
					colRight.push_back(it - referenceTagCountRightCurrent.begin());

			//左天线读取的标签数据
			vector<string> epcDataLeft(myDataResultLeft.textData.epcData.begin(), myDataResultLeft.textData.epcData.begin() + odometerLeftStartPoint);

			//左天线读取的信号强度数据
			vector<int> RSSI_DataLeftCurrent(myDataResultLeft.data.RSSI.begin(), myDataResultLeft.data.RSSI.begin() + odometerLeftStartPoint);

			//左天线读取的标签数量，考虑信号强度
			vector<double> referenceTagCountLeftCurrent(referenceTagNum);
			for (int j = 0; j < epcDataLeft.size(); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					if ((referenceTag_EPC[k] == epcDataLeft[j]) && (RSSI_DataLeftCurrent[j] >= -53))
					{
						++referenceTagCountLeftCurrent[k];
						break;
					}

			//列数就是标签的位置，colLeft中存储的是被读取且信号强度满足条件的标签序号
			vector<int> colLeft;
			for (auto it = referenceTagCountLeftCurrent.begin(); it != referenceTagCountLeftCurrent.end(); ++it)
				if (*it)
					colLeft.push_back(it - referenceTagCountLeftCurrent.begin());

			//机器人所在的大致范围
			vector<vector<double>> PF_Scope = calPF_Scope(referenceTag, colLeft, colRight);

			//依据该范围，生成粒子
			for (int j = 0; j < PF_Count; ++j)
			{
				//粒子x坐标
				PF_ParticleRobot[0][j] = PF_Scope[0][0] + rand() / double(RAND_MAX) * (PF_Scope[0][1] - PF_Scope[0][0]);
				//粒子y坐标
				PF_ParticleRobot[1][j] = PF_Scope[1][0] + rand() / double(RAND_MAX) * (PF_Scope[1][1] - PF_Scope[1][0]);
				//粒子方向
				PF_ParticleRobot[2][j] = (-PI) + rand() / double(RAND_MAX) * (PI - (-PI));
			}

			//机器人位姿所对应的天线的位置
			//for (int j = 0; j < PF_Count; ++j)
			//{
			//	PF_ParticleAntennaLeft[i][0][j] = PF_ParticleRobot[i][0][j] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobot[i][2][j]);
			//	PF_ParticleAntennaLeft[i][1][j] = PF_ParticleRobot[i][1][j] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobot[i][2][j]);
			//}
			//for (int j = 0; j < PF_Count; ++j)
			//{
			//	PF_ParticleAntennaRight[i][0][j] = PF_ParticleRobot[i][0][j] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobot[i][2][j]);
			//	PF_ParticleAntennaRight[i][1][j] = PF_ParticleRobot[i][1][j] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobot[i][2][j]);
			//}

			for (int j = 0; j < PF_Count; ++j)
				PF_W[1][j] = 1 / double(PF_Count);

			//x y坐标的均值及其协方差
			vector<double> PF_CenterMean(4);
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					PF_CenterMean[j] += PF_ParticleRobot[j][k] * PF_W[1][k];

			double weightSqr = 0;
			for (int j = 0; j < PF_Count; ++j)
				weightSqr += PF_W[1][j] * PF_W[1][j];
			double factor = 0;
			if (abs(weightSqr - 1.0) < sqrt(DBL_EPSILON))
				factor = 1.0;
			else
				factor = 1 / (1 - weightSqr);

			vector<vector<double>> meanDiff(2, vector<double>(PF_Count));
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					meanDiff[j][k] = PF_ParticleRobot[j][k] - PF_CenterMean[j];

			//for (int j = 0; j < 2; ++j)
			//	for (int k = 0; k < 2; ++k)
			//	{
			//		for (int m = 0; m < PF_Count; ++m)
			//			PF_CenterVar[i][j][k] += meanDiff[j][m] * PF_W[i][1][m] * meanDiff[k][m];
			//		PF_CenterVar[i][j][k] *= factor;
			//	}

			//角度均值及协方差：
			double sinsum = 0;
			for (int j = 0; j < PF_Count; ++j)
				sinsum += PF_W[1][j] * sin(PF_ParticleRobot[2][j]);

			double cossum = 0;
			for (int j = 0; j < PF_Count; ++j)
				cossum += PF_W[1][j] * cos(PF_ParticleRobot[2][j]);

			double resultantLength = sqrt(pow(sinsum, 2) + pow(cossum, 2));
			PF_CenterMean[2] = atan2(sinsum, cossum);
			//PF_CenterVar[i][2][2] = -2 * log(resultantLength);
			for (int j = 0; j < PF_Count; ++j)
				PF_CenterMean[3] += PF_ParticleRobot[2][j] * PF_W[1][j];

			//定位误差
			double robotXt = mobileRobotPoseVisionRight[numberFlag][0];
			double robotYt = mobileRobotPoseVisionRight[numberFlag][1];
			double robotTht = mobileRobotPoseVisionRight[numberFlag][2];

			double antennaLeftX = robotXt - antennaHLeftError * cos(PI - antennaAlphaLeftError - robotTht);
			double antennaLeftY = robotYt + antennaHLeftError * sin(PI - antennaAlphaLeftError - robotTht);
			double antennaRightX = robotXt + antennaHRightError * cos(-antennaAlphaRightError + robotTht);
			double antennaRightY = robotYt + antennaHRightError * sin(-antennaAlphaRightError + robotTht);

			//robotEstimationError[0][i] = PF_CenterMean[0][i] - robotXt[i];
			//robotEstimationError[1][i] = PF_CenterMean[1][i] - robotYt[i];
			//robotEstimationError[2][i] = sqrt(pow(robotEstimationError[0][i], 2) + pow(robotEstimationError[1][i], 2));
			//robotEstimationError[3][i] = PF_CenterMean[2][i] - robotTht[i];
			//robotEstimationError[4][i] = PF_CenterMean[3][i] - robotTht[i];

			//机器人真实位置
			setcolor(BLUE);
			setfillcolor(BLUE);
			fillcircle(robotXt + 200, robotYt + 200, 5);
			//机器人左右天线
			setcolor(BLACK);
			setfillcolor(BLACK);
			fillcircle(antennaLeftX + 200, antennaLeftY + 200, 3);
			fillcircle(antennaRightX + 200, antennaRightY + 200, 3);
			Sleep(100);
		}
		else
		{
			//右天线数据范围
			int numberFlagPre = numberFlag;

			while (1)
			{
				++numberFlag;
				if (numberFlag < myDataResultRight.data.readerTime.size())
				{
					double timeDiff = myDataResultRight.data.readerTime[numberFlag] - myDataResultRight.data.readerTime[numberFlagPre];
					if (timeDiff >= 200 * 1000)
						break;
				}
				else
					//跳出当前循环
					break;
			}
			//跳出大循环
			if (numberFlag >= myDataResultRight.data.readerTime.size())
				break;

			//状态转移：预测
			double robotXtDiff = myDataResultRight.data.Xcoordinate[numberFlag] - robotXtAssume;
			double robotYtDiff = myDataResultRight.data.Ycoordinate[numberFlag] - robotYtAssume;
			double robotThtDiff = myDataResultRight.data.Zcoordinate[numberFlag] - robotThtAssume;

			vector<double> positionDifference1 =
			{
				robotXtDiff * cos(-robotThtAssume) - robotYtDiff * sin(-robotThtAssume),
				robotXtDiff * sin(-robotThtAssume) + robotYtDiff * cos(-robotThtAssume)
			};

			robotXtAssume = myDataResultRight.data.Xcoordinate[numberFlag];
			robotYtAssume = myDataResultRight.data.Ycoordinate[numberFlag];
			robotThtAssume = myDataResultRight.data.Zcoordinate[numberFlag];

			//此时，用于计算的粒子还是上一时刻的粒子
			vector<vector<double>> positionDifference2(2, vector<double>(PF_Count));
			for (int j = 0; j < PF_Count; ++j)
			{
				positionDifference2[0][j] = positionDifference1[0] * cos(PF_ParticleRobot[2][j]) - positionDifference1[1] * sin(PF_ParticleRobot[2][j]);
				positionDifference2[1][j] = positionDifference1[0] * sin(PF_ParticleRobot[2][j]) + positionDifference1[1] * cos(PF_ParticleRobot[2][j]);
			}

			//生成粒子，从上一时刻递推到当前时刻，用同一个变量记录
			for (int j = 0; j < PF_Count; ++j)
			{
				PF_ParticleRobot[0][j] = PF_ParticleRobot[0][j] + positionDifference2[0][j] + GaussRand(0, PF_Q[0][0]);
				PF_ParticleRobot[1][j] = PF_ParticleRobot[1][j] + positionDifference2[1][j] + GaussRand(0, PF_Q[0][0]);
				PF_ParticleRobot[2][j] = PF_ParticleRobot[2][j] + robotThtDiff + GaussRand(0, PF_Q[1][1]);
			}

			//右天线读取的机器人移动数据
			vector<vector<double>> trackMobileRobotRight(3);
			trackMobileRobotRight[0].assign(myDataResultRight.data.Xcoordinate.begin() + numberFlagPre + 1, myDataResultRight.data.Xcoordinate.begin() + numberFlag + 1);
			trackMobileRobotRight[1].assign(myDataResultRight.data.Ycoordinate.begin() + numberFlagPre + 1, myDataResultRight.data.Ycoordinate.begin() + numberFlag + 1);
			trackMobileRobotRight[2].assign(myDataResultRight.data.Zcoordinate.begin() + numberFlagPre + 1, myDataResultRight.data.Zcoordinate.begin() + numberFlag + 1);

			//右天线读取的标签数据
			vector<string> epcDataRight(myDataResultRight.textData.epcData.begin() + numberFlagPre + 1, myDataResultRight.textData.epcData.begin() + numberFlag + 1);

			//右天线读取的信号强度数据
			vector<int> RSSI_DataRightCurrent(myDataResultRight.data.RSSI.begin() + numberFlagPre + 1, myDataResultRight.data.RSSI.begin() + numberFlag + 1);

			//用来统计每个标签被读取的次数（当前时刻，且满足信号强度条件）
			vector<int> referenceTagCountRightCurrent(referenceTagNum);
			for (int j = 0; j < epcDataRight.size(); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					//判断已知标签与天线读取标签是否相同，且信号强度是否满足要求
					if ((referenceTag_EPC[k] == epcDataRight[j]) && (RSSI_DataRightCurrent[j] >= -52))
					{
						//数量加1
						++referenceTagCountRightCurrent[k];
						break;
					}

			//用来统计每个标签被读取的次数（当前时刻，但不考虑信号强度条件）
			vector<int> referenceTagCountRight(referenceTagNum);
			//该变量为一个二维向量（当前时刻，但不考虑信号强度条件）
			//向量的行：某一标签被读取的次数，列：某一标签序号
			//向量的每个元素为该标签在第几次被读取到
			vector<vector<int>> referenceTagNumRight;
			for (int j = 0; j < epcDataRight.size(); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					//判断已知标签与天线读取标签是否相同，且信号强度是否满足要求
					if (referenceTag_EPC[k] == epcDataRight[j])
					{
						//数量加1
						++referenceTagCountRight[k];
						if (referenceTagNumRight.size() < referenceTagCountRight[k])
						{
							vector<int> v(referenceTagNum, -1);
							referenceTagNumRight.push_back(v);
						}
						referenceTagNumRight[referenceTagCountRight[k] - 1][k] = j;
						break;
					}

			//右天线读取的相位数据
			vector<double> epcPhaseRight(myDataResultRight.data.phase.begin() + numberFlagPre + 1, myDataResultRight.data.phase.begin() + numberFlag + 1);

			//将原始相位数据进行处理
			vector<vector<double>> PF_ObserveNativeRight = PhaseProcess(epcPhaseRight, referenceTagNumRight, referenceTagCountRight);

			//寻找指定范围内的索引
			int colRight = FindMaxRSSI(RSSI_DataRightCurrent, referenceTagNumRight);

			//找到离RSSI最大的点的最近的9个参考标签
			vector<int> optionalTagRightFlag = knnsearch(referenceTag, referenceTag[colRight], PF_TagReadableCount);

			//所探寻的标签的可读性
			vector<bool> rightTagReadFlag(PF_TagReadableCount);
			for (int j = 0; j < PF_TagReadableCount; ++j)
				rightTagReadFlag[j] = (referenceTagCountRight[optionalTagRightFlag[j]] > 1) && (referenceTagCountRightCurrent[optionalTagRightFlag[j]] > 0);

			//右天线
			vector<vector<int>> numberFlagRight(2, vector<int>(referenceTagNum));
			for (int j = 0; j < referenceTagNum; ++j)
			{
				if (referenceTagCountRightCurrent[j] > 0)
				{
					int numberFlagOption = referenceTagCountRight[j] - 1;
					if (referenceTagNumRight[numberFlagOption][j] == epcDataRight.size() - 1)
						--numberFlagOption;
					if (!isnan(double(numberFlagOption)))
						//因为是C++里的索引，所以不是==1而是==0
						if (numberFlagOption == 0)
						{
							//同理，预判为可能索引的存在，故赋值为0
							numberFlagRight[1][j] = 0;
							numberFlagRight[0][j] = 0;
						}
						else
						{
							numberFlagRight[1][j] = numberFlagOption;
							numberFlagRight[0][j] = numberFlagRight[1][j];
							double distanceInterval = sqrt(pow((trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[1][j]][j]] - trackMobileRobotRight[0].back()), 2) +
								pow((trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[1][j]][j]] - trackMobileRobotRight[1].back()), 2));
							if (distanceInterval < distanceFarThreshold)
								while (1)
								{
									distanceInterval = sqrt(pow((trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[1][j]][j]] - trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[0][j]][j]]), 2) +
										pow((trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[1][j]][j]] - trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[0][j]][j]]), 2));
									if (distanceInterval > gradientLen)
										break;
									else
										--numberFlagRight[0][j];
									if (numberFlagRight[0][j] < 1)
										break;
									double rightPhaseDis = sqrt(pow((trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[0][j]][j]] - trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[0][j] + 1][j]]), 2) +
										pow((trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[0][j]][j]] - trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[0][j] + 1][j]]), 2));
									if (rightPhaseDis > 7)
										if (distanceInterval > distanceIntervalLp)
										{
											++numberFlagRight[0][j];
											break;
										}
										else
										{
											numberFlagRight[1][j] = 0;
											numberFlagRight[0][j] = 0;
											break;
										}
								}
							else
							{
								numberFlagRight[1][j] = 0;
								numberFlagRight[0][j] = 0;
							}
						}
					else
					{
						numberFlagRight[1][j] = 0;
						numberFlagRight[0][j] = 0;
					}
				}
				else
				{
					numberFlagRight[1][j] = 0;
					numberFlagRight[0][j] = 0;
				}
			}

			//右天线：信号强度分析
			vector<vector<int>> RSSI_TagRight(referenceTagNumRight.size(), vector<int>(referenceTagNum));
			for (int j = 0; j < referenceTagNum; ++j)
				if (referenceTagCountRight[j] > 1)
					for (int k = 0; k < referenceTagCountRight[j]; ++k)
						RSSI_TagRight[k][j] = RSSI_DataRightCurrent[referenceTagNumRight[k][j]];

			//右天线
			vector<double> RSSI_MeanTagRight(referenceTagNum, -200);
			for (int j = 0; j < referenceTagNum; ++j)
				if (numberFlagRight[1][j] != numberFlagRight[0][j])
				{
					double sum = 0;
					for (int k = numberFlagRight[0][j]; k <= numberFlagRight[1][j]; ++k)
						sum += RSSI_TagRight[k][j];
					RSSI_MeanTagRight[j] = sum / (numberFlagRight[1][j] - numberFlagRight[0][j] + 1);
				}

			//升序排序
			vector<int> I_RSSI_Right = AscSort(RSSI_MeanTagRight);

			//右天线：计算每个标签的前后两个时刻点，移动机器人的位姿，及其对应的天线位置, 并计算其权值
			vector<double> PF_W_RightImprovement(PF_Count, -1);

			//右天线：选出最合适的标签
			if (RSSI_MeanTagRight[I_RSSI_Right.back()] > -53)
			{
				int j = I_RSSI_Right.back();
				if (numberFlagRight[1][j] != numberFlagRight[0][j])
				{
					//后一个时刻，粒子状态--右天线
					double robotXtAssumePso = trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[1][j]][j]];
					double robotYtAssumePso = trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[1][j]][j]];
					double robotThtAssumePso = trackMobileRobotRight[2][referenceTagNumRight[numberFlagRight[1][j]][j]];

					double robotXtDiff = robotXtAssumePso - robotXtAssume;
					double robotYtDiff = robotYtAssumePso - robotYtAssume;
					double robotThtDiff = robotThtAssumePso - robotThtAssume;

					vector<double> positionDifference1 =
					{
						robotXtDiff * cos(-robotThtAssume) - robotYtDiff * sin(-robotThtAssume),
						robotXtDiff * sin(-robotThtAssume) + robotYtDiff * cos(-robotThtAssume)
					};

					vector<vector<double>> positionDifference2(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[2][k]);
					}

					vector<vector<double>> PF_ParticleRobotRightPso(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotRightPso[0][k] = PF_ParticleRobot[0][k] + positionDifference2[0][k];
						PF_ParticleRobotRightPso[1][k] = PF_ParticleRobot[1][k] + positionDifference2[1][k];
						PF_ParticleRobotRightPso[2][k] = PF_ParticleRobot[2][k] + robotThtDiff;
					}

					vector<vector<double>> PF_ParticleAntennaRightPso(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						//x坐标
						PF_ParticleAntennaRightPso[0][k] = PF_ParticleRobotRightPso[0][k] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobotRightPso[2][k]);
						//y坐标
						PF_ParticleAntennaRightPso[1][k] = PF_ParticleRobotRightPso[1][k] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobotRightPso[2][k]);
					}

					//前一个时刻，粒子状态--右天线
					double robotXtAssumePre = trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[0][j]][j]];
					double robotYtAssumePre = trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[0][j]][j]];
					double robotThtAssumePre = trackMobileRobotRight[2][referenceTagNumRight[numberFlagRight[0][j]][j]];

					robotXtDiff = robotXtAssumePre - robotXtAssume;
					robotYtDiff = robotYtAssumePre - robotYtAssume;
					robotThtDiff = robotThtAssumePre - robotThtAssume;

					positionDifference1[0] = robotXtDiff * cos(-robotThtAssume) - robotYtDiff * sin(-robotThtAssume);
					positionDifference1[1] = robotXtDiff * sin(-robotThtAssume) + robotYtDiff * cos(-robotThtAssume);

					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[2][k]);
					}

					vector<vector<double>> PF_ParticleRobotRightPre(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotRightPre[0][k] = PF_ParticleRobot[0][k] + positionDifference2[0][k];
						PF_ParticleRobotRightPre[1][k] = PF_ParticleRobot[1][k] + positionDifference2[1][k];
						PF_ParticleRobotRightPre[2][k] = PF_ParticleRobot[2][k] + robotThtDiff;
					}

					vector<vector<double>> PF_ParticleAntennaRightPre(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						//x坐标
						PF_ParticleAntennaRightPre[0][k] = PF_ParticleRobotRightPre[0][k] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobotRightPre[2][k]);
						//y坐标
						PF_ParticleAntennaRightPre[1][k] = PF_ParticleRobotRightPre[1][k] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobotRightPre[2][k]);
					}

					//观测相位梯度，观测：保存右天线两个时刻之间的相位差值
					double PF_ObserveGradientRight = PF_ObserveNativeRight[numberFlagRight[1][j]][j] - PF_ObserveNativeRight[numberFlagRight[0][j]][j];

					//理论相位梯度
					vector<double> PF_DistanceRightAntennaPre(PF_Count);
					vector<double> PF_DistanceRightAntennaPso(PF_Count);
					//预测：每一行对应一个参考标签，右天线两个时刻之间的相位差值
					vector<double> PF_PredictionRight(PF_Count);
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceRightAntennaPre[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaRightPre[0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaRightPre[1][k]), 2) + pow((referenceTag[j][2] - myVisionRightAntennaHigh), 2)) * 2;
						PF_DistanceRightAntennaPso[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaRightPso[0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaRightPso[1][k]), 2) + pow((referenceTag[j][2] - myVisionRightAntennaHigh), 2)) * 2;
						PF_PredictionRight[k] = (PF_DistanceRightAntennaPso[k] - PF_DistanceRightAntennaPre[k]) * 2 * PI / waveLengthVar;
					}

					//粒子与观测值之间的差距,每一行对应一个参考标签
					vector<double> PF_DistanceRight(PF_Count);
					//粒子权重评估：用相位差
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceRight[k] = PF_PredictionRight[k] - PF_ObserveGradientRight;
						//求权重
						PF_W_RightImprovement[k] = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * exp(-pow(PF_DistanceRight[k], 2) / 2 / (2 * PF_R));
					}
				}
			}

			//左天线数据范围
			int numberFlagVicePre = numberFlagVice;
			//寻找满足条件的索引并附带上0，然后寻找最大值。
			for (auto it = myDataResultLeft.data.readerTime.begin(); it != myDataResultLeft.data.readerTime.end(); ++it)
				if ((*it) < myDataResultRight.data.readerTime[numberFlag])
					numberFlagVice = it - myDataResultLeft.data.readerTime.begin();

			//左天线读取的机器人移动数据
			vector<vector<double>> trackMobileRobotLeft(3);
			trackMobileRobotLeft[0].assign(myDataResultLeft.data.Xcoordinate.begin() + numberFlagVicePre + 1, myDataResultLeft.data.Xcoordinate.begin() + numberFlagVice + 1);
			trackMobileRobotLeft[1].assign(myDataResultLeft.data.Ycoordinate.begin() + numberFlagVicePre + 1, myDataResultLeft.data.Ycoordinate.begin() + numberFlagVice + 1);
			trackMobileRobotLeft[2].assign(myDataResultLeft.data.Zcoordinate.begin() + numberFlagVicePre + 1, myDataResultLeft.data.Zcoordinate.begin() + numberFlagVice + 1);

			//左天线读取标签时的时间信息（用于判断读取到的标签是否有用）
			vector<double> readerTimeLeft(myDataResultLeft.data.readerTime.begin() + numberFlagVicePre + 1, myDataResultLeft.data.readerTime.begin() + numberFlagVice + 1);

			//左天线读取的标签数据
			vector<string> epcDataLeft(myDataResultLeft.textData.epcData.begin() + numberFlagVicePre + 1, myDataResultLeft.textData.epcData.begin() + numberFlagVice + 1);

			//左天线读取的信号强度数据
			vector<int> RSSI_DataLeftCurrent(myDataResultLeft.data.RSSI.begin() + numberFlagVicePre + 1, myDataResultLeft.data.RSSI.begin() + numberFlagVice + 1);

			//用来统计每个标签被读取的次数（当前时刻）
			vector<int> referenceTagCountLeftCurrent(referenceTagNum);
			for (int j = 0; j < epcDataLeft.size(); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					//判断已知标签与天线读取标签是否相同，且信号强度是否满足要求
					if ((referenceTag_EPC[k] == epcDataLeft[j]) && (RSSI_DataLeftCurrent[j] >= -52))
					{
						//数量加1
						++referenceTagCountLeftCurrent[k];
						break;
					}

			//用来统计每个标签被读取的次数（当前时刻，但不考虑信号强度条件）
			vector<int> referenceTagCountLeft(referenceTagNum);
			//该变量为一个二维向量（当前时刻，但不考虑信号强度条件）
			//向量的行：某一标签被读取的次数，列：某一标签序号
			//向量的每个元素为该标签在第几次被读取到
			vector<vector<int>> referenceTagNumLeft;
			for (int j = 0; j < epcDataLeft.size(); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					//判断已知标签与天线读取标签是否相同，且信号强度是否满足要求
					if (referenceTag_EPC[k] == epcDataLeft[j])
					{
						//数量加1
						++referenceTagCountLeft[k];
						if (referenceTagNumLeft.size() < referenceTagCountLeft[k])
						{
							vector<int> v(referenceTagNum, -1);
							referenceTagNumLeft.push_back(v);
						}
						referenceTagNumLeft[referenceTagCountLeft[k] - 1][k] = j;
						break;
					}

			//左天线读取的相位数据
			vector<double> epcPhaseLeft(myDataResultLeft.data.phase.begin() + numberFlagVicePre + 1, myDataResultLeft.data.phase.begin() + numberFlagVice + 1);

			//将原始相位数据进行处理
			vector<vector<double>> PF_ObserveNativeLeft = PhaseProcess(epcPhaseLeft, referenceTagNumLeft, referenceTagCountLeft);

			//寻找指定范围内的索引
			int colLeft = FindMaxRSSI(RSSI_DataLeftCurrent, referenceTagNumLeft);

			//找到离RSSI最大的点的最近的9个参考标签
			vector<int> optionalTagLeftFlag = knnsearch(referenceTag, referenceTag[colLeft], PF_TagReadableCount);

			//所探寻的标签的可读性
			vector<bool> leftTagReadFlag(PF_TagReadableCount);
			for (int j = 0; j < PF_TagReadableCount; ++j)
				leftTagReadFlag[j] = (referenceTagCountLeft[optionalTagLeftFlag[j]] > 1) && (referenceTagCountLeftCurrent[optionalTagLeftFlag[j]] > 0);

			//左天线：找主天线下，到每个标签对应的数据段--左天线。注：超过一定时间的数据不用，该逻辑还未实现
			vector<vector<int>> numberFlagLeft(2, vector<int>(referenceTagNum));
			for (int j = 0; j < referenceTagNum; ++j)
			{
				if ((referenceTagCountLeftCurrent[j] > 0) && (numberFlagVice > 1))
				{
					int numberFlagOption = referenceTagCountLeft[j] - 1;
					if (referenceTagNumLeft[numberFlagOption][j] == epcDataLeft.size() - 1)
						--numberFlagOption;
					if (!isnan(double(numberFlagOption)))
						//因为是C++里的索引，所以不是==1而是==0
						if (numberFlagOption == 0)
						{
							//同理，预判为可能索引的存在，故赋值为0
							numberFlagLeft[1][j] = 0;
							numberFlagLeft[0][j] = 0;
						}
						else
						{
							numberFlagLeft[1][j] = numberFlagOption;
							numberFlagLeft[0][j] = numberFlagLeft[1][j];
							//所以为什么是left和right相减呢
							double distanceInterval = sqrt(pow((trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotRight[0].back()), 2) +
								pow((trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotRight[1].back()), 2));
							//double distanceInterval = sqrt(pow((trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotLeft[0].back()), 2) +
								//pow((trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotLeft[1].back()), 2));
							if (distanceInterval < distanceFarThreshold)
								while (1)
								{
									double distanceInterval = sqrt(pow((trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[0][j]][j]]), 2) +
										pow((trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[0][j]][j]]), 2));
									double leftTimeInterval = (readerTimeLeft[referenceTagNumLeft[numberFlagLeft[1][j]][j]] - readerTimeLeft[referenceTagNumLeft[numberFlagLeft[0][j]][j]]) / 1000000;
									//距离间隔不能大于...，时间间隔不能大于...
									if ((distanceInterval > gradientLen) || (leftTimeInterval > gradientTimeLen))
										break;
									else
										--numberFlagLeft[0][j];
									if (numberFlagLeft[0][j] < 1)
										break;
									double leftPhaseDis = sqrt(pow((trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[0][j]][j]] - trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[0][j] + 1][j]]), 2) +
										pow((trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[0][j]][j]] - trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[0][j] + 1][j]]), 2));
									if (leftPhaseDis > 7)
										if (distanceInterval > distanceIntervalLp)
										{
											++numberFlagLeft[0][j];
											break;
										}
										else
										{
											numberFlagLeft[1][j] = 0;
											numberFlagLeft[0][j] = 0;
											break;
										}
								}
							else
							{
								numberFlagLeft[1][j] = 0;
								numberFlagLeft[0][j] = 0;
							}
						}
					else
					{
						numberFlagLeft[1][j] = 0;
						numberFlagLeft[0][j] = 0;
					}
				}
				else
				{
					numberFlagLeft[1][j] = 0;
					numberFlagLeft[0][j] = 0;
				}
			}

			//左天线：信号强度分析
			vector<vector<int>> RSSI_TagLeft(referenceTagNumLeft.size(), vector<int>(referenceTagNum));
			for (int j = 0; j < referenceTagNum; ++j)
				if (referenceTagCountLeft[j] > 1)
					for (int k = 0; k < referenceTagCountLeft[j]; ++k)
						RSSI_TagLeft[k][j] = RSSI_DataLeftCurrent[referenceTagNumLeft[k][j]];

			//左天线
			vector<double> RSSI_MeanTagLeft(referenceTagNum, -200);
			for (int j = 0; j < referenceTagNum; ++j)
				if (numberFlagLeft[1][j] != numberFlagLeft[0][j])
				{
					double sum = 0;
					for (int k = numberFlagLeft[0][j]; k <= numberFlagLeft[1][j]; ++k)
						sum += RSSI_TagLeft[k][j];
					RSSI_MeanTagLeft[j] = sum / (numberFlagLeft[1][j] - numberFlagLeft[0][j] + 1);
				}

			//升序排序
			vector<int> I_RSSI_Left = AscSort(RSSI_MeanTagLeft);

			//左天线：计算每个标签的前后两个时刻点，移动机器人的位姿，及其对应的天线位置, 并计算其权值
			vector<double> PF_W_LeftImprovement(PF_Count, -1);

			if (RSSI_MeanTagLeft[I_RSSI_Left.back()] > -53)
			{
				int j = I_RSSI_Left.back();
				if (numberFlagLeft[1][j] != numberFlagLeft[0][j])
				{
					//后一个时刻，粒子状态--左天线
					double robotXtAssumePso = trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]];
					double robotYtAssumePso = trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]];
					double robotThtAssumePso = trackMobileRobotLeft[2][referenceTagNumLeft[numberFlagLeft[1][j]][j]];

					double robotXtDiff = robotXtAssumePso - robotXtAssume;
					double robotYtDiff = robotYtAssumePso - robotYtAssume;
					double robotThtDiff = robotThtAssumePso - robotThtAssume;

					vector<double> positionDifference1 =
					{
						robotXtDiff * cos(-robotThtAssume) - robotYtDiff * sin(-robotThtAssume),
						robotXtDiff * sin(-robotThtAssume) + robotYtDiff * cos(-robotThtAssume)
					};

					vector<vector<double>> positionDifference2(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[2][k]);
					}

					vector<vector<double>> PF_ParticleRobotLeftPso(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotLeftPso[0][k] = PF_ParticleRobot[0][k] + positionDifference2[0][k];
						PF_ParticleRobotLeftPso[1][k] = PF_ParticleRobot[1][k] + positionDifference2[1][k];
						PF_ParticleRobotLeftPso[2][k] = PF_ParticleRobot[2][k] + robotThtDiff;
					}

					vector<vector<double>> PF_ParticleAntennaLeftPso(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						//x坐标
						PF_ParticleAntennaLeftPso[0][k] = PF_ParticleRobotLeftPso[0][k] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPso[2][k]);
						//y坐标
						PF_ParticleAntennaLeftPso[1][k] = PF_ParticleRobotLeftPso[1][k] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPso[2][k]);
					}

					//前一个时刻，粒子状态--左天线
					double robotXtAssumePre = trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[0][j]][j]];
					double robotYtAssumePre = trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[0][j]][j]];
					double robotThtAssumePre = trackMobileRobotLeft[2][referenceTagNumLeft[numberFlagLeft[0][j]][j]];

					robotXtDiff = robotXtAssumePre - robotXtAssume;
					robotYtDiff = robotYtAssumePre - robotYtAssume;
					robotThtDiff = robotThtAssumePre - robotThtAssume;

					positionDifference1[0] = robotXtDiff * cos(-robotThtAssume) - robotYtDiff * sin(-robotThtAssume);
					positionDifference1[1] = robotXtDiff * sin(-robotThtAssume) + robotYtDiff * cos(-robotThtAssume);

					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[2][k]);
					}

					vector<vector<double>> PF_ParticleRobotLeftPre(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotLeftPre[0][k] = PF_ParticleRobot[0][k] + positionDifference2[0][k];
						PF_ParticleRobotLeftPre[1][k] = PF_ParticleRobot[1][k] + positionDifference2[1][k];
						PF_ParticleRobotLeftPre[2][k] = PF_ParticleRobot[2][k] + robotThtDiff;
					}

					vector<vector<double>> PF_ParticleAntennaLeftPre(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						//x坐标
						PF_ParticleAntennaLeftPre[0][k] = PF_ParticleRobotLeftPre[0][k] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPre[2][k]);
						//y坐标
						PF_ParticleAntennaLeftPre[1][k] = PF_ParticleRobotLeftPre[1][k] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPre[2][k]);
					}

					//观测相位梯度，观测：保存左天线两个时刻之间的相位差值
					double PF_ObserveGradientLeft = PF_ObserveNativeLeft[numberFlagLeft[1][j]][j] - PF_ObserveNativeLeft[numberFlagLeft[0][j]][j];

					//理论相位梯度
					vector<double> PF_DistanceLeftAntennaPre(PF_Count);
					vector<double> PF_DistanceLeftAntennaPso(PF_Count);
					//预测：每一行对应一个参考标签，左天线两个时刻之间的相位差值
					vector<double> PF_PredictionLeft(PF_Count);
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceLeftAntennaPre[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaLeftPre[0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaLeftPre[1][k]), 2) + pow((referenceTag[j][2] - myVisionLeftAntennaHigh), 2)) * 2;
						PF_DistanceLeftAntennaPso[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaLeftPso[0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaLeftPso[1][k]), 2) + pow((referenceTag[j][2] - myVisionLeftAntennaHigh), 2)) * 2;
						PF_PredictionLeft[k] = (PF_DistanceLeftAntennaPso[k] - PF_DistanceLeftAntennaPre[k]) * 2 * PI / waveLengthVar;
					}

					//粒子与观测值之间的差距,每一行对应一个参考标签
					vector<double> PF_DistanceLeft(PF_Count);
					//粒子权重评估：用相位差
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceLeft[k] = PF_PredictionLeft[k] - PF_ObserveGradientLeft;
						//求权重
						PF_W_LeftImprovement[k] = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * exp(-pow(PF_DistanceLeft[k], 2) / 2 / (2 * PF_R));
					}
				}
			}

			//计算当前位姿下，左右天线和阴影控制点位置
			vector<vector<double>> PF_ParticleAntennaLeftCurrent(2, vector<double>(PF_Count));
			vector<vector<double>> PF_ParticleAntennaRightCurrent(2, vector<double>(PF_Count));
			for (int j = 0; j < PF_Count; ++j)
			{
				PF_ParticleAntennaLeftCurrent[0][j] = PF_ParticleRobot[0][j] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobot[2][j]);
				PF_ParticleAntennaLeftCurrent[1][j] = PF_ParticleRobot[1][j] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobot[2][j]);
				PF_ParticleAntennaRightCurrent[0][j] = PF_ParticleRobot[0][j] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobot[2][j]);
				PF_ParticleAntennaRightCurrent[1][j] = PF_ParticleRobot[1][j] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobot[2][j]);
			}

			vector<int> PF_W_RSSI(PF_Count);
			for (int j = 0; j < PF_Count; ++j)
			{
				vector<vector<double>> PF_ParticleShadowLeftCurrent(4, vector<double>(4));
				vector<vector<double>> PF_ParticleShadowRightCurrent(4, vector<double>(4));
				for (int k = 0; k < 4; ++k)
				{
					PF_ParticleShadowLeftCurrent[k][0] = PF_ParticleRobot[0][j] + leftShadowUnreadablePointH[k] * cos(leftShadowUnreadablePointAlpha[k] + PF_ParticleRobot[2][j]);
					PF_ParticleShadowLeftCurrent[k][1] = PF_ParticleRobot[1][j] + leftShadowUnreadablePointH[k] * sin(leftShadowUnreadablePointAlpha[k] + PF_ParticleRobot[2][j]);
					PF_ParticleShadowLeftCurrent[k][2] = PF_ParticleRobot[0][j] + leftShadowReadablePointH[k] * cos(leftShadowReadablePointAlpha[k] + PF_ParticleRobot[2][j]);
					PF_ParticleShadowLeftCurrent[k][3] = PF_ParticleRobot[1][j] + leftShadowReadablePointH[k] * sin(leftShadowReadablePointAlpha[k] + PF_ParticleRobot[2][j]);

					PF_ParticleShadowRightCurrent[k][0] = PF_ParticleRobot[0][j] + rightShadowUnreadablePointH[k] * cos(rightShadowUnreadablePointAlpha[k] + PF_ParticleRobot[2][j]);
					PF_ParticleShadowRightCurrent[k][1] = PF_ParticleRobot[1][j] + rightShadowUnreadablePointH[k] * sin(rightShadowUnreadablePointAlpha[k] + PF_ParticleRobot[2][j]);
					PF_ParticleShadowRightCurrent[k][2] = PF_ParticleRobot[0][j] + rightShadowReadablePointH[k] * cos(rightShadowReadablePointAlpha[k] + PF_ParticleRobot[2][j]);
					PF_ParticleShadowRightCurrent[k][3] = PF_ParticleRobot[1][j] + rightShadowReadablePointH[k] * sin(rightShadowReadablePointAlpha[k] + PF_ParticleRobot[2][j]);
				}

				//逐个标签确定是否当前的移动机器人位姿是否符合要求
				vector<bool> optionalFlag(PF_TagReadableCount);

				//参考标签与天线的距离
				vector<double> leftDistanceThresholdFlag(PF_TagReadableCount);
				vector<double> rightDistanceThresholdFlag(PF_TagReadableCount);
				for (int k = 0; k < PF_TagReadableCount; ++k)
				{
					leftDistanceThresholdFlag[k] = sqrt(pow((referenceTag[optionalTagLeftFlag[k]][0] - PF_ParticleAntennaLeftCurrent[0][j]), 2)
						+ pow((referenceTag[optionalTagLeftFlag[k]][1] - PF_ParticleAntennaLeftCurrent[1][j]), 2));
					rightDistanceThresholdFlag[k] = sqrt(pow((referenceTag[optionalTagRightFlag[k]][0] - PF_ParticleAntennaRightCurrent[0][j]), 2)
						+ pow((referenceTag[optionalTagRightFlag[k]][1] - PF_ParticleAntennaRightCurrent[1][j]), 2));
				}

				//参考标签与阴影区的关系
				//参考标签是否在阴影区内
				vector<bool> leftInpolygonReadableFlag = Inpolygon(referenceTag, optionalTagLeftFlag, PF_ParticleShadowLeftCurrent, PF_TagReadableCount);

				//参考标签是否在阴影区内
				vector<bool> rightInpolygonReadableFlag = Inpolygon(referenceTag, optionalTagRightFlag, PF_ParticleShadowRightCurrent, PF_TagReadableCount);

				for (int k = 0; k < PF_TagReadableCount; ++k)
				{
					//左右天线都未读到
					if ((leftTagReadFlag[k] == 0) && (rightTagReadFlag[k] == 0))
					{
						//距离大于阈值，或者 在阴影区内|| left_inpolygon_unreadable_flag(k) == 1
						if (leftDistanceThresholdFlag[k] > tagUnreadableRadius)
							//距离大于阈值，或者 在阴影区内|| right_inpolygon_unreadable_flag(k) == 1
							if (rightDistanceThresholdFlag[k] > tagUnreadableRadius)
								optionalFlag[k] = 1;
					}
					//左天线读到，右天线未读到
					else if ((leftTagReadFlag[k] == 1) && (rightTagReadFlag[k] == 0))
					{
						//距离小于阈值，并且 不在阴影区内
						if ((leftDistanceThresholdFlag[k] < tagReadableRadius) && (leftInpolygonReadableFlag[k] == 0))
							//距离大于阈值，或者 在阴影区内|| right_inpolygon_unreadable_flag(k) == 1
							if (rightDistanceThresholdFlag[k] > tagUnreadableRadius)
								optionalFlag[k] = 1;
					}
					//左天线未读到，右天线读到
					else if ((leftTagReadFlag[k] == 0) && (rightTagReadFlag[k] == 1))
					{
						//距离大于阈值，或者 在阴影区内|| left_inpolygon_unreadable_flag(k) == 1
						if (leftDistanceThresholdFlag[k] > tagUnreadableRadius)
							//距离小于阈值，并且 不在阴影区内
							if ((rightDistanceThresholdFlag[k] < tagReadableRadius) && (rightInpolygonReadableFlag[k] == 0))
								optionalFlag[k] = 1;
					}
					//左右天线都读到
					else if ((leftTagReadFlag[k] == 1) && (rightTagReadFlag[k] == 1))
					{
						//距离小于阈值，并且 不再阴影区内
						if ((leftDistanceThresholdFlag[k] < tagReadableRadius) && (leftInpolygonReadableFlag[k] == 0))
							//距离小于阈值，并且 不再阴影区内
							if ((rightDistanceThresholdFlag[k] < tagReadableRadius) && (rightInpolygonReadableFlag[k] == 0))
								optionalFlag[k] = 1;
					}
				}

				//所有标签的可读性都符合常理
				if (FindNotN<bool>(optionalFlag, false) == PF_TagReadableCount)
					PF_W_RSSI[j] = 1;
				else
					PF_W_RSSI[j] = 0;
			}

			//下面的是不是太过繁琐了？写成一个函数？
			//粒子权重评估
			vector<vector<double>> PF_W_PrePe;
			int n = 0;
			if (FindNotN<double>(PF_W_RightImprovement, -1) != 0)
			{
				++n;
				PF_W_PrePe.resize(n);
				PF_W_PrePe[n - 1] = PF_W_RightImprovement;
			}
			if (FindNotN<double>(PF_W_LeftImprovement, -1) != 0)
			{
				++n;
				PF_W_PrePe.resize(n);
				PF_W_PrePe[n - 1] = PF_W_LeftImprovement;
			}

			vector<double> PF_W_Pre(PF_Count, 1);
			if (PF_W_PrePe.size() == 0)
			{
				//phaseFlag[i] = false;
			}
			else if (PF_W_PrePe.size() == 1)
			{
				double sum = 1;
				for (int j = 0; j < PF_Count; ++j)
					sum *= PF_W_PrePe[0][j];
				for (int j = 0; j < PF_Count; ++j)
					PF_W_Pre[j] = sum;
				//phaseFlag[i] = true;
			}
			else
			{
				for (int j = 0; j < PF_Count; ++j)
					PF_W_Pre[j] = PF_W_PrePe[0][j] * PF_W_PrePe[1][j];
				//phaseFlag[i] = true;
			}

			for (int j = 0; j < PF_Count; ++j)
			{
				PF_W[0][j] = PF_W_Pre[j] + 1e-99;
				PF_W[1][j] = PF_W[0][j] * PF_W[1][j] * PF_W_RSSI[j];
			}

			double sum = accumulate(PF_W[1].begin(), PF_W[1].end(), 0.0);
			for (auto& x : PF_W[1])
				x /= sum;

			//x，y坐标的均值及其协方差
			//所有粒子的中心位置--x，y坐标
			vector<double> PF_CenterMean(4);
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					PF_CenterMean[j] += PF_ParticleRobot[j][k] * PF_W[1][k];
			double weightSqr = 0;
			for (int j = 0; j < PF_Count; ++j)
				weightSqr += PF_W[1][j] * PF_W[1][j];

			double factor = 0;
			if (abs(weightSqr - 1.0) < sqrt(DBL_EPSILON))
				factor = 1.0;
			else
				factor = 1 / (1 - weightSqr);

			vector<vector<double>> meanDiff(2, vector<double>(PF_Count));
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					meanDiff[j][k] = PF_ParticleRobot[j][k] - PF_CenterMean[j];

			//for (int j = 0; j < 2; ++j)
			//	for (int k = 0; k < 2; ++k)
			//	{
			//		for (int m = 0; m < PF_Count; ++m)
			//			PF_CenterVar[i][j][k] += meanDiff[j][m] * PF_W[i][1][m] * meanDiff[k][m];
			//		PF_CenterVar[i][j][k] *= factor;
			//	}

			//角度均值及协方差：
			double sinsum = 0;
			for (int j = 0; j < PF_Count; ++j)
				sinsum += PF_W[1][j] * sin(PF_ParticleRobot[2][j]);
			double cossum = 0;
			for (int j = 0; j < PF_Count; ++j)
				cossum += PF_W[1][j] * cos(PF_ParticleRobot[2][j]);
			double resultantLength = sqrt(pow(sinsum, 2) + pow(cossum, 2));
			PF_CenterMean[2] = atan2(sinsum, cossum);
			//PF_CenterVar[i][2][2] = -2 * log(resultantLength);
			for (int j = 0; j < PF_Count; ++j)
				PF_CenterMean[3] += PF_ParticleRobot[2][j] * PF_W[1][j];

			//判断当前是否启动重采样
			double squaredSum = weightSqr;

			//有效粒子的数量
			double neff = 1 / squaredSum;
			//neffRatioPre[i] = neff / PF_Count;
			double neffRatioPre = neff / PF_Count;
			//if (neffRatioPre[i] < neffRatioThreshold)
			if (neffRatioPre < neffRatioThreshold)
			{
				//执行系统重采样
				int numNewParticles = PF_Count;
				double randNum = rand() / double(RAND_MAX);
				vector<double> randSamples(numNewParticles);
				for (int j = 0; j < numNewParticles; ++j)
					randSamples[j] = j * 1.0 / numNewParticles + randNum / numNewParticles;

				int n = 0;
				int index = 0;
				vector<double> Q(PF_Count);
				for (int j = 0; j < PF_Count; ++j)
					Q[j] = accumulate(PF_W[1].begin(), PF_W[1].begin() + j + 1, 0.0);
				vector<vector<double>> PF_ParticleNew(3, vector<double>(numNewParticles));
				while (n < numNewParticles)
				{
					while (Q[index] <= randSamples[n])
						index = index % (PF_Count - 1) + 1;
					//得到新粒子集
					for (int j = 0; j < 3; ++j)
						PF_ParticleNew[j][n] = PF_ParticleRobot[j][index];
					++n;
				}
				//得到最后的重采样粒子集
				PF_ParticleRobot = PF_ParticleNew;
				//求本次的最终权重
				for (auto& x : PF_W[1])
					x = 1.0 / PF_Count;
				//PF_W_Slow = 0;
				//PF_W_Fast = 0;
				//PF_ReSample[i] = 1;
			}
			else
			{
				//不执行重采样
				//PF_ReSample[i] = -1;
			}

			//定位误差
			double robotXt = mobileRobotPoseVisionRight[numberFlag][0];
			double robotYt = mobileRobotPoseVisionRight[numberFlag][1];
			double robotTht = mobileRobotPoseVisionRight[numberFlag][2];

			double antennaLeftX = robotXt - antennaHLeftError * cos(PI - antennaAlphaLeftError - robotTht);
			double antennaLeftY = robotYt + antennaHLeftError * sin(PI - antennaAlphaLeftError - robotTht);
			double antennaRightX = robotXt + antennaHRightError * cos(-antennaAlphaRightError + robotTht);
			double antennaRightY = robotYt + antennaHRightError * sin(-antennaAlphaRightError + robotTht);

			//robotEstimationError[0][i] = PF_CenterMean[0][i] - robotXt;
			//robotEstimationError[1][i] = PF_CenterMean[1][i] - robotYt;
			//robotEstimationError[2][i] = sqrt(pow(robotEstimationError[0][i], 2) + pow(robotEstimationError[1][i], 2));
			//robotEstimationError[3][i] = PF_CenterMean[2][i] - robotTht;
			//robotEstimationError[4][i] = PF_CenterMean[3][i] - robotTht;

			//机器人真实位置
			setcolor(BLUE);
			setfillcolor(BLUE);
			fillcircle(robotXt + 200, robotYt + 200, 5);
			//机器人左右天线
			setcolor(BLACK);
			setfillcolor(BLACK);
			fillcircle(antennaLeftX + 200, antennaLeftY + 200, 3);
			fillcircle(antennaRightX + 200, antennaRightY + 200, 3);
			Sleep(100);
			//绘图
			setcolor(YELLOW);
			setfillcolor(YELLOW);
			fillcircle(PF_CenterMean[0] + 200, PF_CenterMean[1] + 200, 3);
			Sleep(100);
		}

		//更新频率把控
		auto endTime = clock();
		//time[i] = (endTime - startTime) / CLOCKS_PER_SEC;
		//在Windows下时间按照毫秒计算，在Linux下时间按照秒计算(???)
		//Sleep((1 - time[i]) * 1000);
		// Sleep(1 - time[i]);
	}
	system("pause");
	closegraph();
	return 0;
}