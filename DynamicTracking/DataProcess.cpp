#include "DataProcess.h"

//处理相位数据
vector<vector<double>> PhaseProcess(vector<double> epcPhase, const vector<vector<int>>& referenceTagNumTotal, const vector<int>& referenceTagCount)
{
	//把角度转化为弧度
	for (double& x : epcPhase)
		x = x / 180 * PI;

	//行数是读取的最多次数，列数是标签总数
	vector<vector<double>> phaseMeasurementAdjust(referenceTagNumTotal.size(), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCount[j] > 1)
		{
			//左边的零是减过1，0表示被读取过一次，j表示第j个标签。故下面一行代码是每个标签被第一次读取的相位
			phaseMeasurementAdjust[0][j] = epcPhase[referenceTagNumTotal[0][j]];
			for (int i = 1; i < referenceTagCount[j]; ++i)
			{
				//预处理
				//把后一次与前一次的相位差调整在-pi/2~pi/2内
				double dif = epcPhase[referenceTagNumTotal[i][j]] - phaseMeasurementAdjust[i - 1][j];
				if ((dif > PI * indexLb) && (dif < PI * indexUb))
					phaseMeasurementAdjust[i][j] = epcPhase[referenceTagNumTotal[i][j]] - PI;
				else if ((-dif > PI * indexLb) && (-dif < PI * indexUb))
					phaseMeasurementAdjust[i][j] = epcPhase[referenceTagNumTotal[i][j]] + PI;
				else
					phaseMeasurementAdjust[i][j] = epcPhase[referenceTagNumTotal[i][j]];
			}
		}

	//这步处理有何作用？
	for (auto& vec : phaseMeasurementAdjust)
		for (auto& x : vec)
			x = 2 * PI - x;

	//行数是读取的最多次数，列数是标签总数
	//下面一段代码将前后两次读取的相位差值缩小到PI内
	vector<vector<double>> PF_ObserveNative(referenceTagNumTotal.size(), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCount[j] > 1)
		{
			PF_ObserveNative[0][j] = phaseMeasurementAdjust[0][j];
			for (int k = 1; k < referenceTagCount[j]; ++k)
			{
				//四舍五入为最近的整数
				int m = round((phaseMeasurementAdjust[k][j] - PF_ObserveNative[k - 1][j]) / PI);
				//但这一步还是有些疑惑
				PF_ObserveNative[k][j] = phaseMeasurementAdjust[k][j] - PI * m;
			}
		}

	return PF_ObserveNative;
}
//寻找指定范围内信号强度最大值的索引
int FindMaxRSSI(const vector<int>& RSSI_Data, const vector<vector<int>>& referenceTagNumTotal)
{
	//寻找指定范围内的最大值以及索引
	//最大值
	int RSSI_Max = RSSI_Data[0];
	//最大值在指定范围内的索引
	int index = 0;
	for (int j = 0; j < RSSI_Data.size(); ++j)
		if (RSSI_Data[j] > RSSI_Max)
		{
			RSSI_Max = RSSI_Data[j];
			index = j;
		}

	//寻找相等并记录行与列的信息
	int col = 0;
	for (int j = 0; j < referenceTagNumTotal.size(); ++j)
		for (int k = 0; k < referenceTagNum; ++k)
			if (referenceTagNumTotal[j][k] == index)
				col = k;

	return col;
}
//升序排序，获取索引
vector<int> AscSort(vector<double> RSSI_MeanTag)
{
	//需要被排序的索引
	vector<int> I_RSSI(referenceTagNum);
	for (int j = 0; j < referenceTagNum; ++j)
		I_RSSI[j] = j;
	for (int j = 0; j < referenceTagNum; ++j)
	{
		bool f = false;
		for (int k = referenceTagNum - 1; k > j; --k)
			if (RSSI_MeanTag[k] < RSSI_MeanTag[k - 1])
			{
				int temp = I_RSSI[k];
				I_RSSI[k] = I_RSSI[k - 1];
				I_RSSI[k - 1] = temp;
				double tem = RSSI_MeanTag[k];
				RSSI_MeanTag[k] = RSSI_MeanTag[k - 1];
				RSSI_MeanTag[k - 1] = tem;
				f = true;
			}
		if (!f)
			break;
	}

	return I_RSSI;
}
//判断点是否落入多边形内
vector<bool> Inpolygon(const vector<vector<double>>& pointData, const vector<int>& optionalTagFlag, const vector<vector<double>>& testData, const int& numOfTag)
{
	vector<double> xPoint(numOfTag);
	vector<double> yPoint(numOfTag);
	for (int k = 0; k < numOfTag; ++k)
	{
		xPoint[k] = pointData[optionalTagFlag[k]][0];
		yPoint[k] = pointData[optionalTagFlag[k]][1];
	}
	vector<double> xLine(4);
	vector<double> yLine(4);
	for (int k = 0; k < 4; ++k)
	{
		xLine[k] = testData[k][2];
		yLine[k] = testData[k][3];
	}

	//开始k-近邻算法
	vector<bool> result(numOfTag);
	for (int i = 0; i < numOfTag; ++i)
	{
		bool flag = false;	   //判断结果（true；点落在多边形内；false:点未落在多边形内）
		int k = testData.size() - 1; //是多边形的最后一个顶点
		for (int j = 0; j < testData.size(); ++j)
		{
			//判断点是否在线段的两侧
			if ((yLine[j] < yPoint[i] && yLine[k] >= yPoint[i]) || (yLine[k] < yPoint[i] && yLine[j] >= yPoint[i]))
			{
				//根据两点式方程计算出过点P且平行于X轴的直线与线段的交点，两点式方程：x = x1 +  (y - y1) * (x2 - x1) / (y2 - y1);
				if (xLine[j] + (yPoint[i] - yLine[j]) * (xLine[k] - xLine[j]) / (yLine[k] - yLine[j]) < xPoint[i])
					flag = !flag;
			}
			//进行下一线段判断
			k = j;
		}
		result[i] = flag;
	}

	return result;
}
//正态分布生成函数，生成期望为mu，标准差为sigma的正态分布
double GaussRand(const double& mu, const double& sigma)
{
	static double V1, V2, S;
	static int phase = 0;
	double X;
	if (phase == 0)
	{
		do
		{
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;
			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);
		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);
	phase = 1 - phase;
	X = X * sigma + mu;
	return X;
}
//计算距离，v1与v2是两个大小相同的向量
double distance(const vector<double>& v1, const vector<double>& v2)
{
	double sum = 0;
	for (int i = 0; i < v1.size(); ++i)
		sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
	return sqrt(sum);
}
//实现k-临近算法
vector<int> knnsearch(const vector<vector<double>>& testData, const vector<double>& targetData, const int& k)
{
	map<int, double> mapIndexDistance;
	for (int i = 0; i < testData.size(); ++i)
		mapIndexDistance[i] = distance(testData[i], targetData);
	vector<pair<int, double>> pairIndexDistance(mapIndexDistance.begin(), mapIndexDistance.end());
	sort(pairIndexDistance.begin(), pairIndexDistance.end(), cmpByValue());
	vector<int> vec(k);
	for (int i = 0; i < k; ++i)
		vec[i] = pairIndexDistance[i].first;
	return vec;
}
//计算机器人所在大致范围（用于初始化粒子）
vector<vector<double>> calPF_Scope(const vector<vector<double>>& referenceTag, const vector<int>& colLeft, const vector<int>& colRight)
{
	vector<vector<double>> PF_Scope(2, vector<double>(2));
	double leftMax = referenceTag[colLeft[0]][0];
	for (auto x : colLeft)
		leftMax = max(leftMax, referenceTag[x][0]);
	double rightMax = referenceTag[colRight[0]][0];
	for (auto x : colRight)
		rightMax = max(rightMax, referenceTag[x][0]);
	PF_Scope[0][0] = max(leftMax, rightMax) - 160;

	double leftMin = referenceTag[colLeft[0]][0];
	for (auto x : colLeft)
		leftMin = min(leftMin, referenceTag[x][0]);
	double rightMin = referenceTag[colRight[0]][0];
	for (auto x : colRight)
		rightMin = min(rightMin, referenceTag[x][0]);
	PF_Scope[0][1] = min(leftMin, rightMin) + 160;

	leftMax = referenceTag[colLeft[0]][1];
	for (auto x : colLeft)
		leftMax = max(leftMax, referenceTag[x][1]);
	rightMax = referenceTag[colRight[0]][1];
	for (auto x : colRight)
		rightMax = max(rightMax, referenceTag[x][1]);
	PF_Scope[1][0] = max(leftMax, rightMax) - 160;

	leftMin = referenceTag[colLeft[0]][1];
	for (auto x : colLeft)
		leftMin = min(leftMin, referenceTag[x][1]);
	rightMin = referenceTag[colRight[0]][1];
	for (auto x : colRight)
		rightMin = min(rightMin, referenceTag[x][1]);
	PF_Scope[1][1] = min(leftMin, rightMin) + 160;

	return PF_Scope;
}
//找到里程计的时间在视觉时间序列中的前后两个最近的位姿点
int findpso(const vector<double>& visionTimeDiff, const vector<double>& odometerTimeDiff, const int& index, bool& flag)
{
	int point = 0;
	for (auto it = visionTimeDiff.begin(); it != visionTimeDiff.end(); ++it)
	{
		if ((*it) >= odometerTimeDiff[index] && !flag)
		{
			point = it - visionTimeDiff.begin();
			flag = true;
		}
		if (flag)
			break;
	}
	return point;
}
int findpre(const vector<double>& visionTimeDiff, const vector<double>& odometerTimeDiff, const int& index, bool& flag)
{
	int point = 0;
	for (auto it = visionTimeDiff.end() - 1; it >= visionTimeDiff.begin(); --it)
	{
		if (((*it) <= odometerTimeDiff[index]) && !flag)
		{
			point = it - visionTimeDiff.begin();
			flag = true;
		}
		if (flag)
			break;
	}
	return point;
}