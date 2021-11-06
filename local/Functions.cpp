#include "Functions.h"
#include <iostream>
#include <map>
#include <algorithm>
#include <mat.h>
using namespace std;

//�������ṹ�嶨��
struct cmpByValue
{
	bool operator()(const pair<int, double>& leftPair, const pair<int, double>& rightPair)
	{
		return leftPair.second < rightPair.second;
	}
};
//��ȡ���������ֵ ��һ������
int mymax(const vector<int>& v)
{
	int m = v.front();
	for (auto x : v)
		m = max(m, x);
	return m;
}
//��̬�ֲ����ɺ�������������Ϊmu����׼��Ϊsigma����̬�ֲ�
double gaussrand(const double& mu, const double& sigma)
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
//������룬v1��v2��������С��ͬ������
double distance(const vector<double>& v1, const vector<double>& v2)
{
	double sum = 0;
	for (int i = 0; i < v1.size(); ++i)
		sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
	return sqrt(sum);
}
//ʵ��k-�ٽ��㷨
vector<int> knnsearch(vector<vector<double>> testData, vector<double> targetData, int k)
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
//�жϵ��Ƿ�����������
vector<bool> inpolygon(const vector<vector<double>>& pointData, const vector<int>& optionalTagFlag, const vector<vector<double>>& testData,const int& numOfTag)
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

	//��ʼk-�����㷨
	vector<bool> result(numOfTag);
	for (int i = 0; i < numOfTag; ++i)
	{
		bool flag = false;	   //�жϽ����true�������ڶ�����ڣ�false:��δ���ڶ�����ڣ�
		int k = testData.size() - 1; //�Ƕ���ε����һ������
		for (int j = 0; j < testData.size(); ++j)
		{
			//�жϵ��Ƿ����߶ε�����
			if ((yLine[j] < yPoint[i] && yLine[k] >= yPoint[i]) || (yLine[k] < yPoint[i] && yLine[j] >= yPoint[i]))
			{
				//��������ʽ���̼��������P��ƽ����X���ֱ�����߶εĽ��㣬����ʽ���̣�x = x1 +  (y - y1) * (x2 - x1) / (y2 - y1);
				if (xLine[j] + (yPoint[i] - yLine[j]) * (xLine[k] - xLine[j]) / (yLine[k] - yLine[j]) < xPoint[i])
					flag = !flag;
			}
			//������һ�߶��ж�
			k = j;
		}
		result[i] = flag;
	}
	return result;
}
//�ҵ���̼Ƶ�ʱ�����Ӿ�ʱ�������е�ǰ�����������λ�˵�
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
//������������ڴ��·�Χ�����ڳ�ʼ�����ӣ�
vector<vector<double>> calPF_Scope(vector<vector<double>>& referenceTag, vector<int>& randData, vector<int>& colLeft, vector<int>& colRight)
{
	vector<vector<double>> PF_Scope(2, vector<double>(2));
	double leftMax = referenceTag[colLeft[0]][0];
	for (auto x : colLeft)
		leftMax = max(leftMax, referenceTag[x][0]);
	double rightMax = referenceTag[colRight[0]][0];
	for (auto x : colRight)
		rightMax = max(rightMax, referenceTag[x][0]);
	PF_Scope[0][0] = max(leftMax, rightMax) - 160 + randData[0];

	double leftMin = referenceTag[colLeft[0]][0];
	for (auto x : colLeft)
		leftMin = min(leftMin, referenceTag[x][0]);
	double rightMin = referenceTag[colRight[0]][0];
	for (auto x : colRight)
		rightMin = min(rightMin, referenceTag[x][0]);
	PF_Scope[0][1] = min(leftMin, rightMin) + 160 + randData[2];

	leftMax = referenceTag[colLeft[0]][1];
	for (auto x : colLeft)
		leftMax = max(leftMax, referenceTag[x][1]);
	rightMax = referenceTag[colRight[0]][1];
	for (auto x : colRight)
		rightMax = max(rightMax, referenceTag[x][1]);
	PF_Scope[1][0] = max(leftMax, rightMax) - 160 + randData[1];

	leftMin = referenceTag[colLeft[0]][1];
	for (auto x : colLeft)
		leftMin = min(leftMin, referenceTag[x][1]);
	rightMin = referenceTag[colRight[0]][1];
	for (auto x : colRight)
		rightMin = min(rightMin, referenceTag[x][1]);
	PF_Scope[1][1] = min(leftMin, rightMin) + 160 + randData[3];

	return PF_Scope;
}
//��ȡ.mat�ļ�
//void readmat(const char* matPath, const char** fieldName, double*** q,int len)
//{
//	//�Ӿ�����
//	//ǰ�ڹ���
//	//�����ļ�·��
//	//const char* p = R"(E:\MATLAB workspace\hgdw\0319_3.mat)";
//	//��������
//	//const char* name[] = { "AGV_relative", "AtennaL_POS", "AtennaR_POS", "AGV11", "TAG_POS" };
//	//���Ӿ������ļ�
//	MATFile* pM = matOpen(matPath, "r");
//	//��ȡ��Ϊ"my_vision_total"�ı���������matlab����һ���ṹ��
//	mxArray* pS = matGetVariable(pM, "my_vision_total");
//	//MyVisionTotal myVisionTotal;
//	//�ṹ����Ԫ�صĵ�ַ
//	//double*** q = &(myVisionTotal.AGV_Relative);
//
//	//ѭ���洢
//	for (int i = 0; i < len; ++i)
//	{
//		//�򿪶�Ӧ���������PF��ŵ��Ǹ�field��ָ��
//		mxArray* pF = mxGetField(pS, 0, fieldName[i]);
//		//�����е����ݴ���һ����ʱ����
//		double* temp = (double*)mxGetData(pF);
//		//��ȡ���ݵ�����
//		auto row = mxGetM(pF);
//		//��ȡ���ݵ�����
//		auto col = mxGetN(pF);
//		//�½�ָ�����飬�⻹�Ǹ�����ָ�룬��������ڴ�ŵ���ÿһ�е��׵�ַ
//		*(q + i) = new double* [row];
//		for (int j = 0; j < row; ++j)
//		{
//			//�½� double ���͵�����
//			*(*(q + i) + j) = new double[col];
//			for (int k = 0; k < col; ++k)
//				//����ʱ������ֵ�����ṹ��ı���
//				*(*(*(q + i) + j) + k) = temp[k * row + j];
//		}
//	}
//	//���ˣ������Ѱ���.mat�ļ��е��Ų�ȫ���������C++�е�myVisionTotal
//}