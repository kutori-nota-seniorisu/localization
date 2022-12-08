#ifndef _DATA_PROCESS_H
#define _DATA_PROCESS_H

#include <vector>
#include <fstream>
#include <map>
#include <string>
#include <algorithm>
using namespace std;

struct Data
{
	vector<int> no;			//1
	vector<double> phase;		//2
	vector<double> myThree;		//3
	vector<double> myFour;		//4
	vector<double> Xcoordinate; 	//5
	vector<double> Ycoordinate; 	//6
	vector<double> Zcoordinate; 	//7
	vector<double> readerTime;	//8
	vector<double> windowsTime;     //9
	vector<double> myTen;		//10
	vector<int> RSSI;		//11 信号的发射强度
};
struct TextData
{
	vector<int> no;
	vector<string> epcData;
};
struct MyDataResult
{
	Data data;
	TextData textData;
};

//PI
const double PI = 3.1415926535898;
//相位预处理
const double indexLb = 0.5; //0.5;%1/2 3/4 2/3
const double indexUb = 1.5; //3/2 5/4 4/3
//标签个数
const int referenceTagNum = 36;
//是否执行重采样阈值
const double neffRatioThreshold = 0.6;
//标签可读性的相关判定值
//不可读的半径，一定大于此值
const int tagUnreadableRadius = 20;
//可读的半径，一定小于此值
const int tagReadableRadius = 110;
//数据精度测试阈值
const double ep = 0.0001;
//光速
const int vLight = 300000000;
//单位 raidan/s,标准差，应该是观测噪声的标准差
const double PhaseGauss = 0.1;
//视觉起点
const int visionStartPoint = 308;
//右天线里程计起点
const int odometerRightStartPoint = 223;
//左天线里程计起点
const int odometerLeftStartPoint = 115;
//波长，因只用到第一个数据，故修改为如下定义
const double waveLengthVar = vLight / 920.625 * 0.0001;
//粒子滤波循环次数，虽然并没有循环这么多次
const int Times = 400;
//在粒子滤波更新中，标签可读性需要判断的周围的标签数量
const int PF_TagReadableCount = 5;
//在粒子滤波中，数据段长度的下界
const int distanceIntervalLp = 1;
//单位cm
const int gradientLen = 2;
//单位cm
const int distanceFarThreshold = 200;
//时间限制，单位s
const int gradientTimeLen = 10;
//粒子数量
const int PF_Count = 500;
//过程噪声，标准差，单位cm, rad
const vector<vector<double>> PF_Q = { {3, 0}, {0, 0.1} };
//测量噪声，方差，单位rad
const double PF_R = (PhaseGauss + 0.4) * (PhaseGauss + 0.4);

//排序规则结构体定义
struct cmpByValue
{
	bool operator()(const pair<int, double>& leftPair, const pair<int, double>& rightPair)
	{
		return leftPair.second < rightPair.second;
	}
};
//处理相位数据
vector<vector<double>> PhaseProcess(vector<double> epcPhase, const vector<vector<int>>& referenceTagNumTotal, const vector<int>& referenceTagCount);
//寻找指定范围内信号强度最大值的索引
int FindMaxRSSI(const vector<int>& RSSI_Data, const vector<vector<int>>& referenceTagNumTotal);
//升序排序，获取索引
vector<int> AscSort(vector<double> RSSI_MeanTag);
//判断点是否落入多边形内
vector<bool> Inpolygon(const vector<vector<double>>& pointData, const vector<int>& optionalTagFlag, const vector<vector<double>>& testData, const int& numOfTag);
//正态分布生成函数，生成期望为mu，标准差为sigma的正态分布
double GaussRand(const double& mu, const double& sigma);
//计算距离，v1与v2是两个大小相同的向量
double distance(const vector<double>& v1, const vector<double>& v2);
//实现k-临近算法
vector<int> knnsearch(const vector<vector<double>>& testData, const vector<double>& targetData, const int& k);
//计算机器人所在大致范围（用于初始化粒子）
vector<vector<double>> calPF_Scope(const vector<vector<double>>& referenceTag, const vector<int>& colLeft, const vector<int>& colRight);
//找到里程计的时间在视觉时间序列中的前后两个最近的位姿点
int findpso(const vector<double>& visionTimeDiff, const vector<double>& odometerTimeDiff, const int& index, bool& flag);
int findpre(const vector<double>& visionTimeDiff, const vector<double>& odometerTimeDiff, const int& index, bool& flag);
//计数，统计不为n的个数
template<typename T>
int FindNotN(vector<T> DataToCal, T n)
{
	int count = 0;
	for (auto x : DataToCal)
		if (x != n)
			++count;
	return count;
}

#endif // !_DATA_PROCESS_H