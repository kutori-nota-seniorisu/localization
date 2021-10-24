#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <numeric>
#include <algorithm>
#include <malloc.h>
#include <mat.h>
#include <windows.h>
// #include <unistd.h>
using namespace std;
struct Data
{
	vector<int> no;				//1
	vector<double> phase;		//2
	vector<double> myThree;		//3
	vector<double> myFour;		//4
	vector<double> Xcoordinate; //5
	vector<double> Ycoordinate; //6
	vector<double> Zcoordinate; //7
	vector<double> readerTime;	//8
	vector<double> windowsTime; //9
	vector<double> myTen;		//10
	vector<int> RSSI;			//11
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
struct MyVisionTotal
{
	double** AGV_Relative;
	double** AntennaL_POS;
	double** AntennaR_POS;
	double** AGV11;
	double** TAG_POS;
};
//�������ṹ�嶨��
struct cmpByValue
{
	bool operator()(const pair<int, double>& leftPair, const pair<int, double>& rightPair)
	{
		return leftPair.second < rightPair.second;
	}
};
//���ݾ��Ȳ�����ֵ
const double ep = 0.0001;
//PI
const double PI = 3.1415926535898;
//���û���õ�
const double Resolution = 0.0015;
//����
const int vLight = 300000000;
//��λ raidan/s,��׼�Ӧ���ǹ۲������ı�׼��
const double PhaseGauss = 0.1; 
//�ļ�·�������԰���Ҫ���ļ����ڹ����ļ����У���·���Ķ���
const string MyDataLeftAntenna = R"(E:\MATLAB workspace\hgdw\48\myData - leftAntenna.txt)";
const string MyDataRightAntenna = R"(E:\MATLAB workspace\hgdw\48\myData - rightAntenna.txt)";
const string OdometerVisionStartPoint = R"(E:\MATLAB workspace\hgdw\48\odometer_vision_start_point.txt)";
//�����˲�ѭ����������Ȼ��û��ѭ����ô���
const int Times = 400;
//��ȡ���������ֵ ��һ������
int mymax(const vector<int>&);
//��̬�ֲ����ɺ�������������Ϊmu����׼��Ϊsigma����̬�ֲ�
double gaussrand(const double& mu, const double& sigma);
//������룬v1��v2��������С��ͬ������
double distance(const vector<double>& v1, const vector<double>& v2);
//ʵ��k-�ٽ��㷨
vector<int> knnsearch(vector<vector<double>> testData, vector<double> targetData, int k);
//�жϵ��Ƿ�����������
vector<bool> inpolygon(const vector<double>& pointX, const vector<double>& pointY, const vector<double>& lineX, const vector<double>& lineY);
int main()
{
	//ϵͳ�����趨
	double antennaHLeft = 0;
	double antennaHRight = 0;
	double antennaHLeftError = antennaHLeft + 0;
	double antennaHRightError = antennaHRight - 0;
	double antennaAlphaLeft = 0;
	double antennaAlphaRight = 0;
	double antennaAlphaLeftError = antennaAlphaLeft + 0 / 180 * PI;
	double antennaAlphaRightError = antennaAlphaRight + 0 / 180 * PI;
	vector<double> waveLengthVar;
	vector<double> frequencyVar;
	for (double l = 920.625; l <= 924.375; l = l + 0.25)
	{
		double f = vLight / l * 0.0001;
		waveLengthVar.push_back(f);
		frequencyVar.push_back(l);
	}

	//ϵͳ���ݵ���
	//RFID�ͻ���������
	fstream fin;

	fin.open(MyDataLeftAntenna, ios::in);
	MyDataResult myDataResultLeft;
	while (1)
	{
		int n;
		string s;
		double d;
		//�����ļ���β���ٴζ��룬eof�Ż��Ϊtrue
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

	fin.open(MyDataRightAntenna, ios::in);
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

	fin.open(OdometerVisionStartPoint, ios::in);
	//�ȶ���vector��β��
	vector<int> odometerVisionStartPoint;
	{
		int OVSP;
		while (fin >> OVSP)
			odometerVisionStartPoint.push_back(OVSP);
	}
	fin.close();//�����ϸ�ڰ�

	//�������δ������ڽ������е�EPC�����ѿ�������36��
	vector<int> referenceTag_EPC_No;
	for (int i = 10; i <= 45; ++i)
		referenceTag_EPC_No.push_back(i);

	vector<string> referenceTag_EPC;
	//��һ�ֱ�����ʽ����
	for (auto x : referenceTag_EPC_No)
		referenceTag_EPC.push_back("1037-9654-FFFF-FFFF-FFFF-00" + to_string(x));

	int visionStartPoint = odometerVisionStartPoint[0];
	int odometerRightStartPoint = odometerVisionStartPoint[1];
	int odometerLeftStartPoint = odometerVisionStartPoint[2];

	//�Ӿ�����
	//ǰ�ڹ���
	//�����ļ�·��
	const char* p = R"(E:\MATLAB workspace\hgdw\0319_3.mat)";
	//��������
	const char* name[] = { "AGV_relative", "AtennaL_POS", "AtennaR_POS", "AGV11", "TAG_POS" };
	//���Ӿ������ļ�
	MATFile* pM = matOpen(p, "r");
	//��ȡ��Ϊ"my_vision_total"�ı���������matlab����һ���ṹ��
	mxArray* pS = matGetVariable(pM, "my_vision_total");
	MyVisionTotal myVisionTotal;
	//�ṹ����Ԫ�صĵ�ַ
	double*** q = &(myVisionTotal.AGV_Relative);

	//ѭ���洢
	for (int i = 0; i < sizeof(myVisionTotal) / sizeof(double**); ++i)
	{
		//�򿪶�Ӧ���������PF��ŵ��Ǹ�field��ָ��
		mxArray* pF = mxGetField(pS, 0, name[i]);
		//�����е����ݴ���һ����ʱ����
		double* temp = (double*)mxGetData(pF);
		//��ȡ���ݵ�����
		auto row = mxGetM(pF);
		//��ȡ���ݵ�����
		auto col = mxGetN(pF);
		//�½�ָ�����飬�⻹�Ǹ�����ָ�룬��������ڴ�ŵ���ÿһ�е��׵�ַ
		*(q + i) = new double* [row];
		for (int j = 0; j < row; ++j)
		{
			//�½� double ���͵�����
			*(*(q + i) + j) = new double[col];
			for (int k = 0; k < col; ++k)
				//����ʱ������ֵ�����ṹ��ı���
				*(*(*(q + i) + j) + k) = temp[k * row + j];
		}
	}
	//���ˣ������Ѱ���.mat�ļ��е��Ų�ȫ���������C++�е�myVisionTotal

	/*���ݼ��*/
	// for (int i = 0; i < sizeof(myVisionTotal) / sizeof(double **); i++)
	// {
	// 	mxArray *pF = mxGetField(pS, 0, name[i]);
	// 	auto row = mxGetM(pF);
	// 	auto col = mxGetN(pF);
	// 	for (int j = 0; j < row; j++)
	// 	{
	// 		for (int k = 0; k < col; k++)
	// 			cout << *(*(*(q + i) + j) + k) << ' ';
	// 		cout << endl;
	// 	}
	// }
	/**************/

	//_msize()�������ڻ�ȡnew�����ڴ�Ĵ�С
	//����ǰ��ֱ�������ȡ������������ע���趨����ʱvector<double>������
	vector<vector<double>> myVisionCurrent_AGV(_msize(myVisionTotal.AGV11) / sizeof(myVisionTotal.AGV11[0]), vector<double>(_msize(myVisionTotal.AGV11[0]) / sizeof(myVisionTotal.AGV11[0][0])));
	for (int i = 0; i < _msize(myVisionTotal.AGV11) / sizeof(myVisionTotal.AGV11[0]); ++i)
		for (int j = 0; j < _msize(myVisionTotal.AGV11[0]) / sizeof(myVisionTotal.AGV11[0][0]); ++j)
			myVisionCurrent_AGV[i][j] = myVisionTotal.AGV11[i][j];

	//����ת����һ��
	vector<vector<double>> referenceTag(_msize(myVisionTotal.TAG_POS[0]) / sizeof(myVisionTotal.TAG_POS[0][0]), vector<double>(_msize(myVisionTotal.TAG_POS) / sizeof(myVisionTotal.TAG_POS[0])));
	for (int i = 0; i < _msize(myVisionTotal.TAG_POS[0]) / sizeof(myVisionTotal.TAG_POS[0][0]); ++i)
		for (int j = 0; j < _msize(myVisionTotal.TAG_POS) / sizeof(myVisionTotal.TAG_POS[0]); ++j)
			referenceTag[i][j] = myVisionTotal.TAG_POS[j][i] * 100;

	//�����õ���ת��ǰ�ľ����������ǩ���Ļ����������ͺã���Ϊ��ʱ��û��ת�ã�
	int referenceTagNum = _msize(myVisionTotal.TAG_POS[0]) / sizeof(myVisionTotal.TAG_POS[0][0]);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < myVisionCurrent_AGV[0].size(); ++j)
			myVisionCurrent_AGV[i][j] *= 100;

	double myVisionRightAntennaHigh = myVisionTotal.AntennaR_POS[1][0] * 100;
	double myVisionLeftAntennaHigh = myVisionTotal.AntennaL_POS[1][0] * 100;

	//1 2 3 4 -> 3 2 1 4
	myVisionCurrent_AGV[0].swap(myVisionCurrent_AGV[2]);
	//3 2 1 4 -> 3 4 1 2
	myVisionCurrent_AGV[1].swap(myVisionCurrent_AGV[3]);
	//3 4 1 2 -> 3 1 4 2
	myVisionCurrent_AGV[1].swap(myVisionCurrent_AGV[2]);

	//�������ֱ����������������������޷�����
	for (auto& vec : referenceTag)
	{
		//ע��ÿһ��ѭ���У�vec����referenceTag�е�ĳһ�С���ѭ������Ҫִ��������
		auto temp = vec[2];
		vec[2] = vec[1];
		vec[1] = vec[0];
		vec[0] = temp;
	}

	//�����������ƶ�����������ϵ�µ�����
	//���ѷ���relative��[3][0]�����ĸ����ǽǶ�
	vector<double> antennaR_Robot(2);
	antennaR_Robot[0] = (myVisionTotal.AntennaR_POS[2][0] - myVisionTotal.AGV_Relative[2][0]) * cos(-myVisionTotal.AGV_Relative[3][0]) - (myVisionTotal.AntennaR_POS[0][0] - myVisionTotal.AGV_Relative[0][0]) * sin(-myVisionTotal.AGV_Relative[3][0]); //�������µ�x����
	antennaR_Robot[1] = (myVisionTotal.AntennaR_POS[2][0] - myVisionTotal.AGV_Relative[2][0]) * sin(-myVisionTotal.AGV_Relative[3][0]) + (myVisionTotal.AntennaR_POS[0][0] - myVisionTotal.AGV_Relative[0][0]) * cos(-myVisionTotal.AGV_Relative[3][0]); //�������µ�y����

	vector<double> antennaL_Robot(2);
	antennaL_Robot[0] = (myVisionTotal.AntennaL_POS[2][0] - myVisionTotal.AGV_Relative[2][0]) * cos(-myVisionTotal.AGV_Relative[3][0]) - (myVisionTotal.AntennaL_POS[0][0] - myVisionTotal.AGV_Relative[0][0]) * sin(-myVisionTotal.AGV_Relative[3][0]); //�������µ�x����
	antennaL_Robot[1] = (myVisionTotal.AntennaL_POS[2][0] - myVisionTotal.AGV_Relative[2][0]) * sin(-myVisionTotal.AGV_Relative[3][0]) + (myVisionTotal.AntennaL_POS[0][0] - myVisionTotal.AGV_Relative[0][0]) * cos(-myVisionTotal.AGV_Relative[3][0]); //�������µ�y����

	//���߸߶ȺͽǶ�
	//���ɶ���
	//�����Ƕ���߶�Ϊ�Σ�
	antennaHRight = sqrt(antennaR_Robot[0] * antennaR_Robot[0] + antennaR_Robot[1] * antennaR_Robot[1]) * 100;
	antennaHRightError = antennaHRight - 0;
	antennaAlphaRight = atan(abs(antennaR_Robot[0] / antennaR_Robot[1])) + PI / 2; //45 / 180 * PI
	antennaAlphaRightError = antennaAlphaRight + 0 / 180 * PI;
	antennaHLeft = sqrt(antennaL_Robot[0] * antennaL_Robot[0] + antennaL_Robot[1] * antennaL_Robot[1]) * 100;
	antennaHLeftError = antennaHLeft + 0;
	antennaAlphaLeft = atan(abs(antennaL_Robot[0] / antennaL_Robot[1])) + PI / 2; //45 / 180 * PI
	antennaAlphaLeftError = antennaAlphaLeft + 0 / 180 * PI;

	//���� - δ���

	//��ȡ�����Բ������ݷ���
	//��ȡ�������߶�ȡ��ǩ��epc����epc��ǩ�Ķ�ȡ��������ǩ������
	for (auto& x : myDataResultLeft.data.phase)
		//����λת��Ϊ����
		x = x / 180 * PI;

	//textData�д�ŵ��Ƕ�ȡ�Ĵ�����ÿһ�ζ�ȡ����epc������һ�δ���ֻ��һ����ȡ����
	vector<string> epcDataLeft(myDataResultLeft.textData.epcData.size());
	for (int i = 0; i < epcDataLeft.size(); ++i)
		epcDataLeft[i] = myDataResultLeft.textData.epcData[i];

	//����ͳ��ÿ����ǩ����ȡ�Ĵ���
	vector<int> referenceTagCountLeft(referenceTagNum);
	vector<vector<int>> referenceTagNumLeft;
	for (int i = 0; i < myDataResultLeft.data.no.size(); ++i)
		for (int j = 0; j < referenceTagNum; ++j)
			if (referenceTag_EPC[j] == epcDataLeft[i])
			{
				++referenceTagCountLeft[j];
				if (referenceTagNumLeft.size() < referenceTagCountLeft[j])
				{
					vector<int> v(referenceTagNum);
					referenceTagNumLeft.push_back(v);
				}
				// -1��Ϊ����vector�涨���ڴ���
				referenceTagNumLeft[referenceTagCountLeft[j] - 1][j] = i;
				break;
			}

	for (auto& x : myDataResultRight.data.phase)
		x = x / 180 * PI;

	vector<string> epcDataRight(myDataResultRight.textData.epcData.size());
	for (int i = 0; i < epcDataRight.size(); ++i)
		epcDataRight[i] = myDataResultRight.textData.epcData[i];

	vector<int> referenceTagCountRight(referenceTagNum);
	vector<vector<int>> referenceTagNumRight;
	for (int i = 0; i < myDataResultRight.data.no.size(); ++i)
		for (int j = 0; j < referenceTagNum; ++j)
			if (referenceTag_EPC[j] == epcDataRight[i])
			{
				referenceTagCountRight[j] = referenceTagCountRight[j] + 1;
				if (referenceTagNumRight.size() < referenceTagCountRight[j])
				{
					vector<int> v(referenceTagNum);
					referenceTagNumRight.push_back(v);
				}
				// -1��Ϊ����vector�涨���ڴ���
				referenceTagNumRight[referenceTagCountRight[j] - 1][j] = i;
				break;
			}

	//ʱ��
	vector<double> readerTimeLeft = myDataResultLeft.data.readerTime;
	vector<double> windowsTimeLeft = myDataResultLeft.data.windowsTime;
	vector<double> readerTimeRight = myDataResultRight.data.readerTime;
	vector<double> windowsTimeRight = myDataResultRight.data.windowsTime;

	//��ǩ��ȡ�ٶ�
	double tagReaderRate = myDataResultLeft.data.no.size() / (myDataResultLeft.data.readerTime.back() - myDataResultLeft.data.readerTime.front()) * 1000000;

	//��λԤ����
	double indexLb = 0.5; //0.5;%1/2 3/4 2/3
	double indexUb = 1.5; //3/2 5/4 4/3

	vector<vector<double>> phaseMeasurementAdjustLeft(mymax(referenceTagCountLeft), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountLeft[j] > 1)
		{
			phaseMeasurementAdjustLeft[0][j] = myDataResultLeft.data.phase[referenceTagNumLeft[0][j]];
			for (int i = 1; i < referenceTagCountLeft[j]; ++i)
			{
				auto dif = myDataResultLeft.data.phase[referenceTagNumLeft[i][j]] - phaseMeasurementAdjustLeft[i - 1][j];
				if ((dif > PI * indexLb) && (dif < PI * indexUb))
					phaseMeasurementAdjustLeft[i][j] = myDataResultLeft.data.phase[referenceTagNumLeft[i][j]] - PI;
				else if ((-dif > PI * indexLb) && (-dif < PI * indexUb))
					phaseMeasurementAdjustLeft[i][j] = myDataResultLeft.data.phase[referenceTagNumLeft[i][j]] + PI;
				else
					phaseMeasurementAdjustLeft[i][j] = myDataResultLeft.data.phase[referenceTagNumLeft[i][j]];
			}
		}

	if (accumulate(referenceTagCountLeft.begin(), referenceTagCountLeft.end(), 0) > 0)
		for (auto& vec : phaseMeasurementAdjustLeft)
			for (auto& x : vec)
				x = 2 * PI - x;

	vector<vector<double>> phasePsoLeft(mymax(referenceTagCountLeft), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountLeft[j] > 1)
		{
			phasePsoLeft[0][j] = phaseMeasurementAdjustLeft[0][j];
			for (int i = 1; i < referenceTagCountLeft[j]; i++)
			{
				int k = round((phaseMeasurementAdjustLeft[i][j] - phasePsoLeft[i - 1][j]) / PI);
				phasePsoLeft[i][j] = phaseMeasurementAdjustLeft[i][j] - PI * k;
			}
		}

	vector<vector<double>> phaseMeasurementAdjustRight(mymax(referenceTagCountRight), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountRight[j] > 1)
		{
			phaseMeasurementAdjustRight[0][j] = myDataResultRight.data.phase[referenceTagNumRight[0][j]];
			for (int i = 1; i < referenceTagCountRight[j]; ++i)
			{
				auto dif = myDataResultRight.data.phase[referenceTagNumRight[i][j]] - phaseMeasurementAdjustRight[i - 1][j];
				if ((dif > PI * indexLb) && (dif < PI * indexUb))
					phaseMeasurementAdjustRight[i][j] = myDataResultRight.data.phase[referenceTagNumRight[i][j]] - PI;
				else if ((-dif > PI * indexLb) && (-dif < PI * indexUb))
					phaseMeasurementAdjustRight[i][j] = myDataResultRight.data.phase[referenceTagNumRight[i][j]] + PI;
				else
					phaseMeasurementAdjustRight[i][j] = myDataResultRight.data.phase[referenceTagNumRight[i][j]];
			}
		}

	for (auto& vec : phaseMeasurementAdjustRight)
		for (auto& x : vec)
			x = 2 * PI - x;

	vector<vector<double>> phasePsoRight(mymax(referenceTagCountRight), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountRight[j] > 1)
		{
			phasePsoRight[0][j] = phaseMeasurementAdjustRight[0][j];
			for (int i = 1; i < referenceTagCountRight[j]; ++i)
			{
				auto k = round((phaseMeasurementAdjustRight[i][j] - phasePsoRight[i - 1][j]) / PI);
				phasePsoRight[i][j] = phaseMeasurementAdjustRight[i][j] - PI * k;
			}
		}

	// �����ߣ�����ʱ�����Ϣ��ͨ����ֵ���õ���λ����ʱ����ʵ���ƶ������˺����ߵ�λ��/λ��
	vector<vector<double>> trackMobileRobotRight;
	trackMobileRobotRight.push_back(myDataResultRight.data.Xcoordinate);
	trackMobileRobotRight.push_back(myDataResultRight.data.Ycoordinate);
	trackMobileRobotRight.push_back(myDataResultRight.data.Zcoordinate);

	//����̼Ƶõ��Ļ����˺����ַ�
	vector<vector<vector<double>>> trackMobileRobotRightTag(referenceTagNum, vector<vector<double>>(mymax(referenceTagCountRight), vector<double>(3)));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountRight[j] > 1)
			for (int i = 0; i < referenceTagCountRight[j]; ++i)
				for (int k = 0; k < 3; ++k)
					trackMobileRobotRightTag[j][i][k] = trackMobileRobotRight[k][referenceTagNumRight[i][j]];

	//������Ӿ���ʱ���
	vector<double> visionTimeDiff(myVisionCurrent_AGV[0].size());
	for (int i = 0; i < myVisionCurrent_AGV[0].size() - visionStartPoint + 1; ++i)
		visionTimeDiff[i + visionStartPoint - 1] = 1.0 / 120 * (i + 1);

	//ȥ��NaN
	for (int i = 0; i < myVisionCurrent_AGV[0].size();)
		if (isnan(myVisionCurrent_AGV[0][i]))
		{
			for (auto& vec : myVisionCurrent_AGV)
				vec.erase(vec.begin() + i, vec.begin() + i + 1);
			visionTimeDiff.erase(visionTimeDiff.begin() + i, visionTimeDiff.begin() + i + 1);
		}
		else
			++i;

	//�����Ӿ�����õ��ƶ������˳��������
	myVisionCurrent_AGV[2][0] = myVisionCurrent_AGV[2][0];
	for (int i = 1; i < myVisionCurrent_AGV[2].size(); ++i)
	{
		auto k = round((myVisionCurrent_AGV[2][i] - myVisionCurrent_AGV[2][i - 1]) / PI);
		myVisionCurrent_AGV[2][i] = myVisionCurrent_AGV[2][i] - PI * k;
	}

	//�������̼Ƶ�ʱ���
	vector<double> odometerTimeDiff(readerTimeRight.size() - odometerRightStartPoint + 1);
	for (int i = 0; i < odometerTimeDiff.size(); ++i)
		odometerTimeDiff[i] = (readerTimeRight[odometerRightStartPoint - 1 + i] - readerTimeRight[odometerRightStartPoint - 1]) / 1000000;

	//ͨ����ֵ������ƶ�������λ�˵�
	vector<vector<double>> mobileRobotPoseVisionRight(odometerTimeDiff.size() + odometerRightStartPoint - 1, vector<double>(4));
	for (int i = 0; i < odometerRightStartPoint - 1; ++i)
		for (int j = 0; j < myVisionCurrent_AGV.size(); ++j)
			mobileRobotPoseVisionRight[i][j] = myVisionCurrent_AGV[j][0];
	for (int j = 0; j < odometerTimeDiff.size(); ++j)
	{
		int pointPre = 0;
		int pointPso = 0;
		bool preFlag = false;
		bool psoFlag = false;
		//���ҵ� ��̼Ƶ�ʱ�� �� �Ӿ�ʱ�������� �� ǰ��������� ��λ�˵�
		for (auto it = visionTimeDiff.begin(); it != visionTimeDiff.end(); ++it)
		{
			if ((*it) >= odometerTimeDiff[j] && !psoFlag)
			{
				pointPso = it - visionTimeDiff.begin();
				psoFlag = true;
			}
			if (psoFlag)
				break;
		}
		for (auto it = visionTimeDiff.end() - 1; it >= visionTimeDiff.begin(); --it)
		{
			if (((*it) <= odometerTimeDiff[j]) && !preFlag)
			{
				pointPre = it - visionTimeDiff.begin();
				preFlag = true;
			}
			if (preFlag)
				break;
		}
		if (psoFlag)
			for (int i = 0; i < 4; ++i)
				mobileRobotPoseVisionRight[j + odometerRightStartPoint - 1][i] = (odometerTimeDiff[j] - visionTimeDiff[pointPso]) / (visionTimeDiff[pointPre] - visionTimeDiff[pointPso]) * myVisionCurrent_AGV[i][pointPre] + (odometerTimeDiff[j] - visionTimeDiff[pointPre]) / (visionTimeDiff[pointPso] - visionTimeDiff[pointPre]) * myVisionCurrent_AGV[i][pointPso];
	}

	//�����ߣ���������ֵ�õ��Ļ�����ʵ��·�����õ����۵���λ����
	vector<vector<vector<double>>> trackMobileRobotRightAntennaReal(referenceTagNum, vector<vector<double>>(mymax(referenceTagCountRight), vector<double>(2)));
	vector<vector<double>> distanceTheory(mymax(referenceTagCountRight), vector<double>(referenceTagNum));
	vector<vector<double>> phaseTagTheoryRight(mymax(referenceTagCountRight), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountRight[j] > 1)
			for (int i = 0; i < referenceTagCountRight[j]; ++i)
			{
				trackMobileRobotRightAntennaReal[j][i][0] = mobileRobotPoseVisionRight[referenceTagNumRight[i][j]][0] + antennaHRightError * cos(-antennaAlphaRightError + mobileRobotPoseVisionRight[referenceTagNumRight[i][j]][2]);
				trackMobileRobotRightAntennaReal[j][i][1] = mobileRobotPoseVisionRight[referenceTagNumRight[i][j]][1] + antennaHRightError * sin(-antennaAlphaRightError + mobileRobotPoseVisionRight[referenceTagNumRight[i][j]][2]);
				distanceTheory[i][j] = sqrt(pow((trackMobileRobotRightAntennaReal[j][i][0] - referenceTag[j][0]), 2) + pow((trackMobileRobotRightAntennaReal[j][i][1] - referenceTag[j][1]), 2) + pow((myVisionRightAntennaHigh - referenceTag[j][2]), 2)) * 2;
				phaseTagTheoryRight[i][j] = distanceTheory[i][j] * 2 * PI / waveLengthVar[0];
			}

	//�����ߣ��ź�ǿ�ȷ���
	vector<vector<int>> RSSI_TagRight(mymax(referenceTagCountRight), vector<int>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountRight[j] > 1)
			for (int i = 0; i < referenceTagCountRight[j]; ++i)
				RSSI_TagRight[i][j] = myDataResultRight.data.RSSI[referenceTagNumRight[i][j]];

	//�����ߣ�����ʱ�����Ϣ��ͨ����ֵ���õ���λ��³ʱ����ʵ���ƶ������˺����ߵ�λ��/λ��
	vector<vector<double>> trackMobileRobotLeft;
	trackMobileRobotLeft.push_back(myDataResultLeft.data.Xcoordinate);
	trackMobileRobotLeft.push_back(myDataResultLeft.data.Ycoordinate);
	trackMobileRobotLeft.push_back(myDataResultLeft.data.Zcoordinate);

	//��̼Ƶõ��Ļ����˺����ַ�
	vector<vector<vector<double>>> trackMobileRobotLeftTag(referenceTagNum, vector<vector<double>>(mymax(referenceTagCountLeft), vector<double>(3)));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountLeft[j] > 1)
			for (int i = 0; i < referenceTagCountLeft[j]; ++i)
				for (int k = 0; k < 3; ++k)
					trackMobileRobotLeftTag[j][i][k] = trackMobileRobotLeft[k][referenceTagNumLeft[i][j]];

	//�������̼Ƶ�ʱ���
	odometerTimeDiff.clear();
	odometerTimeDiff.resize(readerTimeLeft.size() - odometerLeftStartPoint + 1);
	for (int i = 0; i < odometerTimeDiff.size(); ++i)
		odometerTimeDiff[i] = (readerTimeLeft[odometerLeftStartPoint - 1 + i] - readerTimeLeft[odometerLeftStartPoint - 1]) / 1000000;

	//ͨ����ֵ������ƶ�������λ�˵�
	vector<vector<double>> mobileRobotPoseVisionLeft(odometerTimeDiff.size() + odometerLeftStartPoint - 1, vector<double>(4));
	for (int i = 0; i < odometerLeftStartPoint - 1; ++i)
		for (int j = 0; j < myVisionCurrent_AGV.size(); ++j)
			mobileRobotPoseVisionLeft[i][j] = myVisionCurrent_AGV[j][0];
	for (int j = 0; j < odometerTimeDiff.size(); ++j)
	{
		int pointPre = 0;
		int pointPso = 0;
		bool preFlag = false;
		bool psoFlag = false;
		//���ҵ� ��̼Ƶ�ʱ�� �� �Ӿ�ʱ�������� �� ǰ��������� ��λ�˵�
		for (auto it = visionTimeDiff.begin(); it != visionTimeDiff.end(); ++it)
		{
			if ((*it) >= odometerTimeDiff[j] && !psoFlag)
			{
				pointPso = it - visionTimeDiff.begin();
				psoFlag = true;
			}
			if (psoFlag)
				break;
		}
		for (auto it = visionTimeDiff.end() - 1; it != visionTimeDiff.begin(); --it)
		{
			if ((*it <= odometerTimeDiff[j]) && !preFlag)
			{
				pointPre = it - visionTimeDiff.begin();
				preFlag = true;
			}
			if (preFlag)
				break;
		}
		if (psoFlag)
			for (int i = 0; i < 4; ++i)
				mobileRobotPoseVisionLeft[j + odometerLeftStartPoint - 1][i] = (odometerTimeDiff[j] - visionTimeDiff[pointPso]) / (visionTimeDiff[pointPre] - visionTimeDiff[pointPso]) * myVisionCurrent_AGV[i][pointPre] + (odometerTimeDiff[j] - visionTimeDiff[pointPre]) / (visionTimeDiff[pointPso] - visionTimeDiff[pointPre]) * myVisionCurrent_AGV[i][pointPso];
	}

	//�����ߣ���������ֵ�õ��Ļ�����ʵ��·�����õ����۵���λ����
	vector<vector<vector<double>>> trackMobileRobotLeftAntennaReal(referenceTagNum, vector<vector<double>>(mymax(referenceTagCountLeft), vector<double>(2)));

	distanceTheory.clear();
	distanceTheory.resize(mymax(referenceTagCountLeft));
	for (auto& vec : distanceTheory)
		vec.resize(referenceTagNum);

	vector<vector<double>> phaseTagTheoryLeft(mymax(referenceTagCountLeft), vector<double>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountLeft[j] > 1)
			for (int i = 0; i < referenceTagCountLeft[j]; ++i)
			{
				trackMobileRobotLeftAntennaReal[j][i][0] = mobileRobotPoseVisionLeft[referenceTagNumLeft[i][j]][0] - antennaHLeftError * cos(PI - antennaAlphaLeftError - mobileRobotPoseVisionLeft[referenceTagNumLeft[i][j]][2]);
				trackMobileRobotLeftAntennaReal[j][i][1] = mobileRobotPoseVisionLeft[referenceTagNumLeft[i][j]][1] + antennaHLeftError * sin(PI - antennaAlphaLeftError - mobileRobotPoseVisionLeft[referenceTagNumLeft[i][j]][2]);
				distanceTheory[i][j] = sqrt(pow((trackMobileRobotLeftAntennaReal[j][i][0] - referenceTag[j][0]), 2) + pow((trackMobileRobotLeftAntennaReal[j][i][1] - referenceTag[j][1]), 2) + pow((myVisionLeftAntennaHigh - referenceTag[j][2]), 2)) * 2;
				phaseTagTheoryLeft[i][j] = distanceTheory[i][j] * 2 * PI / waveLengthVar[0];
			}

	//�����ߣ��ź�ǿ�ȷ���
	vector<vector<int>> RSSI_TagLeft(mymax(referenceTagCountLeft), vector<int>(referenceTagNum));
	for (int j = 0; j < referenceTagNum; ++j)
		if (referenceTagCountLeft[j] > 1)
			for (int i = 0; i < referenceTagCountLeft[j]; ++i)
				RSSI_TagLeft[i][j] = myDataResultLeft.data.RSSI[referenceTagNumLeft[i][j]];

	//ִ�������˲���λ�㷨
	int PF_Count = 500;

	//����״̬���ƶ�������λ�ˣ�����һ����������Ϊ�����˵�x y thֵ
	vector<vector<vector<double>>> PF_ParticleRobot(500, vector<vector<double>>(3, vector<double>(PF_Count)));
	//����״̬���任���󣩣��ֱ��Ӧ����ƽ�ơ�һ����ת
	vector<vector<vector<double>>> PF_ParticleTransformation(500, vector<vector<double>>(3, vector<double>(PF_Count)));
	//����״̬��������λ�ã�����һ������Ϊ�����ߵ�x yֵ
	vector<vector<vector<double>>> PF_ParticleAntennaLeft(20000, vector<vector<double>>(2, vector<double>(PF_Count)));
	//����״̬��������λ�ã�����һ������Ϊ�����ߵ�x yֵ
	vector<vector<vector<double>>> PF_ParticleAntennaRight(20000, vector<vector<double>>(2, vector<double>(PF_Count)));

	//�۲⣺��������������ʱ��֮�����λ��ֵ,��������
	vector<double> PF_ObserveGradientLeft(referenceTag.size());
	//�۲⣺��������������ʱ��֮�����λ��ֵ,��������
	vector<double> PF_ObserveGradientRight(referenceTag.size());

	//Ԥ�⣺��1����������ǰһ��ʱ�̵����λ�������λ������2���������ߵ�ǰʱ�̵����λ�������λ������3Ϊ����������ʱ��֮�����λ��ֵ
	vector<vector<vector<double>>> PF_PredictionLeft(referenceTag.size(), vector<vector<double>>(3, vector<double>(PF_Count)));
	//Ԥ�⣺��1����������ǰһ��ʱ�̵����λ�������λ������2���������ߵ�ǰʱ�̵����λ�������λ������3Ϊ����������ʱ��֮�����λ��ֵ
	vector<vector<vector<double>>> PF_PredictionRight(referenceTag.size(), vector<vector<double>>(3, vector<double>(PF_Count)));

	//��λcm
	int gradientLen = 2;
	//��λcm
	int distanceFarThreshold = 200;
	//ʱ�����ƣ���λs
	int gradientTimeLen = 10;
	//������������׼���λcm, rad
	vector<vector<double>> PF_Q = { {3, 0}, {0, 0.1} };
	//���������������λrad
	double PF_R = (PhaseGauss + 0.4) * (PhaseGauss + 0.4);
	//������۲�ֵ֮��Ĳ��,ÿһ�ж�Ӧһ���ο���ǩ
	vector<vector<double>> PF_DistanceLeft(referenceTag.size(), vector<double>(PF_Count));
	//������۲�ֵ֮��Ĳ��,ÿһ�ж�Ӧһ���ο���ǩ
	vector<vector<double>> PF_DistanceRight(referenceTag.size(), vector<double>(PF_Count));
	//������۲�ֵ֮���Ȩ�����ӣ������������ۺ�����
	vector<vector<double>> PF_W_Left(referenceTag.size(), vector<double>(PF_Count));
	//������۲�ֵ֮���Ȩ�����ӣ������������ۺ�����
	vector<vector<double>> PF_W_Right(referenceTag.size(), vector<double>(PF_Count));
	//��һ�У����ι۲��Ȩ�أ��ڶ��У��������յ�Ȩ��
	vector<vector<vector<double>>> PF_W(Times, vector<vector<double>>(2, vector<double>(PF_Count)));
	//Э�������
	vector<vector<vector<double>>> PF_CenterVar(200, vector<vector<double>>(3, vector<double>(3)));
	double neffRatioThreshold = 0.6;

	//��ǩ�ɶ��Ե�����ж�ֵ
	//���ɶ��İ뾶��һ�����ڴ�ֵ
	int tagUnreadableRadius = 20;
	//�ɶ��İ뾶��һ��С�ڴ�ֵ
	int tagReadableRadius = 110;

	//�ƶ������˱�����ڵ�����
	vector<vector<double>> leftShadowUnreadablePointCor = { {120, 20}, {-120, 20}, {-120, -120}, {120, -120} };

	vector<double> leftShadowUnreadablePointH(leftShadowUnreadablePointCor.size());
	for (int i = 0; i < leftShadowUnreadablePointH.size(); ++i)
		leftShadowUnreadablePointH[i] = sqrt(pow(leftShadowUnreadablePointCor[i][0], 2) + pow(leftShadowUnreadablePointCor[i][1], 2));

	vector<double> leftShadowUnreadablePointAlpha(leftShadowUnreadablePointCor.size());
	for (int i = 0; i < leftShadowUnreadablePointAlpha.size(); ++i)
		leftShadowUnreadablePointAlpha[i] = atan2(leftShadowUnreadablePointCor[i][1], leftShadowUnreadablePointCor[i][0]);

	vector<vector<double>> leftShadowReadablePointCor = { {120, -20}, {-120, -20}, {-120, -120}, {120, -120} };

	vector<double> leftShadowReadablePointH(leftShadowReadablePointCor.size());
	for (int i = 0; i < leftShadowReadablePointH.size(); ++i)
		leftShadowReadablePointH[i] = sqrt(pow(leftShadowReadablePointCor[i][0], 2) + pow(leftShadowReadablePointCor[i][1], 2));

	vector<double> leftShadowReadablePointAlpha(leftShadowReadablePointCor.size());
	for (int i = 0; i < leftShadowReadablePointAlpha.size(); ++i)
		leftShadowReadablePointAlpha[i] = atan2(leftShadowReadablePointCor[i][1], leftShadowReadablePointCor[i][0]);

	vector<vector<double>> rightShadowUnreadablePointCor = { {120, -20}, {-120, -20}, {-120, 120}, {120, 120} };

	vector<double> rightShadowUnreadablePointH(rightShadowUnreadablePointCor.size());
	for (int i = 0; i < rightShadowUnreadablePointH.size(); ++i)
		rightShadowUnreadablePointH[i] = sqrt(pow(rightShadowUnreadablePointCor[i][0], 2) + pow(rightShadowUnreadablePointCor[i][1], 2));

	vector<double> rightShadowUnreadablePointAlpha(rightShadowUnreadablePointCor.size());
	for (int i = 0; i < rightShadowUnreadablePointAlpha.size(); ++i)
		rightShadowUnreadablePointAlpha[i] = atan2(rightShadowUnreadablePointCor[i][1], rightShadowUnreadablePointCor[i][0]);

	vector<vector<double>> rightShadowReadablePointCor = { {120, 20}, {-120, 20}, {-120, 120}, {120, 120} };

	vector<double> rightShadowReadablePointH(rightShadowReadablePointCor.size());
	for (int i = 0; i < rightShadowReadablePointH.size(); ++i)
		rightShadowReadablePointH[i] = sqrt(pow(rightShadowReadablePointCor[i][0], 2) + pow(rightShadowReadablePointCor[i][1], 2));

	vector<double> rightShadowReadablePointAlpha(rightShadowReadablePointCor.size());
	for (int i = 0; i < rightShadowReadablePointAlpha.size(); ++i)
		rightShadowReadablePointAlpha[i] = atan2(rightShadowReadablePointCor[i][1], rightShadowReadablePointCor[i][0]);

	//�����ӳ�ʼ���У�PF_tag_readable_count
	int PF_TagReadableCountInit = 5;
	//�������˲������У���ǩ�ɶ�����Ҫ�жϵ���Χ�ı�ǩ����
	int PF_TagReadableCount = 5;
	//�������˲��У����ݶγ��ȵ��½�
	int distanceIntervalLp = 1;

	//Ϊ�˽����ز�����Ƶ��
	double recoveryAlphaSlow = 0.001; //0.05
	double recoveryAlphaFast = 0.2;	  //0.2
	int PF_W_Slow = 0;
	int PF_W_Fast = 0;

	//�����ߵı�ţ�Ϊ���������ʼ�һ
	int numberFlag = 0;
	//�����ߵı�ţ�Ϊ���������ʼ�һ
	int numberFlagVice = 0;

	vector<vector<double>> PF_ObserveNativeLeft = phasePsoLeft;
	vector<vector<double>> PF_ObserveNativeRight = phasePsoRight;

	vector<double> robotXtAssume(Times);
	vector<double> robotYtAssume(Times);
	vector<double> robotThtAssume(Times);
	vector<double> robotXt(Times);
	vector<double> robotYt(Times);
	vector<double> robotTht(Times);

	vector<vector<int>> numberFlagRight(2, vector<int>(referenceTagNum));
	vector<vector<int>> numberFlagLeft(2, vector<int>(referenceTagNum));
	vector<vector<double>> antennaLeft(2, vector<double>(Times));
	vector<vector<double>> antennaRight(2, vector<double>(Times));
	vector<vector<double>> robotEstimationError(5, vector<double>(Times));
	vector<int> readableTagRightNum(Times);
	vector<int> readableTagLeftNum(Times);

	vector<vector<double>> PF_CenterMean(4, vector<double>(Times));
	vector<vector<double>> robotPositionAssume(3, vector<double>(Times));
	vector<vector<int>> PF_W_RSSI(Times, vector<int>(PF_Count));
	vector<vector<vector<double>>> PF_ParticleAntennaRightPso;
	vector<vector<vector<double>>> PF_ParticleAntennaRightPre;
	vector<vector<vector<double>>> PF_ParticleAntennaLeftPso;
	vector<vector<vector<double>>> PF_ParticleAntennaLeftPre;
	vector<bool> phaseFlag(Times);
	vector<double> neffRatioPre(Times);
	vector<int> PF_ReSample(Times);
	vector<double> time(Times);

	/*��֤���ݵ��룺�����*/
	//vector<double> randNum(Times);
	//vector<vector<vector<double>>> ran(Times, vector<vector<double>>(3, vector<double>(PF_Count)));
	//MATFile* pM0 = matOpen("E:/MATLAB workspace/hgdw/test400.mat", "r");
	//mxArray* ppV1 = matGetVariable(pM0, "ran");
	//double* temp1 = (double*)mxGetData(ppV1);
	//mxArray* ppV2 = matGetVariable(pM0, "randnum");
	//double* temp2 = (double*)mxGetData(ppV2);
	//for (int i = 0; i < Times; ++i)
	//{
	//	for (int j = 0; j < 3; ++j)
	//	{
	//		for (int k = 0; k < 500; ++k)
	//		{
	//			ran[i][j][k] = temp1[i * 500 * 3 + k * 3 + j];
	//		}
	//	}
	//	randNum[i] = temp2[i];
	//}
	/*******���******/

	for (int i = 0; i < Times; ++i)
	{
		auto startTime = clock();
		if (i == 0)
		{
			robotXtAssume[i] = trackMobileRobotRight[0][numberFlag];
			robotYtAssume[i] = trackMobileRobotRight[1][numberFlag];
			robotThtAssume[i] = trackMobileRobotRight[2][numberFlag];

			robotPositionAssume[0][i] = robotXtAssume[i];
			robotPositionAssume[1][i] = robotYtAssume[i];
			robotPositionAssume[2][i] = 1;

			//�ҵ���ֹ�����£����ܹ������ı�ǩ����������Щ��ǩ�����ֱ꣬�Ӷ����ӽ��г�ʼ��
			//������
			epcDataRight.clear();
			epcDataRight.resize(odometerRightStartPoint);
			for (int j = 0; j < odometerRightStartPoint; ++j)
				epcDataRight[j] = myDataResultRight.textData.epcData[j];

			//�������
			vector<double> RSSI_DataRightCurrent(odometerRightStartPoint);
			for (int j = 0; j < odometerRightStartPoint; ++j)
				RSSI_DataRightCurrent[j] = myDataResultRight.data.RSSI[j];

			vector<double> referenceTagCountRightCurrent(referenceTagNum);
			for (int j = 0; j < odometerRightStartPoint; ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					if ((referenceTag_EPC[k] == epcDataRight[j]) && (RSSI_DataRightCurrent[j] >= -53))
					{
						//������1
						++referenceTagCountRightCurrent[k];
						break;
					}

			//ͳ��referenceTagCountRightCurrent�з���Ԫ�ظ���
			int count = 0;
			for (auto x : referenceTagCountRightCurrent)
				if (x)
					++count;
			readableTagRightNum[i] = count;

			//�������Ǳ�ǩ��λ��
			vector<int> colRight;
			for (auto it = referenceTagCountRightCurrent.begin(); it != referenceTagCountRightCurrent.end(); ++it)
				if (*it)
					colRight.push_back(it - referenceTagCountRightCurrent.begin());

			//������
			epcDataLeft.clear();
			epcDataLeft.resize(odometerLeftStartPoint);
			for (int j = 0; j < odometerLeftStartPoint; ++j)
				epcDataLeft[j] = myDataResultLeft.textData.epcData[j];

			//�������
			vector<double> RSSI_DataLeftCurrent(odometerLeftStartPoint);
			for (int j = 0; j < odometerLeftStartPoint; ++j)
				RSSI_DataLeftCurrent[j] = myDataResultLeft.data.RSSI[j];

			vector<double> referenceTagCountLeftCurrent(referenceTagNum);
			for (int j = 0; j < odometerLeftStartPoint; ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					if ((referenceTag_EPC[k] == epcDataLeft[j]) && (RSSI_DataLeftCurrent[j] >= -53))
					{
						++referenceTagCountLeftCurrent[k];
						break;
					}

			//�������Ǳ�ǩ��λ��
			vector<int> colLeft;
			for (auto it = referenceTagCountLeftCurrent.begin(); it != referenceTagCountLeftCurrent.end(); ++it)
				if (*it)
					colLeft.push_back(it - referenceTagCountLeftCurrent.begin());

			//��΢�е㷱����
			vector<int> randData(4);
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

			//���ݸ÷�Χ����������
			for (int j = 0; j < PF_Count; ++j)
			{
				//����x����
				PF_ParticleRobot[i][0][j] = PF_Scope[0][0] + rand() / double(RAND_MAX) * (PF_Scope[0][1] - PF_Scope[0][0]);
				//����y����
				PF_ParticleRobot[i][1][j] = PF_Scope[1][0] + rand() / double(RAND_MAX) * (PF_Scope[1][1] - PF_Scope[1][0]);
				//���ӷ���
				PF_ParticleRobot[i][2][j] = (-PI) + rand() / double(RAND_MAX) * (PI - (-PI));
			}

			/*��֤���ݵ���*/
			//for (int j = 0; j < PF_Count; ++j)
			//{
			//	//����x����
			//	PF_ParticleRobot[i][0][j] = PF_Scope[0][0] + (PF_Scope[0][1] - PF_Scope[0][0]) * ran[i][0][j];
			//	//����y����
			//	PF_ParticleRobot[i][1][j] = PF_Scope[1][0] + (PF_Scope[1][1] - PF_Scope[1][0]) * ran[i][1][j];
			//	//���ӷ���
			//	PF_ParticleRobot[i][2][j] = (-PI) + (PI - (-PI)) * ran[i][2][j];
			//}
			/********��֤********/

			//������λ������Ӧ�����ߵ�λ��
			for (int j = 0; j < PF_Count; ++j)
			{
				PF_ParticleAntennaLeft[i][0][j] = PF_ParticleRobot[i][0][j] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobot[i][2][j]);
				PF_ParticleAntennaLeft[i][1][j] = PF_ParticleRobot[i][1][j] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobot[i][2][j]);
			}
			for (int j = 0; j < PF_Count; ++j)
			{
				PF_ParticleAntennaRight[i][0][j] = PF_ParticleRobot[i][0][j] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobot[i][2][j]);
				PF_ParticleAntennaRight[i][1][j] = PF_ParticleRobot[i][1][j] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobot[i][2][j]);
			}

			for (int j = 0; j < PF_Count; ++j)
				PF_W[i][1][j] = 1 / double(PF_Count);

			//x y����ľ�ֵ����Э����
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					PF_CenterMean[j][i] += PF_ParticleRobot[i][j][k] * PF_W[i][1][k];
			
			double weightSqr = 0;
			for (int j = 0; j < PF_Count; ++j)
				weightSqr += PF_W[i][1][j] * PF_W[i][1][j];
			double factor = 0;
			if (abs(weightSqr - 1.0) < sqrt(DBL_EPSILON))
				factor = 1.0;
			else
				factor = 1 / (1 - weightSqr);

			vector<vector<double>> meanDiff(2, vector<double>(PF_Count));
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					meanDiff[j][k] = PF_ParticleRobot[i][j][k] - PF_CenterMean[j][i];

			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < 2; ++k)
				{
					for (int m = 0; m < PF_Count; ++m)
						PF_CenterVar[i][j][k] += meanDiff[j][m] * PF_W[i][1][m] * meanDiff[k][m];
					PF_CenterVar[i][j][k] *= factor;
				}

			//�ǶȾ�ֵ��Э���
			double sinsum = 0;
			for (int j = 0; j < PF_Count; ++j)
				sinsum += PF_W[i][1][j] * sin(PF_ParticleRobot[i][2][j]);

			double cossum = 0;
			for (int j = 0; j < PF_Count; ++j)
				cossum += PF_W[i][1][j] * cos(PF_ParticleRobot[i][2][j]);

			double resultantLength = sqrt(pow(sinsum, 2) + pow(cossum, 2));
			PF_CenterMean[2][i] = atan2(sinsum, cossum);
			PF_CenterVar[i][2][2] = -2 * log(resultantLength);
			for (int j = 0; j < PF_Count; ++j)
				PF_CenterMean[3][i] += PF_ParticleRobot[i][2][j] * PF_W[i][1][j];

			//��λ���
			robotXt[i] = mobileRobotPoseVisionRight[numberFlag][0];
			robotYt[i] = mobileRobotPoseVisionRight[numberFlag][1];
			robotTht[i] = mobileRobotPoseVisionRight[numberFlag][2];

			antennaLeft[0][i] = robotXt[i] - antennaHLeftError * cos(PI - antennaAlphaLeftError - robotTht[i]);
			antennaLeft[1][i] = robotYt[i] + antennaHLeftError * sin(PI - antennaAlphaLeftError - robotTht[i]);
			antennaRight[0][i] = robotXt[i] + antennaHRightError * cos(-antennaAlphaRightError + robotTht[i]);
			antennaRight[1][i] = robotYt[i] + antennaHRightError * sin(-antennaAlphaRightError + robotTht[i]);

			robotEstimationError[0][i] = PF_CenterMean[0][i] - robotXt[i];
			robotEstimationError[1][i] = PF_CenterMean[1][i] - robotYt[i];
			robotEstimationError[2][i] = sqrt(pow(robotEstimationError[0][i], 2) + pow(robotEstimationError[1][i], 2));

			robotEstimationError[3][i] = PF_CenterMean[2][i] - robotTht[i];

			robotEstimationError[4][i] = PF_CenterMean[3][i] - robotTht[i];

			numberFlagRight[0].assign(referenceTagNum, -1);
			numberFlagRight[1].assign(referenceTagNum, -1);

			//��ͼ
			// 	 figure
			// h1 = plot(reference_tag(:,1), reference_tag(:,2), 'k.', 'markersize',25);  %ϵͳ״̬λ��
			// hold on
			// h2 = plot(robot_xt(1:i),robot_yt(1:i),'r.', 'markersize',50);%�����˵�λ��
			// h3 = plot(PF_particle_robot(1, :, i), PF_particle_robot(2, :, i),'.','markersize',10);
			// h4 = quiver(robot_xt(1:i),robot_yt(1:i), cos(robot_tht(i)), sin(robot_tht(i)),15, 'color', 'r', 'linewidth', 3);
			// h5 = quiver(PF_particle_robot(1, :, i), PF_particle_robot(2, :, i), cos(PF_particle_robot(3, :, i)), sin(PF_particle_robot(3, :, i)),0.3, 'color', 'c', 'linewidth', 3);
			// legend('�ο���ǩ����', '�ƶ���������ʵλ��','�ƶ������˿���λ��');
			// set(gca,'child',[h2 h4 h3 h5 h1])
			// xlabel('x (cm)');ylabel('y (cm)');
			// legend('�ο���ǩ����', '�ƶ���������ʵλ��','�ƶ������˿���λ��');

			// figure
			// h1 = plot(reference_tag(:,1), reference_tag(:,2), 'k.', 'marker', 'pentagram', 'markersize', 10);  %ϵͳ״̬λ��
			// hold on
			// h2 = plot(robot_xt(1:1),robot_yt(1:1),'r.', 'markersize',35);%�����˵�λ��
			// h3 = plot(PF_particle_robot(1, :, 1), PF_particle_robot(2, :, 1),'.','markersize',15);
			// h4 = quiver(robot_xt(1:1),robot_yt(1:1), cos(robot_tht(1)), sin(robot_tht(1)),3, 'color', 'r', 'linewidth', 3,'MaxHeadSize',5);
			// h5 = quiver(PF_particle_robot(1, :, 1), PF_particle_robot(2, :, 1), cos(PF_particle_robot(3, :, 1)), sin(PF_particle_robot(3, :, 1)),0.3, 'color', 'c', 'linewidth', 3);
			// legend('Reference tag', 'Ture pose of mobile robot','Distribution of particle');

			// set(gca,'child',[h2 h4 h3 h5 h1]);
			// xlabel('X (cm)');
			// ylabel('Y (cm)');
			// axis([-170 -110 -55 -00]);
		}
		else
		{
			//cout << i << endl;
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
					//������ǰѭ��
					break;
			}
			//������ѭ��
			if (numberFlag >= myDataResultRight.data.readerTime.size())
				break;

			robotXtAssume[i] = trackMobileRobotRight[0][numberFlag];
			robotYtAssume[i] = trackMobileRobotRight[1][numberFlag];
			robotThtAssume[i] = trackMobileRobotRight[2][numberFlag];

			robotPositionAssume[0][i] = robotXtAssume[i];
			robotPositionAssume[1][i] = robotYtAssume[i];
			robotPositionAssume[2][i] = 1;

			//״̬ת�ƣ�Ԥ��
			double robotXtDiff = robotXtAssume[i] - robotXtAssume[i - 1];
			double robotYtDiff = robotYtAssume[i] - robotYtAssume[i - 1];
			double robotThtDiff = robotThtAssume[i] - robotThtAssume[i - 1];

			vector<double> positionDifference1(2);
			positionDifference1[0] = robotXtDiff * cos(-robotThtAssume[i - 1]) - robotYtDiff * sin(-robotThtAssume[i - 1]);
			positionDifference1[1] = robotXtDiff * sin(-robotThtAssume[i - 1]) + robotYtDiff * cos(-robotThtAssume[i - 1]);

			vector<vector<double>> positionDifference2(2, vector<double>(PF_Count));
			//����д�ȽϽ���������
			for (int j = 0; j < PF_Count; ++j)
			{
				positionDifference2[0][j] = positionDifference1[0] * cos(PF_ParticleRobot[i - 1][2][j]) - positionDifference1[1] * sin(PF_ParticleRobot[i - 1][2][j]);
				positionDifference2[1][j] = positionDifference1[0] * sin(PF_ParticleRobot[i - 1][2][j]) + positionDifference1[1] * cos(PF_ParticleRobot[i - 1][2][j]);
			}

			for (int j = 0; j < PF_Count; ++j)
			{
				PF_ParticleRobot[i][0][j] = PF_ParticleRobot[i - 1][0][j] + positionDifference2[0][j] + gaussrand(0, PF_Q[0][0]);
				PF_ParticleRobot[i][1][j] = PF_ParticleRobot[i - 1][1][j] + positionDifference2[1][j] + gaussrand(0, PF_Q[0][0]);
				PF_ParticleRobot[i][2][j] = PF_ParticleRobot[i - 1][2][j] + robotThtDiff + gaussrand(0, PF_Q[1][1]);
			}

			/*��֤���ݵ���*/
			//for (int j = 0; j < PF_Count; ++j)
			//{
			//	//����x����
			//	PF_ParticleRobot[i][0][j] = PF_ParticleRobot[i - 1][0][j] + positionDifference2[0][j] + ran[i][0][j];
			//	//����y����
			//	PF_ParticleRobot[i][1][j] = PF_ParticleRobot[i - 1][1][j] + positionDifference2[1][j] + ran[i][1][j];
			//	//���ӷ���
			//	PF_ParticleRobot[i][2][j] = PF_ParticleRobot[i - 1][2][j] + robotThtDiff + ran[i][2][j];
			//}
			/********��֤********/

			//������
			epcDataRight.clear();
			epcDataRight.resize(numberFlag - numberFlagPre);
			for (int j = 0; j < (numberFlag - numberFlagPre); ++j)
				epcDataRight[j] = myDataResultRight.textData.epcData[numberFlagPre + 1 + j];

			//�������
			vector<double> RSSI_DataRightCurrent(numberFlag - numberFlagPre);
			for (int j = 0; j < (numberFlag - numberFlagPre); ++j)
				RSSI_DataRightCurrent[j] = myDataResultRight.data.RSSI[numberFlagPre + 1 + j];

			vector<double> referenceTagCountRightCurrent(referenceTagNum);
			for (int j = 0; j < (numberFlag - numberFlagPre); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					if ((referenceTag_EPC[k] == epcDataRight[j]) && (RSSI_DataRightCurrent[j] >= -52))
					{
						//������1
						++referenceTagCountRightCurrent[k];
						break;
					}

			//ͳ��referenceTagCountRightCurrent�з���Ԫ�ظ���
			int count = 0;
			for (auto x : referenceTagCountRightCurrent)
				if (x)
					++count;
			readableTagRightNum[i] = count;

			//������
			int numberFlagVicePre = numberFlagVice;

			//Ѱ������������������������0��Ȼ��Ѱ�����ֵ��
			int m;
			for (auto it = myDataResultLeft.data.readerTime.begin(); it != myDataResultLeft.data.readerTime.end(); ++it)
				if ((*it) < myDataResultRight.data.readerTime[numberFlag])
					m = it - myDataResultLeft.data.readerTime.begin();
			numberFlagVice = (m > 0) ? m : 0;

			//������
			epcDataLeft.clear();
			epcDataLeft.resize(numberFlagVice - numberFlagVicePre);
			for (int j = 0; j < (numberFlagVice - numberFlagVicePre); ++j)
				epcDataLeft[j] = myDataResultLeft.textData.epcData[numberFlagVicePre + 1 + j];

			//�������
			vector<double> RSSI_DataLeftCurrent(numberFlagVice - numberFlagVicePre);
			for (int j = 0; j < (numberFlagVice - numberFlagVicePre); ++j)
				RSSI_DataLeftCurrent[j] = myDataResultLeft.data.RSSI[numberFlagVicePre + 1 + j];

			vector<double> referenceTagCountLeftCurrent(referenceTagNum);
			for (int j = 0; j < (numberFlagVice - numberFlagVicePre); ++j)
				for (int k = 0; k < referenceTagNum; ++k)
					if ((referenceTag_EPC[k] == epcDataLeft[j]) && (RSSI_DataLeftCurrent[j] >= -52))
					{
						//������1
						++referenceTagCountLeftCurrent[k];
						break;
					}

			//ͳ��referenceTagCountRightCurrent�з���Ԫ�ظ���
			count = 0;
			for (auto x : referenceTagCountLeftCurrent)
				if (x)
					++count;
			readableTagLeftNum[i] = count;

			//�����±�ǩ�Ŀɶ��ԣ������ӽ��г���ɸѡ--�����Ͽɶ��Եģ�Ȩ�ر�Ϊ0
			//Ѱ��ָ����Χ�ڵ����ֵ�Լ�����
			//���ֵ
			int RSSI_LeftMaxIndex = myDataResultLeft.data.RSSI[numberFlagVicePre + 1];
			//���ֵ��ָ����Χ�ڵ�����
			int I_Left = 0;
			for (int j = numberFlagVicePre + 1; j < numberFlagVice; ++j)
				if (myDataResultLeft.data.RSSI[j] > RSSI_LeftMaxIndex)
				{
					RSSI_LeftMaxIndex = myDataResultLeft.data.RSSI[j];
					I_Left = j - numberFlagVicePre - 1;
				}

			//Ѱ����Ȳ���¼�����е���Ϣ
			int rowLeft = 0;
			int colLeft = 0;
			for (int j = 0; j < referenceTagNumLeft.size(); ++j)
				for (int k = 0; k < referenceTagNumLeft[0].size(); ++k)
					if (referenceTagNumLeft[j][k] == (I_Left + numberFlagVicePre + 1))
					{
						rowLeft = j;
						colLeft = k;
					}

			//ʵ��k-�ٽ��㷨 ��Ϊһ������
			//�ҵ���RSSI���ĵ�������9���ο���ǩ
			vector<int> optionalTagLeftFlag = knnsearch(referenceTag, referenceTag[colLeft], PF_TagReadableCount);

			//�ҳ���������̽Ѱ�����ɸ���
			//Ѱ��ָ����Χ�ڵ����ֵ�Լ�����
			//���ֵ
			int RSSI_RightMaxIndex = myDataResultRight.data.RSSI[numberFlagPre + 1];
			//���ֵ��ָ����Χ�ڵ�����
			int I_Right = 0;
			for (int j = numberFlagPre + 1; j < numberFlag; ++j)
				if (myDataResultRight.data.RSSI[j] > RSSI_RightMaxIndex)
				{
					RSSI_RightMaxIndex = myDataResultRight.data.RSSI[j];
					I_Right = j - numberFlagPre - 1;
				}

			//Ѱ����Ȳ���¼�����е���Ϣ
			int rowRight = 0;
			int colRight = 0;
			for (int j = 0; j < referenceTagNumRight.size(); ++j)
				for (int k = 0; k < referenceTagNumRight[0].size(); ++k)
					if (referenceTagNumRight[j][k] == (I_Right + numberFlagPre + 1))
					{
						rowRight = j;
						colRight = k;
					}

			//�ҵ���RSSI���ĵ�������9���ο���ǩ
			vector<int> optionalTagRightFlag = knnsearch(referenceTag, referenceTag[colRight], PF_TagReadableCount);

			//��̽Ѱ��9����ǩ�Ŀɶ���
			vector<bool> leftTagReadFlag(PF_TagReadableCount);
			for (int j = 0; j < PF_TagReadableCount; ++j)
				leftTagReadFlag[j] = (referenceTagCountLeft[optionalTagLeftFlag[j]] > 1) && (referenceTagCountLeftCurrent[optionalTagLeftFlag[j]] > 0);
			vector<bool> rightTagReadFlag(PF_TagReadableCount);
			for (int j = 0; j < PF_TagReadableCount; ++j)
				rightTagReadFlag[j] = (referenceTagCountRight[optionalTagRightFlag[j]] > 1) && (referenceTagCountRightCurrent[optionalTagRightFlag[j]] > 0);

			//���㵱ǰλ���£��������ߺ���Ӱ���Ƶ�λ��
			vector<vector<double>> PF_ParticleAntennaLeftCurrent(2, vector<double>(PF_Count));
			vector<vector<double>> PF_ParticleAntennaRightCurrent(2, vector<double>(PF_Count));
			for (int j = 0; j < PF_Count; ++j)
			{
				PF_ParticleAntennaLeftCurrent[0][j] = PF_ParticleRobot[i][0][j] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobot[i][2][j]);
				PF_ParticleAntennaLeftCurrent[1][j] = PF_ParticleRobot[i][1][j] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobot[i][2][j]);
				PF_ParticleAntennaRightCurrent[0][j] = PF_ParticleRobot[i][0][j] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobot[i][2][j]);
				PF_ParticleAntennaRightCurrent[1][j] = PF_ParticleRobot[i][1][j] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobot[i][2][j]);
			}

			vector<vector<double>> PF_ParticleShadowLeftCurrent(4, vector<double>(4));
			vector<vector<double>> PF_ParticleShadowRightCurrent(4, vector<double>(4));
			for (int j = 0; j < PF_Count; ++j)
			{
				//cout << j << ' ';
				for (int k = 0; k < 4; ++k)
				{
					PF_ParticleShadowLeftCurrent[k][0] = PF_ParticleRobot[i][0][j] + leftShadowUnreadablePointH[k] * cos(leftShadowUnreadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
					PF_ParticleShadowLeftCurrent[k][1] = PF_ParticleRobot[i][1][j] + leftShadowUnreadablePointH[k] * sin(leftShadowUnreadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
					PF_ParticleShadowLeftCurrent[k][2] = PF_ParticleRobot[i][0][j] + leftShadowReadablePointH[k] * cos(leftShadowReadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
					PF_ParticleShadowLeftCurrent[k][3] = PF_ParticleRobot[i][1][j] + leftShadowReadablePointH[k] * sin(leftShadowReadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);

					PF_ParticleShadowRightCurrent[k][0] = PF_ParticleRobot[i][0][j] + rightShadowUnreadablePointH[k] * cos(rightShadowUnreadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
					PF_ParticleShadowRightCurrent[k][1] = PF_ParticleRobot[i][1][j] + rightShadowUnreadablePointH[k] * sin(rightShadowUnreadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
					PF_ParticleShadowRightCurrent[k][2] = PF_ParticleRobot[i][0][j] + rightShadowReadablePointH[k] * cos(rightShadowReadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
					PF_ParticleShadowRightCurrent[k][3] = PF_ParticleRobot[i][1][j] + rightShadowReadablePointH[k] * sin(rightShadowReadablePointAlpha[k] + PF_ParticleRobot[i][2][j]);
				}

				//�����ǩȷ���Ƿ�ǰ���ƶ�������λ���Ƿ����Ҫ��
				vector<bool> optionalFlag(PF_TagReadableCount);

				//�ο���ǩ�����ߵľ���
				vector<double> leftDistanceThresholdFlag(PF_TagReadableCount);
				vector<double> rightDistanceThresholdFlag(PF_TagReadableCount);
				for (int k = 0; k < PF_TagReadableCount; ++k)
				{
					leftDistanceThresholdFlag[k] = sqrt(pow((referenceTag[optionalTagLeftFlag[k]][0] - PF_ParticleAntennaLeftCurrent[0][j]), 2) + pow((referenceTag[optionalTagLeftFlag[k]][1] - PF_ParticleAntennaLeftCurrent[1][j]), 2));
					rightDistanceThresholdFlag[k] = sqrt(pow((referenceTag[optionalTagRightFlag[k]][0] - PF_ParticleAntennaRightCurrent[0][j]), 2) + pow((referenceTag[optionalTagRightFlag[k]][1] - PF_ParticleAntennaRightCurrent[1][j]), 2));
				}

				//�ο���ǩ����Ӱ���Ĺ�ϵ
				//�ο���ǩ�Ƿ�����Ӱ����
				vector<double> xPointLeft(PF_TagReadableCount);
				vector<double> yPointLeft(PF_TagReadableCount);
				for (int k = 0; k < PF_TagReadableCount; ++k)
				{
					xPointLeft[k] = referenceTag[optionalTagLeftFlag[k]][0];
					yPointLeft[k] = referenceTag[optionalTagLeftFlag[k]][1];
				}
				vector<double> xLineLeft(4);
				vector<double> yLineLeft(4);
				for (int k = 0; k < 4; ++k)
				{
					xLineLeft[k] = PF_ParticleShadowLeftCurrent[k][2];
					yLineLeft[k] = PF_ParticleShadowLeftCurrent[k][3];
				}
				vector<bool> leftInpolygonReadableFlag = inpolygon(xPointLeft, yPointLeft, xLineLeft, yLineLeft);

				//�ο���ǩ�Ƿ�����Ӱ����
				vector<double> xPointRight(PF_TagReadableCount);
				vector<double> yPointRight(PF_TagReadableCount);
				for (int k = 0; k < PF_TagReadableCount; ++k)
				{
					xPointRight[k] = referenceTag[optionalTagRightFlag[k]][0];
					yPointRight[k] = referenceTag[optionalTagRightFlag[k]][1];
				}
				vector<double> xLineRight(4);
				vector<double> yLineRight(4);
				for (int k = 0; k < 4; ++k)
				{
					xLineRight[k] = PF_ParticleShadowRightCurrent[k][2];
					yLineRight[k] = PF_ParticleShadowRightCurrent[k][3];
				}
				vector<bool> rightInpolygonReadableFlag = inpolygon(xPointRight, yPointRight, xLineRight, yLineRight);

				for (int k = 0; k < PF_TagReadableCount; ++k)
					//�������߶�δ����
					if ((leftTagReadFlag[k] == 0) && (rightTagReadFlag[k] == 0))
					{
						//���������ֵ������ ����Ӱ����|| left_inpolygon_unreadable_flag(k) == 1
						if (leftDistanceThresholdFlag[k] > tagUnreadableRadius)
							//���������ֵ������ ����Ӱ����|| right_inpolygon_unreadable_flag(k) == 1
							if (rightDistanceThresholdFlag[k] > tagUnreadableRadius)
								optionalFlag[k] = 1;
					}
				//�����߶�����������δ����
					else if ((leftTagReadFlag[k] == 1) && (rightTagReadFlag[k] == 0))
					{
						//����С����ֵ������ ������Ӱ����
						if ((leftDistanceThresholdFlag[k] < tagReadableRadius) && (leftInpolygonReadableFlag[k] == 0))
							//���������ֵ������ ����Ӱ����|| right_inpolygon_unreadable_flag(k) == 1
							if (rightDistanceThresholdFlag[k] > tagUnreadableRadius)
								optionalFlag[k] = 1;
					}
				//������δ�����������߶���
					else if ((leftTagReadFlag[k] == 0) && (rightTagReadFlag[k] == 1))
					{
						//���������ֵ������ ����Ӱ����|| left_inpolygon_unreadable_flag(k) == 1
						if (leftDistanceThresholdFlag[k] > tagUnreadableRadius)
							//����С����ֵ������ ������Ӱ����
							if ((rightDistanceThresholdFlag[k] < tagReadableRadius) && (rightInpolygonReadableFlag[k] == 0))
								optionalFlag[k] = 1;
					}
				//�������߶�����
					else if ((leftTagReadFlag[k] == 1) && (rightTagReadFlag[k] == 1))
					{
						//����С����ֵ������ ������Ӱ����
						if ((leftDistanceThresholdFlag[k] < tagReadableRadius) && (leftInpolygonReadableFlag[k] == 0))
							//����С����ֵ������ ������Ӱ����
							if ((rightDistanceThresholdFlag[k] < tagReadableRadius) && (rightInpolygonReadableFlag[k] == 0))
								optionalFlag[k] = 1;
					}

				//���б�ǩ�Ŀɶ��Զ����ϳ���
				int count = 0;
				for (auto flag : optionalFlag)
					if (flag)
						++count;
				if (count == PF_TagReadableCount)
					PF_W_RSSI[i][j] = 1;
				else
					PF_W_RSSI[i][j] = 0;
				
			}

			//������
			auto numberFlagRightPre = numberFlagRight;
			for (int j = 0; j < referenceTagNum; ++j)
			{
				if (referenceTagCountRightCurrent[j] >= 1)
				{
					//�Ӻ���ǰ�ҵ�һ��С��numberFlag��ֵ������
					int numberFlagOption = 0;
					bool f = false;
					for (int k = referenceTagCountRight[j] - 1; k >= 0; --k)
					{
						if ((referenceTagNumRight[k][j] < numberFlag) && !f)
						{
							numberFlagOption = k;
							f = true;
						}
						if (f)
							break;
					}
					if (!isnan(double(numberFlagOption)))
						//��Ϊ��C++������������Բ���==1����==0
						if (numberFlagOption == 0)
						{
							//ͬ��Ԥ��Ϊ���������Ĵ��ڣ��ʸ�ֵΪ0
							numberFlagRight[1][j] = 0;
							numberFlagRight[0][j] = 0;
						}
						else
						{
							numberFlagRight[1][j] = numberFlagOption;
							numberFlagRight[0][j] = numberFlagRight[1][j];
							double distanceInterval = sqrt(pow((trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[1][j]][j]] - trackMobileRobotRight[0][numberFlag]), 2) +
								pow((trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[1][j]][j]] - trackMobileRobotRight[1][numberFlag]), 2));
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

			//�����ߣ����������£���ÿ����ǩ��Ӧ�����ݶ�--�����ߡ�ע������һ��ʱ������ݲ��ã����߼���δʵ��
			for (int j = 0; j < referenceTagNum; ++j)
			{
				if ((referenceTagCountLeftCurrent[j] >= 1) && (numberFlagVice > 1))
				{
					//�Ӻ���ǰ�ҵ�һ��С��numberFlag��ֵ������
					int numberFlagOption = 0;
					bool f = false;
					for (int k = referenceTagCountLeft[j] - 1; k >= 0; --k)
					{
						if ((referenceTagNumLeft[k][j] < numberFlagVice) && !f)
						{
							numberFlagOption = k;
							f = true;
						}
						if (f)
							break;
					}
					if (!isnan(double(numberFlagOption)))
						//��Ϊ��C++������������Բ���==1����==0
						if (numberFlagOption == 0)
						{
							//ͬ��Ԥ��Ϊ���������Ĵ��ڣ��ʸ�ֵΪ0
							numberFlagLeft[1][j] = 0;
							numberFlagLeft[0][j] = 0;
						}
						else
						{
							numberFlagLeft[1][j] = numberFlagOption;
							numberFlagLeft[0][j] = numberFlagLeft[1][j];
							//����Ϊʲô��left��right�����
							double distanceInterval = sqrt(pow((trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotRight[0][numberFlag]), 2) +
								pow((trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotRight[1][numberFlag]), 2));
							if (distanceInterval < distanceFarThreshold)
								while (1)
								{
									distanceInterval = sqrt(pow((trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[0][j]][j]]), 2) +
										pow((trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]] - trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[0][j]][j]]), 2));
									double leftTimeInterval = (readerTimeLeft[referenceTagNumLeft[numberFlagLeft[1][j]][j]] - readerTimeLeft[referenceTagNumLeft[numberFlagLeft[0][j]][j]]) / 1000000;
									//���������ܴ���...��ʱ�������ܴ���...
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

			//������
			vector<double> RSSI_MeanTagRight(referenceTagNum, -200);
			for (int j = 0; j < referenceTagNum; ++j)
				if (numberFlagRight[1][j] != numberFlagRight[0][j])
				{
					double sum = 0;
					for (int k = numberFlagRight[0][j]; k <= numberFlagRight[1][j]; ++k)
						sum += RSSI_TagRight[k][j];
					RSSI_MeanTagRight[j] = sum / (numberFlagRight[1][j] - numberFlagRight[0][j] + 1);
				}

			//������
			vector<double> RSSI_MeanTagLeft(referenceTagNum, -200);
			for (int j = 0; j < referenceTagNum; ++j)
				if (numberFlagLeft[1][j] != numberFlagLeft[0][j])
				{
					double sum = 0;
					for (int k = numberFlagLeft[0][j]; k <= numberFlagLeft[1][j]; ++k)
						sum += RSSI_TagLeft[k][j];
					RSSI_MeanTagLeft[j] = sum / (numberFlagLeft[1][j] - numberFlagLeft[0][j] + 1);
				}

			//��������
			vector<int> I_RSSI_Right(referenceTagNum);
			vector<double> RMTR(RSSI_MeanTagRight.begin(), RSSI_MeanTagRight.end());
			for (int j = 0; j < referenceTagNum; ++j)
				I_RSSI_Right[j] = j;
			for (int j = 0; j < referenceTagNum; ++j)
			{
				bool f = false;
				for (int k = referenceTagNum - 1; k > j; --k)
					if (RMTR[k] < RMTR[k - 1])
					{
						int temp = I_RSSI_Right[k];
						I_RSSI_Right[k] = I_RSSI_Right[k - 1];
						I_RSSI_Right[k - 1] = temp;
						double tem = RMTR[k];
						RMTR[k] = RMTR[k - 1];
						RMTR[k - 1] = tem;
						f = true;
					}
				if (!f)
					break;
			}

			//�����ߣ�����ÿ����ǩ��ǰ������ʱ�̵㣬�ƶ������˵�λ�ˣ������Ӧ������λ��, ��������Ȩֵ
			vector<double> PF_W_RightImprovement(PF_Count, -1);

			//ѡ������ʵı�ǩ
			if (RSSI_MeanTagRight[I_RSSI_Right.back()] > -53)
			{
				int j = I_RSSI_Right.back();
				if (numberFlagRight[1][j] != numberFlagRight[0][j])
				{
					//��һ��ʱ�̣�����״̬--������
					double robotXtAssumePso = trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[1][j]][j]];
					double robotYtAssumePso = trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[1][j]][j]];
					double robotThtAssumePso = trackMobileRobotRight[2][referenceTagNumRight[numberFlagRight[1][j]][j]];

					robotXtDiff = robotXtAssumePso - robotXtAssume[i];
					robotYtDiff = robotYtAssumePso - robotYtAssume[i];
					robotThtDiff = robotThtAssumePso - robotThtAssume[i];

					positionDifference1[0] = robotXtDiff * cos(-robotThtAssume[i]) - robotYtDiff * sin(-robotThtAssume[i]);
					positionDifference1[1] = robotXtDiff * sin(-robotThtAssume[i]) + robotYtDiff * cos(-robotThtAssume[i]);

					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[i][2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[i][2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[i][2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[i][2][k]);
					}

					vector<vector<double>> PF_ParticleRobotRightPso(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotRightPso[0][k] = PF_ParticleRobot[i][0][k] + positionDifference2[0][k];
						PF_ParticleRobotRightPso[1][k] = PF_ParticleRobot[i][1][k] + positionDifference2[1][k];
						PF_ParticleRobotRightPso[2][k] = PF_ParticleRobot[i][2][k] + robotThtDiff;
					}

					if (PF_ParticleAntennaRightPso.size() < (j + 1))
						PF_ParticleAntennaRightPso.resize(j + 1);
					vector<vector<double>> temp(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						//x����
						temp[0][k] = PF_ParticleRobotRightPso[0][k] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobotRightPso[2][k]);
						//y����
						temp[1][k] = PF_ParticleRobotRightPso[1][k] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobotRightPso[2][k]);
					}
					PF_ParticleAntennaRightPso[j] = temp;

					//ǰһ��ʱ�̣�����״̬--������
					double robotXtAssumePre = trackMobileRobotRight[0][referenceTagNumRight[numberFlagRight[0][j]][j]];
					double robotYtAssumePre = trackMobileRobotRight[1][referenceTagNumRight[numberFlagRight[0][j]][j]];
					double robotThtAssumePre = trackMobileRobotRight[2][referenceTagNumRight[numberFlagRight[0][j]][j]];

					robotXtDiff = robotXtAssumePre - robotXtAssume[i];
					robotYtDiff = robotYtAssumePre - robotYtAssume[i];
					robotThtDiff = robotThtAssumePre - robotThtAssume[i];

					positionDifference1[0] = robotXtDiff * cos(-robotThtAssume[i]) - robotYtDiff * sin(-robotThtAssume[i]);
					positionDifference1[1] = robotXtDiff * sin(-robotThtAssume[i]) + robotYtDiff * cos(-robotThtAssume[i]);

					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[i][2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[i][2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[i][2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[i][2][k]);
					}

					vector<vector<double>> PF_ParticleRobotRightPre(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotRightPre[0][k] = PF_ParticleRobot[i][0][k] + positionDifference2[0][k];
						PF_ParticleRobotRightPre[1][k] = PF_ParticleRobot[i][1][k] + positionDifference2[1][k];
						PF_ParticleRobotRightPre[2][k] = PF_ParticleRobot[i][2][k] + robotThtDiff;
					}

					if (PF_ParticleAntennaRightPre.size() < (j + 1))
						PF_ParticleAntennaRightPre.resize(j + 1);
					for (int k = 0; k < PF_Count; ++k)
					{
						//x����
						temp[0][k] = PF_ParticleRobotRightPre[0][k] + antennaHRightError * cos(-antennaAlphaRightError + PF_ParticleRobotRightPre[2][k]);
						//y����
						temp[1][k] = PF_ParticleRobotRightPre[1][k] + antennaHRightError * sin(-antennaAlphaRightError + PF_ParticleRobotRightPre[2][k]);
					}
					PF_ParticleAntennaRightPre[j] = temp;

					//�۲���λ�ݶ�
					PF_ObserveGradientRight[j] = PF_ObserveNativeRight[numberFlagRight[1][j]][j] - PF_ObserveNativeRight[numberFlagRight[0][j]][j];

					//������λ�ݶ�
					vector<double> PF_DistanceRightAntennaPre(PF_Count);
					vector<double> PF_DistanceRightAntennaPso(PF_Count);
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceRightAntennaPre[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaRightPre[j][0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaRightPre[j][1][k]), 2) + pow((referenceTag[j][2] - myVisionRightAntennaHigh), 2)) * 2;
						PF_DistanceRightAntennaPso[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaRightPso[j][0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaRightPso[j][1][k]), 2) + pow((referenceTag[j][2] - myVisionRightAntennaHigh), 2)) * 2;
						PF_PredictionRight[j][2][k] = (PF_DistanceRightAntennaPso[k] - PF_DistanceRightAntennaPre[k]) * 2 * PI / waveLengthVar[0];
					}

					//����Ȩ������������λ��
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceRight[j][k] = PF_PredictionRight[j][2][k] - PF_ObserveGradientRight[j];
						//��Ȩ��
						PF_W_Right[j][k] = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * exp(-pow(PF_DistanceRight[j][k], 2) / 2 / (2 * PF_R));
						PF_W_RightImprovement[k] = PF_W_Right[j][k];
					}
				}
				else
					for (auto& x : PF_W_Right[j])
						x = -1;
			}

			//��������
			vector<int> I_RSSI_Left(referenceTagNum);
			vector<double> RMTL(RSSI_MeanTagLeft.begin(), RSSI_MeanTagLeft.end());
			for (int j = 0; j < referenceTagNum; ++j)
				I_RSSI_Left[j] = j;
			for (int j = 0; j < referenceTagNum; ++j)
			{
				bool f = false;
				for (int k = referenceTagNum - 1; k > j; --k)
					if (RMTL[k] < RMTL[k - 1])
					{
						int temp = I_RSSI_Left[k];
						I_RSSI_Left[k] = I_RSSI_Left[k - 1];
						I_RSSI_Left[k - 1] = temp;
						double tem = RMTL[k];
						RMTL[k] = RMTL[k - 1];
						RMTL[k - 1] = tem;
						f = true;
					}
				if (!f)
					break;
			}

			//�����ߣ�����ÿ����ǩ��ǰ������ʱ�̵㣬�ƶ������˵�λ�ˣ������Ӧ������λ��, ��������Ȩֵ
			vector<double> PF_W_LeftImprovement(PF_Count, -1);

			if (RSSI_MeanTagLeft[I_RSSI_Left.back()] > -53)
			{
				int j = I_RSSI_Left.back();
				if (numberFlagLeft[1][j] != numberFlagLeft[0][j])
				{
					//��һ��ʱ�̣�����״̬--������
					double robotXtAssumePso = trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[1][j]][j]];
					double robotYtAssumePso = trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[1][j]][j]];
					double robotThtAssumePso = trackMobileRobotLeft[2][referenceTagNumLeft[numberFlagLeft[1][j]][j]];

					robotXtDiff = robotXtAssumePso - robotXtAssume[i];
					robotYtDiff = robotYtAssumePso - robotYtAssume[i];
					robotThtDiff = robotThtAssumePso - robotThtAssume[i];

					positionDifference1[0] = robotXtDiff * cos(-robotThtAssume[i]) - robotYtDiff * sin(-robotThtAssume[i]);
					positionDifference1[1] = robotXtDiff * sin(-robotThtAssume[i]) + robotYtDiff * cos(-robotThtAssume[i]);

					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[i][2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[i][2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[i][2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[i][2][k]);
					}

					vector<vector<double>> PF_ParticleRobotLeftPso(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotLeftPso[0][k] = PF_ParticleRobot[i][0][k] + positionDifference2[0][k];
						PF_ParticleRobotLeftPso[1][k] = PF_ParticleRobot[i][1][k] + positionDifference2[1][k];
						PF_ParticleRobotLeftPso[2][k] = PF_ParticleRobot[i][2][k] + robotThtDiff;
					}

					if (PF_ParticleAntennaLeftPso.size() < (j + 1))
						PF_ParticleAntennaLeftPso.resize(j + 1);
					vector<vector<double>> temp(2, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						//x����
						temp[0][k] = PF_ParticleRobotLeftPso[0][k] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPso[2][k]);
						//y����
						temp[1][k] = PF_ParticleRobotLeftPso[1][k] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPso[2][k]);
					}
					PF_ParticleAntennaLeftPso[j] = temp;

					//ǰһ��ʱ�̣�����״̬--������
					double robotXtAssumePre = trackMobileRobotLeft[0][referenceTagNumLeft[numberFlagLeft[0][j]][j]];
					double robotYtAssumePre = trackMobileRobotLeft[1][referenceTagNumLeft[numberFlagLeft[0][j]][j]];
					double robotThtAssumePre = trackMobileRobotLeft[2][referenceTagNumLeft[numberFlagLeft[0][j]][j]];

					robotXtDiff = robotXtAssumePre - robotXtAssume[i];
					robotYtDiff = robotYtAssumePre - robotYtAssume[i];
					robotThtDiff = robotThtAssumePre - robotThtAssume[i];

					positionDifference1[0] = robotXtDiff * cos(-robotThtAssume[i]) - robotYtDiff * sin(-robotThtAssume[i]);
					positionDifference1[1] = robotXtDiff * sin(-robotThtAssume[i]) + robotYtDiff * cos(-robotThtAssume[i]);

					for (int k = 0; k < PF_Count; ++k)
					{
						positionDifference2[0][k] = positionDifference1[0] * cos(PF_ParticleRobot[i][2][k]) - positionDifference1[1] * sin(PF_ParticleRobot[i][2][k]);
						positionDifference2[1][k] = positionDifference1[0] * sin(PF_ParticleRobot[i][2][k]) + positionDifference1[1] * cos(PF_ParticleRobot[i][2][k]);
					}

					vector<vector<double>> PF_ParticleRobotLeftPre(3, vector<double>(PF_Count));
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_ParticleRobotLeftPre[0][k] = PF_ParticleRobot[i][0][k] + positionDifference2[0][k];
						PF_ParticleRobotLeftPre[1][k] = PF_ParticleRobot[i][1][k] + positionDifference2[1][k];
						PF_ParticleRobotLeftPre[2][k] = PF_ParticleRobot[i][2][k] + robotThtDiff;
					}

					if (PF_ParticleAntennaLeftPre.size() < (j + 1))
						PF_ParticleAntennaLeftPre.resize(j + 1);
					for (int k = 0; k < PF_Count; ++k)
					{
						//x����
						temp[0][k] = PF_ParticleRobotLeftPre[0][k] - antennaHLeftError * cos(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPre[2][k]);
						//y����
						temp[1][k] = PF_ParticleRobotLeftPre[1][k] + antennaHLeftError * sin(PI - antennaAlphaLeftError - PF_ParticleRobotLeftPre[2][k]);
					}
					PF_ParticleAntennaLeftPre[j] = temp;

					//�۲���λ�ݶ�
					PF_ObserveGradientLeft[j] = PF_ObserveNativeLeft[numberFlagLeft[1][j]][j] - PF_ObserveNativeLeft[numberFlagLeft[0][j]][j];

					//������λ�ݶ�
					vector<double> PF_DistanceLeftAntennaPre(PF_Count);
					vector<double> PF_DistanceLeftAntennaPso(PF_Count);
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceLeftAntennaPre[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaLeftPre[j][0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaLeftPre[j][1][k]), 2) + pow((referenceTag[j][2] - myVisionLeftAntennaHigh), 2)) * 2;
						PF_DistanceLeftAntennaPso[k] = sqrt(pow((referenceTag[j][0] - PF_ParticleAntennaLeftPso[j][0][k]), 2) + pow((referenceTag[j][1] - PF_ParticleAntennaLeftPso[j][1][k]), 2) + pow((referenceTag[j][2] - myVisionLeftAntennaHigh), 2)) * 2;
						PF_PredictionLeft[j][2][k] = (PF_DistanceLeftAntennaPso[k] - PF_DistanceLeftAntennaPre[k]) * 2 * PI / waveLengthVar[0];
					}

					//����Ȩ������������λ��
					for (int k = 0; k < PF_Count; ++k)
					{
						PF_DistanceLeft[j][k] = PF_PredictionLeft[j][2][k] - PF_ObserveGradientLeft[j];
						//��Ȩ��
						PF_W_Left[j][k] = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * exp(-pow(PF_DistanceLeft[j][k], 2) / 2 / (2 * PF_R));
						PF_W_LeftImprovement[k] = PF_W_Left[j][k];
					}
				}
				else
					for (auto& x : PF_W_Left[j])
						x = -1;
			}

			//������ǲ���̫�������ˣ�д��һ��������
			//����Ȩ������
			vector<vector<double>> PF_W_PrePe;
			int n = 0;
			count = 0;
			for (auto x : PF_W_RightImprovement)
				if (x == -1)
					++count;
			if (count != PF_Count)
			{
				++n;
				PF_W_PrePe.resize(n);
				PF_W_PrePe[n - 1] = PF_W_RightImprovement;
			}
			count = 0;
			for (auto x : PF_W_LeftImprovement)
				if (x == -1)
					++count;
			if (count != PF_Count)
			{
				++n;
				PF_W_PrePe.resize(n);
				PF_W_PrePe[n - 1] = PF_W_LeftImprovement;
			}

			vector<double> PF_W_Pre(PF_Count, 1);
			if (PF_W_PrePe.size() == 0)
			{
				phaseFlag[i] = false;
			}
			else if (PF_W_PrePe.size() == 1)
			{
				double sum = 1;
				for (int j = 0; j < PF_Count; ++j)
					sum *= PF_W_PrePe[0][j];
				for (int j = 0; j < PF_Count; ++j)
					PF_W_Pre[j] = sum;
				phaseFlag[i] = true;
			}
			else
			{
				for (int j = 0; j < PF_Count; ++j)
					PF_W_Pre[j] = PF_W_PrePe[0][j] * PF_W_PrePe[1][j];
				phaseFlag[i] = true;
			}

			for (int j = 0; j < PF_Count; ++j)
			{
				PF_W[i][0][j] = PF_W_Pre[j] + 1e-99;
				PF_W[i][1][j] = PF_W[i][0][j] * PF_W[i - 1][1][j] * PF_W_RSSI[i][j];
			}

			double sum = accumulate(PF_W[i][1].begin(), PF_W[i][1].end(), 0.0);
			for (auto& x : PF_W[i][1])
				x /= sum;

			//x��y����ľ�ֵ����Э����
			//�������ӵ�����λ��--x��y����
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					PF_CenterMean[j][i] += PF_ParticleRobot[i][j][k] * PF_W[i][1][k];
			double weightSqr = 0;
			for (int j = 0; j < PF_Count; ++j)
				weightSqr += PF_W[i][1][j] * PF_W[i][1][j];

			double factor = 0;
			if (abs(weightSqr - 1.0) < sqrt(DBL_EPSILON))
				factor = 1.0;
			else
				factor = 1 / (1 - weightSqr);

			vector<vector<double>> meanDiff(2, vector<double>(PF_Count));
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < PF_Count; ++k)
					meanDiff[j][k] = PF_ParticleRobot[i][j][k] - PF_CenterMean[j][i];

			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < 2; ++k)
				{
					for (int m = 0; m < PF_Count; ++m)
						PF_CenterVar[i][j][k] += meanDiff[j][m] * PF_W[i][1][m] * meanDiff[k][m];
					PF_CenterVar[i][j][k] *= factor;
				}

			//�ǶȾ�ֵ��Э���
			double sinsum = 0;
			for (int j = 0; j < PF_Count; ++j)
				sinsum += PF_W[i][1][j] * sin(PF_ParticleRobot[i][2][j]);
			double cossum = 0;
			for (int j = 0; j < PF_Count; ++j)
				cossum += PF_W[i][1][j] * cos(PF_ParticleRobot[i][2][j]);
			double resultantLength = sqrt(pow(sinsum, 2) + pow(cossum, 2));
			PF_CenterMean[2][i] = atan2(sinsum, cossum);
			PF_CenterVar[i][2][2] = -2 * log(resultantLength);
			for (int j = 0; j < PF_Count; ++j)
				PF_CenterMean[3][i] += PF_ParticleRobot[i][2][j] * PF_W[i][1][j];

			//�жϵ�ǰ�Ƿ������ز���
			double squaredSum = weightSqr;

			//��Ч���ӵ�����
			double neff = 1 / squaredSum;
			neffRatioPre[i] = neff / PF_Count;
			//cout << neffRatioPre[i] << ' ';
			//�ǵû�ԭ������
			if (neffRatioPre[i] < neffRatioThreshold)
			{
				//ִ��ϵͳ�ز���
				int numNewParticles = PF_Count;
				double randNum = rand() / double(RAND_MAX);
				vector<double> randSamples(numNewParticles);
				for (int j = 0; j < numNewParticles; ++j)
					//randSamples[j] = j * (1 - 1.0 / numNewParticles) / (numNewParticles - 1) + randNum[i];//����
					randSamples[j] = j * (1 - 1.0 / numNewParticles) / (numNewParticles - 1) + randNum / numNewParticles;

				int n = 0;
				int index = 0;
				// Q = cumsum(PF_w(2,:,i));
				vector<double> Q(PF_Count);
				for (int j = 0; j < PF_Count; ++j)
					Q[j] = accumulate(PF_W[i][1].begin(), PF_W[i][1].begin() + j + 1, 0.0);
				vector<vector<double>> PF_ParticleNew(3, vector<double>(numNewParticles));
				while (n < numNewParticles)
				{
					while (Q[index] <= randSamples[n])
						index = index % (PF_Count - 1) + 1;
					//�õ������Ӽ�
					for (int j = 0; j < 3; ++j)
						PF_ParticleNew[j][n] = PF_ParticleRobot[i][j][index];
					++n;
				}
				//�õ������ز������Ӽ�
				PF_ParticleRobot[i] = PF_ParticleNew;
				//�󱾴ε�����Ȩ��
				for (auto& x : PF_W[i][1])
					x = 1.0 / PF_Count;
				PF_W_Slow = 0;
				PF_W_Fast = 0;
				PF_ReSample[i] = 1;
			}
			else
			{
				//��ִ���ز���
				PF_ReSample[i] = -1;
			}

			//��λ���
			robotXt[i] = mobileRobotPoseVisionRight[numberFlag][0];
			robotYt[i] = mobileRobotPoseVisionRight[numberFlag][1];
			robotTht[i] = mobileRobotPoseVisionRight[numberFlag][2];

			antennaLeft[0][i] = robotXt[i] - antennaHLeftError * cos(PI - antennaAlphaLeftError - robotTht[i]);
			antennaLeft[1][i] = robotYt[i] + antennaHLeftError * sin(PI - antennaAlphaLeftError - robotTht[i]);
			antennaRight[0][i] = robotXt[i] + antennaHRightError * cos(-antennaAlphaRightError + robotTht[i]);
			antennaRight[1][i] = robotYt[i] + antennaHRightError * sin(-antennaAlphaRightError + robotTht[i]);

			robotEstimationError[0][i] = PF_CenterMean[0][i] - robotXt[i];
			robotEstimationError[1][i] = PF_CenterMean[1][i] - robotYt[i];
			robotEstimationError[2][i] = sqrt(pow(robotEstimationError[0][i], 2) + pow(robotEstimationError[1][i], 2));

			robotEstimationError[3][i] = PF_CenterMean[2][i] - robotTht[i];

			robotEstimationError[4][i] = PF_CenterMean[3][i] - robotTht[i];

		}
		//��ͼ
		// 	figure(1)
		//     hold off
		//     plot(reference_tag(:,1), reference_tag(:,2), 'r.', 'markersize',50);  %ϵͳ״̬λ��
		//     hold on

		//     color = PF_w(2, :,i);
		//     scatter(PF_particle_robot(1, :, i), PF_particle_robot(2, :, i), 'filled','cdata',color);   %��������λ��
		//     plot(PF_center_mean(1, 1:i),PF_center_mean(2, 1:i),'b.', 'markersize',25);%��������λ��
		//     plot(robot_xt(1:i),robot_yt(1:i),'r.');%�����˵�λ��
		//     plot(antenna_left(1, 1:i),antenna_left(2, 1:i),'k.');%������λ��
		//     plot(antenna_right(1, 1:i),antenna_right(2, 1:i),'k.');%������λ��

		//     quiver(PF_center_mean(1, i),PF_center_mean(2,i), cos(PF_center_mean(3, i)), sin(PF_center_mean(3, i)),10, 'color', 'c', 'linewidth', 3);
		//     quiver(PF_particle_robot(1, :, i), PF_particle_robot(2, :, i), cos(PF_particle_robot(3, :, i)), sin(PF_particle_robot(3, :, i)),1, 'color', 'k', 'linewidth', 3);
		//     set(gca,'DataAspectRatio',[1 1 1])

		//     axis([PF_scope(1,1)-200 400 PF_scope(2,1)-100 PF_scope(2,2)+100]);

		//����Ƶ�ʰѿ�
		auto endTime = clock();
		//time[i] = (endTime - startTime) / CLOCKS_PER_SEC;
		//��Windows��ʱ�䰴�պ�����㣬��Linux��ʱ�䰴�������(???)
		//Sleep((1 - time[i]) * 1000);
		// Sleep(1 - time[i]);
	}

	//����
	for (int i = 0; i < sizeof(myVisionTotal) / sizeof(double**); i++)
	{
		mxArray* pF = mxGetField(pS, 0, name[i]);
		auto row = mxGetM(pF);
		auto col = mxGetN(pF);
		for (int j = 0; j < row; j++)
			delete[] * (*(q + i) + j);
		delete[] * (q + i);
	}
	matClose(pM);

	/*������֤�����ⲿ���ݵ��벢��֮�������ֵ����ֵ�Աȣ���������ֵ�����*/
	/*�ⲿ���ݴ洢*/
	//vector<vector<vector<double>>> PF_ParticleRobot0(500, vector<vector<double>>(3, vector<double>(PF_Count)));
	//vector<vector<vector<double>>> PF_W0(Times, vector<vector<double>>(2, vector<double>(PF_Count)));
	//vector<vector<double>> PF_CenterMean0(4, vector<double>(Times));
	//vector<double> robotXt0(Times);
	//vector<double> robotYt0(Times);
	//vector<vector<double>> antennaLeft0(2, vector<double>(Times));
	//vector<vector<double>> antennaRight0(2, vector<double>(Times));

	//MATFile* pM0 = matOpen("E:/MATLAB workspace/hgdw/test400.mat", "r");

	//mxArray* pV1 = matGetVariable(pM0, "PF_particle_robot");
	//double* pT1 = (double*)mxGetData(pV1);

	//mxArray* pV2 = matGetVariable(pM0, "PF_w");
	//double* pT2 = (double*)mxGetData(pV2);

	//mxArray* pV3 = matGetVariable(pM0, "PF_center_mean");
	//double* pT3 = (double*)mxGetData(pV3);

	//mxArray* pV4 = matGetVariable(pM0, "robot_xt");
	//double* pT4 = (double*)mxGetData(pV4);

	//mxArray* pV5 = matGetVariable(pM0, "robot_yt");
	//double* pT5 = (double*)mxGetData(pV5);

	//mxArray* pV6 = matGetVariable(pM0, "antenna_left");
	//double* pT6 = (double*)mxGetData(pV6);

	//mxArray* pV7 = matGetVariable(pM0, "antenna_right");
	//double* pT7 = (double*)mxGetData(pV7);

	//for (int i = 0; i < Times; ++i)
	//{
	//	for (int j = 0; j < 3; ++j)
	//	{
	//		for (int k = 0; k < 500; ++k)
	//		{
	//			PF_ParticleRobot0[i][j][k] = pT1[i * 500 * 3 + k * 3 + j];
	//		}
	//	}
	//	for (int j = 0; j < 2; ++j)
	//	{
	//		for (int k = 0; k < 500; ++k)
	//		{
	//			PF_W0[i][j][k] = pT2[i * 500 * 2 + k * 2 + j];
	//		}
	//	}
	//	robotXt0[i] = pT4[i];
	//	robotYt0[i] = pT5[i];
	//}
	//for (int i = 0; i < 4; ++i)
	//{
	//	for (int j = 0; j < Times; ++j)
	//	{
	//		PF_CenterMean0[i][j] = pT3[j * 4 + i];
	//	}
	//}
	//for (int i = 0; i < 2; ++i)
	//{
	//	for (int j = 0; j < Times; ++j)
	//	{
	//		antennaLeft0[i][j] = pT6[j * 2 + i];
	//		antennaRight0[i][j] = pT7[j * 2 + i];
	//	}
	//}

	/*���ⲿ�����������*/
	//int count = 0;
	//for (int i = 0; i < Times; ++i)
	//{
	//	//cout << i << endl;
	//	for (int j = 0; j < 3; ++j)
	//	{
	//		for (int k = 0; k < 500; ++k)
	//		{
	//			if (abs(PF_ParticleRobot0[i][j][k] - PF_ParticleRobot[i][j][k]) > ep)
	//			{
	//				++count;
	//				//cout << abs(PF_ParticleRobot0[i][j][k] - PF_ParticleRobot[i][j][k]) << ' ';
	//			}
	//				
	//		}
	//		//cout << endl;
	//	}
	//}
	//cout << count << endl;
	//
	//for (int i = 0; i < Times; ++i)
	//{
	//	//cout << i << endl;
	//	for (int j = 0; j < 2; ++j)
	//	{
	//		for (int k = 0; k < 500; ++k)
	//		{
	//			if (abs(PF_W0[i][j][k] - PF_W[i][j][k]) > ep)
	//			{
	//				++count;
	//				//cout << abs(PF_W0[i][j][k] - PF_W[i][j][k]) << ' ';
	//			}
	//		}
	//		//cout << endl;
	//	}
	//	if (abs(robotXt0[i] - robotXt[i]) > ep)
	//	{
	//		++count;
	//		//cout << abs(robotXt0[i] - robotXt[i]) << ' ';
	//	}
	//		
	//	if (abs(robotYt0[i] - robotYt[i]) > ep)
	//		++count;
	//}
	//cout << count << endl;

	//for (int i = 0; i < 4; ++i)
	//{
	//	for (int j = 0; j < Times; ++j)
	//	{
	//		if (abs(PF_CenterMean0[i][j] - PF_CenterMean[i][j]) > ep)
	//			++count;
	//	}
	//}
	//cout << count << endl;

	//for (int i = 0; i < 2; ++i)
	//{
	//	for (int j = 0; j < Times; ++j)
	//	{
	//		if (abs(antennaLeft0[i][j] - antennaLeft[i][j]) > ep)
	//			++count;
	//		if (abs(antennaRight0[i][j] - antennaRight[i][j]) > ep)
	//			++count;
	//	}
	//}
	//cout << count << endl;

	return 0;
}
int mymax(const vector<int>& v)
{
	int m = v.front();
	for (auto x : v)
		m = max(m, x);
	return m;
}
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
double distance(const vector<double>& v1, const vector<double>& v2)
{
	double sum = 0;
	for (int i = 0; i < v1.size(); ++i)
		sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
	return sqrt(sum);
}
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
vector<bool> inpolygon(const vector<double>& xp, const vector<double>& yp, const vector<double>& xl, const vector<double>& yl)
{
	vector<bool> result(xp.size());
	for (int i = 0; i < xp.size(); ++i)
	{
		bool flag = false;	   //�жϽ����true�������ڶ�����ڣ�false:��δ���ڶ�����ڣ�
		int k = xl.size() - 1; //�Ƕ���ε����һ������
		for (int j = 0; j < xl.size(); ++j)
		{
			//�жϵ��Ƿ����߶ε�����
			if ((yl[j] < yp[i] && yl[k] >= yp[i]) || (yl[k] < yp[i] && yl[j] >= yp[i]))
			{
				//��������ʽ���̼��������P��ƽ����X���ֱ�����߶εĽ��㣬����ʽ���̣�x = x1 +  (y - y1) * (x2 - x1) / (y2 - y1);
				if (xl[j] + (yp[i] - yl[j]) * (xl[k] - xl[j]) / (yl[k] - yl[j]) < xp[i])
					flag = !flag;
			}
			//������һ�߶��ж�
			k = j;
		}
		result[i] = flag;
	}
	return result;
}
