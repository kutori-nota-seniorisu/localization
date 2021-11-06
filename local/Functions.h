#pragma once
#include <vector>
using std::vector;

//��ȡ���������ֵ ��һ������
int mymax(const vector<int>&);
//��̬�ֲ����ɺ�������������Ϊmu����׼��Ϊsigma����̬�ֲ�
double gaussrand(const double& mu, const double& sigma);
//������룬v1��v2��������С��ͬ������
double distance(const vector<double>& v1, const vector<double>& v2);
//ʵ��k-�ٽ��㷨
vector<int> knnsearch(vector<vector<double>> testData, vector<double> targetData, int k);
//�жϵ��Ƿ�����������
vector<bool> inpolygon(const vector<vector<double>>& pointData, const vector<int>& optionalTagFlag, const vector<vector<double>>& testData,const int& numOfTag);
//�ҵ���̼Ƶ�ʱ�����Ӿ�ʱ�������е�ǰ�����������λ�˵�
int findpso(const vector<double>& visionTimeDiff, const vector<double>& odometerTimeDiff, const int& index, bool& flag);
int findpre(const vector<double>& visionTimeDiff, const vector<double>& odometerTimeDiff, const int& index, bool& flag);
//������������ڴ��·�Χ�����ڳ�ʼ�����ӣ�
vector<vector<double>> calPF_Scope(vector<vector<double>>& referenceTag, vector<int>& randData, vector<int>& colLeft, vector<int>& colRight);
//��ȡ.mat�ļ�
//void readmat(const char *matPath,const char **fieldName,double ***q,int len);