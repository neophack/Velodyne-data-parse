#include <iostream>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <conio.h>
#include <io.h>

#include <algorithm>
#include "Config.h"

using namespace std;

struct tPoint
{
	float px ;
	float py;
	float pz;
	float pr ;
	tPoint(float px, float py, float pz, float pr)
	{
		this->pr = pr;
		this->px = px;
		this->py = py;
		this->pz = pz;
	}
};

void getFiles(std::string path, std::vector<std::string>& files);
bool sortbyname(std::string a, std::string b);

vector<tPoint> points;
vector<string> binaryfilespath;

string readpath_;
string savepath_;

const char ConfigFile[] = "param.txt";
Config configSettings(ConfigFile);

void paramRead()
{
	readpath_ = configSettings.Read("readpath_", readpath_);
	savepath_ = configSettings.Read("savepath_", savepath_);

	cout << "----------------------param confirm---------------------- " << endl;
	cout << "readpath_ : " << readpath_ << endl;
	cout << "savepath_     : " << savepath_ << endl;

	cout << "----------------------param confirm---------------------- " << endl;
	system("pause");

}

void main()
{
	paramRead();

	getFiles(readpath_, binaryfilespath);
	cout << binaryfilespath.size() << endl;
	for (auto binaryfile : binaryfilespath)
	{
		// allocate 4 MB buffer (only ~ 130*4*4 KB are needed)
		
		int32_t num = 1000000;
		float *data = (float*)malloc(num*sizeof(float));

		// pointers
		float *px = data + 0;
		float *py = data + 1;
		float *pz = data + 2;
		float *pr = data + 3;

		// load point cloud
		FILE *stream;
		stream = fopen(binaryfile.c_str(), "rb");
		num = fread(data, sizeof(float), num, stream) / 4;
		for (int32_t i = 0; i<num; i++) {
			points.push_back(tPoint(*px, *py, *pz, *pr));
			px += 4; py += 4; pz += 4; pr += 4;
		}

		fclose(stream);
		int  f = binaryfile.find("0000");
		ofstream exampleout(savepath_ + "\\"+ binaryfile.substr(f, 9) + ".txt");

		for (auto point : points)
		{
			exampleout << point.px << " " << point.py << " " << point.pz << " " << point.pr << endl;
		}
		exampleout.close();

		cout << binaryfile.substr(f, 9).c_str() << " extract done. " << endl;
		points.clear();
	}


	system("pause");
}


bool sortbyname(std::string a, std::string b)
{
	//���54���ܻ�仯
	a = a.erase(a.length() - 4, a.length() - 2);
	a = a.erase(0, 125);
	b = b.erase(b.length() - 4, b.length() - 2);
	b = b.erase(0, 125);

	return atoi(a.c_str()) < atoi(b.c_str());
}


void getFiles(std::string path, std::vector<std::string>& files)
{
	//�ļ����  
	long long  hFile = 0;
	//�ļ���Ϣ������һ���洢�ļ���Ϣ�Ľṹ��  
	struct _finddata_t fileinfo;
	std::string p;//�ַ��������·��
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//�����ҳɹ��������
	{
		do
		{
			//�����Ŀ¼,����֮�����ļ����ڻ����ļ��У�  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				//�ļ���������"."&&�ļ���������".."
				//.��ʾ��ǰĿ¼
				//..��ʾ��ǰĿ¼�ĸ�Ŀ¼
				//�ж�ʱ�����߶�Ҫ���ԣ���Ȼ�����޵ݹ�������ȥ�ˣ�
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}

			}
			//�������,�����б�  
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		//_findclose������������
		_findclose(hFile);
	}

	sort(files.begin(), files.end(), sortbyname);
}