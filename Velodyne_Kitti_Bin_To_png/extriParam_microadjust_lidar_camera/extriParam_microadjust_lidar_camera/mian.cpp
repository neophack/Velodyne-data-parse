#include <iostream>
#include <opencv2\opencv.hpp>
#include <fstream>
#include <thread>
#include <windows.h>
#include <string.h> 
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h> 
#include <conio.h>
#include <../Eigen/Dense>  
#include "Config.h" 
#include "mvp.h"
#include <GL/glut.h>
#include <GL/GL.h>
#include <GL/GLU.h>
#include <GL\freeglut.h>
#include <direct.h>  
#include <io.h>  

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace cv;

#define RADIAN 3.1415926 / 180
#define DEGREE  180 / 3.1415926 
#define PI  3.1415926 
static int  index = 0;
void showParam();

CRITICAL_SECTION g_cs;
CRITICAL_SECTION vector_cs;


//三维原始点云数据
typedef struct lidardata{
	double x;
	double y;
	double z;
	float intensity;
	int laserid;
}laser;

//投影二维点云
typedef struct lidarPro {
	double x;
	double y;
	float intensity;
	int isBackground;
	int laserid;
}laserPro;

Mat Rot = Mat::ones(3, 3, CV_32FC1);
Mat Tra = Mat::ones(3, 1, CV_32FC1);
Mat Intri = Mat::ones(3, 3, CV_32FC1);
Mat pixel = Mat::ones(3, 1, CV_32FC1);
Mat cameraCoor = Mat::ones(3, 1, CV_32FC1);

Mat screenshow = Mat::zeros(960,1280,CV_8UC1);

vector<laser> pointSeq2;
vector<Point3f> pointSeq;
IplImage* pScr;
bool bQuit = false;
bool showCtl = true;
bool vectorCtl = true;

string cameraPath;
string liadrPath;
string ParamPath;

string camera_file_Path;
string liadr_file_Path;
int file_suffix = 1;    /*1 - 30*/

const char ConfigFile1[] = "configsetup.txt";
Config configSettings2(ConfigFile1);


typedef GLbyte* bytePt;

int winWidth = 1280;
int winHeight = 960;
int arrLen = winWidth * winHeight * 3;
GLbyte* colorArr = new GLbyte[arrLen];


struct tPoint
{
	float px;
	float py;
	float pz;
	float pr;
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

//读取配置文件中的二进制点云file路径
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

//根据传入的文件名排序
bool sortbyname(std::string a, std::string b)
{
	//这个54可能会变化
	a = a.erase(a.length() - 4, a.length() - 2);
	a = a.erase(0, 125);
	b = b.erase(b.length() - 4, b.length() - 2);
	b = b.erase(0, 125);

	return atoi(a.c_str()) < atoi(b.c_str());
}

//获取文件夹下所有的文件1
void getFiles(std::string path, std::vector<std::string>& files)
{
	//文件句柄  
	long long  hFile = 0;
	//文件信息，声明一个存储文件信息的结构体  
	struct _finddata_t fileinfo;
	std::string p;//字符串，存放路径
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//若查找成功，则进入
	{
		do
		{
			//如果是目录,迭代之（即文件夹内还有文件夹）  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}

			}
			//如果不是,加入列表  
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		//_findclose函数结束查找
		_findclose(hFile);
	}

	sort(files.begin(), files.end(), sortbyname);

}

//根据文件大小排序
bool cmp(std::string const &arg_a, std::string const &arg_b) {
	return arg_a.size() < arg_b.size() || (arg_a.size() == arg_b.size() && arg_a < arg_b);
}


//获取文件夹下所有的文件
vector<string> getFiles(string cate_dir)
{
	vector<string> files;//存放文件名  

	_finddata_t file;
	intptr_t  lf;
	//输入文件夹路径  
	if ((lf = _findfirst(cate_dir.c_str(), &file)) == -1) {
		//cout << cate_dir << " not found!!!" << endl;
	}
	else {
		while (_findnext(lf, &file) == 0) {
			//cout << file.name << endl;//输出文件名  
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			files.push_back(file.name);
		}
	}
	_findclose(lf);
	//排序，按从小到大排序  
	sort(files.begin(), files.end(), cmp);
	return files;
}


//将点云数据保存到pixel .txt
void saveColorData(bytePt& _pt, string& _str) {
	FILE* pFile = NULL;
	ofstream pixelsfile(_str.c_str());
	pFile = fopen(_str.c_str(), "wt");
	if (!pFile) 
	{ 
		fprintf(stderr, "error \n"); 
		exit(-1); 
	}
	for (int i = 0; i<winWidth * winHeight * 3; i++) 
	{
		if (colorArr[i] == -1) { 
			colorArr[i] = 255; 
		}
	}
	for (int i = 0; i<300; i+3) 
	{
		//cout << "R:" << colorArr[i] << "G:" << colorArr[i + 1] << "B:" << colorArr[i + 2] << endl;
		pixelsfile << "R:" << colorArr[i] << "G:" << colorArr[i + 1] << "B:" << colorArr[i + 2] << endl;
		//fprintf(pFile, "%d\t%d\t%d\n", colorArr[i], colorArr[i+1], colorArr[i+2]);
	}
	pixelsfile.close();
	fclose(pFile);
	printf("color data saved! \n");
}


//init background color
void init() {
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);
}

//trangles projection
void display() {
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);//view矩阵
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glMatrixMode(GL_PROJECTION);//project矩阵
	glLoadIdentity();
	gluPerspective(45.0, 1.0, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);

	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_TRIANGLES);
	glVertex3f(0.0, 25.0, 0.0);
	glVertex3f(-25.0, -25.0, 0.0);
	glVertex3f(25.0, -25.0, 0.0);
	glEnd();

	glFlush();
}

//save to .png 
void saveColorData2img(bytePt& _pt, string& _str) {
	cv::Mat img;
	vector<cv::Mat> imgPlanes;
	img.create(winHeight, winWidth, CV_8UC3);
	cv::split(img, imgPlanes);

	for (int i = 0; i < winHeight; i++) {
		UCHAR* plane0Ptr = imgPlanes[0].ptr<UCHAR>(i);
		UCHAR* plane1Ptr = imgPlanes[1].ptr<UCHAR>(i);
		UCHAR* plane2Ptr = imgPlanes[2].ptr<UCHAR>(i);
		for (int j = 0; j < winWidth; j++) {
			int k = 3 * (i * winWidth + j);
			plane2Ptr[j] = _pt[k];
			plane1Ptr[j] = _pt[k + 1];
			plane0Ptr[j] = _pt[k + 2];
		}
	}
	cv::merge(imgPlanes, img);
	cv::flip(img, img, 0); // !!!  
	cv::imwrite(_str.c_str(), img);

	printf("opencv save opengl img done! \n");
}

void keyboard(unsigned char key, int x, int y) {
	GLint viewPort[4] = { 0 };
	GLfloat* _data;
	GLsizei numComponet;
	GLsizei bufferSize;
	unsigned char*  data;
	string pngfilename = "D:\\KITTI velodynedata\\2011_09_26_drive_0101_sync\\velodyne_points\\png\\" + to_string(index) + ".png";
	switch (key) {
	case 'c':
	case 'C':
		glGetIntegerv(GL_VIEWPORT, viewPort);

		numComponet = 3;
		bufferSize = viewPort[2] * viewPort[3] * sizeof(GLfloat)*numComponet;

		 _data = new GLfloat[bufferSize];
		data = new unsigned char[bufferSize];
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);//设置4字节对齐  
		glReadBuffer(GL_FRONT);

		glReadPixels(viewPort[0], viewPort[1], viewPort[2], viewPort[3], GL_RGB, GL_UNSIGNED_BYTE, colorArr);
		/*glReadBuffer(GL_BACK);
		for (int i = 0; i <bufferSize; i++)
		{
			data[i] = colorArr[i] * 255;
		}*/
		//printf("color data read !\n");

		saveColorData2img(colorArr, pngfilename);
		cout << "save " << pngfilename << endl;

		//saveColorData2img(colorArr, (string)"ddd.png");
		//saveColorData(colorArr, (string)"tmpcolor.txt");
	default:
		break;
	}
}

/*
something wrong 
void ScreenShot()
{
	int width = 1280;
	int height = 960;
	GLint pView[4];
	glGetIntegerv(GL_VIEWPORT, pView);//得到视图矩阵,pView[2]为宽即width,pView[3]为高即height  

	GLsizei numComponet = 3;
	GLsizei bufferSize = pView[2] * pView[3] * sizeof(GLfloat)*numComponet;

	GLfloat* _data = new GLfloat[bufferSize];
	unsigned char*  data = new unsigned char[bufferSize];
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);//设置4字节对齐  
	glReadBuffer(GL_FRONT);
	glReadPixels(pView[0], pView[1], pView[2], pView[3], GL_BGR_EXT, GL_FLOAT, _data);//不是GL_RGB的读取方式，而是GL_BGR或者GL_BGR_Ext  
	glReadBuffer(GL_BACK);
	for (int i = 0; i <bufferSize; i++)
	{
		data[i] = _data[i] * 255;
	}
	BITMAPFILEHEADER fileHeader;
	BITMAPINFOHEADER infoHeader;
	infoHeader.biSize = 40;
	infoHeader.biWidth = width;
	infoHeader.biHeight = height;
	infoHeader.biPlanes = 1;
	infoHeader.biBitCount = 24;
	infoHeader.biCompression = BI_RGB;
	infoHeader.biSizeImage = pView[2] * pView[3] * 3;
	infoHeader.biXPelsPerMeter = 0;
	infoHeader.biYPelsPerMeter = 0;
	infoHeader.biClrUsed = 0;
	infoHeader.biClrImportant = 0;
	fileHeader.bfType = 0x4D42;
	fileHeader.bfReserved1 = 0;
	fileHeader.bfReserved2 = 0;
	fileHeader.bfOffBits = 54;
	fileHeader.bfSize = (DWORD)(sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + pView[2] * pView[3] * 3);
	FILE *fd;
	if (!(fd = fopen("aaa.bmp", "wb+")))//filepath为你所保存文件的名字  
	{
		//AfxMessageBox("bmp图片申请出错");
		exit(0);
	}
	else
	{
		fwrite(&fileHeader, 1, sizeof(BITMAPFILEHEADER), fd);
		fwrite(&infoHeader, 1, sizeof(BITMAPINFOHEADER), fd);
		fwrite(data, 1, pView[2] * pView[3] * 3, fd);
		fclose(fd);
	}
	delete[] data;
	delete[] _data;
}
*/

void read3DPoints(string path, std::vector<laser> &pointSeq2)
{
	cout << path << endl;
	ifstream infile(path);
	if (!infile)
	{
		cout << "erreor" << endl;
	}

	float x = 0, y = 0, z = 0, intensity = 0;//, laserid = 0, angle = 0;
	while (!infile.eof()){
		infile >> x >> y >> z >> intensity;//>>laserid >> angle ;
		laser p3{ x,y,z,intensity,0 };//;laserid
		pointSeq2.push_back(p3);
	}

	infile.close();
	cout << path << " is write done" << endl; 
}

// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
	// Calculate rotation about x axis
	Mat R_x = (Mat_<float>(3, 3) <<
		1, 0, 0,
		0, cos(theta[2]), -sin(theta[2]),
		0, sin(theta[2]), cos(theta[2])
		);

	// Calculate rotation about y axis
	Mat R_y = (Mat_<float>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	Mat R_z = (Mat_<float>(3, 3) <<
		cos(theta[0]), -sin(theta[0]), 0,
		sin(theta[0]), cos(theta[0]), 0,
		0, 0, 1);


	// Combined rotation matrix
	Mat R = R_z * R_y * R_x;

	return R;

}

/*
	6 个自由度：x，y，z，roll，pitch，yaw
*/

float x(0), y(0), z(0), roll(0), pitch(0),yaw(0);

/*人工控制参数*/
void Manual_Control()
{
	//Matrix3f m1(3, 3);
	//m1 << 0.996860, 0.069389, 0.038153
	//	, 0.054332, -0.248847, -0.967018
	//	, -0.057606, 0.966054, -0.251836;

	//roll  = m1.eulerAngles(2, 1, 0).x() * 180 / PI;
	//pitch = m1.eulerAngles(2, 1, 0).y() * 180 / PI;
	//yaw	  = m1.eulerAngles(2, 1, 0).z() * 180 / PI;

	//x = -0.102771; y = 0.171601; z = -0.577584; 

	//Rot:[0.9998579, 0.015362915, -0.0069392053;
	//	-0.0065850243, -0.022986056, -0.99971408;
	//	-0.015518027, 0.99961776, -0.022881625]
	//	Tra : [0.19722891;
	//	-0.428399;
	//	-0.8775841]

	Matrix3f m1(3, 3);
	m1 << 0.9998579, 0.015362915, -0.0069392053
		, -0.0065850243, -0.022986056, -0.99971408
		, -0.015518027, 0.99961776, -0.022881625;

	yaw   = m1.eulerAngles(2, 1, 0).x() * 180 / PI;
	pitch = m1.eulerAngles(2, 1, 0).y() * 180 / PI;
	roll  = m1.eulerAngles(2, 1, 0).z() * 180 / PI;

	x = 0.19722891; y = -0.428399; z = -0.8775841;

	while (1)
	{
		//char* bu = vs->SubMsg("ControlInstruct");
		int c = _getch();
		cout << "keyboard_inputc：" << c << endl;
		showParam();
		//Angle_target = 90;
		switch (c)
		{
		/*x,y,z*/
		case 52: x += 0.1;  cout << "x + 0.1:" << endl; break;
		case 53: y += 0.1;	cout << "y + 0.1:" << endl; break;
		case 54: z += 0.1;  cout << "z + 0.1:" << endl; break;
		case 49: x -= 0.1;  cout << "x - 0.1:" << endl; break;
		case 50: y -= 0.1;	cout << "y - 0.1:" << endl; break;
		case 51: z -= 0.1;  cout << "z - 0.1:" << endl; break;

		/*roll,pitch,yaw*/
		case 113: roll += 0.1;	cout << "roll+5:" << endl; break;
		case 119: pitch += 0.1;	cout << "pitch+5:" << endl; break;
		case 101: yaw += 0.1;	cout << "yaw+5:" << endl; break;
		case 97: roll -= 0.1;	cout << "roll-5:" << endl; break;
		case 115: pitch -= 0.1;	cout << "pitch-5:" << endl; break;
		case 100: yaw -= 0.1;	cout << "yaw-5:" << endl; break;

		case 48:	cout << "nothing be done" << endl; break;
		
		/*next lidar-camera file pairs*/
		case 32: 
			file_suffix += 1;		
			if (file_suffix > 30)
				break;
			else
				EnterCriticalSection(&vector_cs);
				pointSeq2.clear();
			
				read3DPoints("D:/changshuloop/0.txt", pointSeq2);
				//read3DPoints(liadrPath + "/" + to_string(file_suffix) + ".txt", pointSeq);
				LeaveCriticalSection(&vector_cs);

			cout << "lidar-camera pairs(1-30):" << file_suffix << endl; break;

		case 27: bQuit = true; cout << "quit"; break;

		default:
			break;
		}
		EnterCriticalSection(&g_cs);
		Rot = eulerAnglesToRotationMatrix(Vec3f(yaw * RADIAN ,pitch *RADIAN ,roll *RADIAN));
		Tra.at<float>(0, 0) = x;
		Tra.at<float>(1, 0) = y;
		Tra.at<float>(2, 0) = z;

		showParam();

		LeaveCriticalSection(&g_cs);
		Sleep(5);
	}
}


//vector<CvPoint> CvPointseq;
vector<laserPro> CvPointseq;

void  myDisplay() {

	glClear(GL_COLOR_BUFFER_BIT);
	//画二维点云
	glColor3f(1.0f, 1.0f, 1.0f);
	glPointSize(1.0f);

	glBegin(GL_POINTS);
	for (auto &x : CvPointseq)
	{
		if (x.x > 0 && x.x <960 && x.y >0 && x.y < 1280)
		{
			int pixelthreshold = 105;
			if (x.intensity > pixelthreshold && !x.isBackground)//&&x.laserid != 63 && x.laserid != 36 && x.laserid != 38 && !x.isBackground)
			{
				glVertex2f(x.y, x.x);
			}
		}
	}
	glEnd();
	
	glFlush();// 刷新OpenGL命令队列
	CvPointseq.clear();
}

void myDisplay2(void)
{
	glClear(GL_COLOR_BUFFER_BIT);       //用当前背景色填充窗口
	glColor3f(1.0f, 0.0f, 0.0f);   //设置当前的绘制颜色为红色
	glRectf(50.0f, 100.0f, 150.0f, 50.0f);  //绘制一个矩形
	glFlush();     //清空OpenGL命令缓冲区，执行OpenGL程序
}

void maintthread(Mat *rota, Mat *trans)
{
	/*
	1、读三维点
	2、由三维计算二维
	3、流出6个参数接口，可以slider，可以键盘加减
	4、将二维像素点画在图像上
	*/
	//Matrix4d vv = LookAt(Vector3d(0, -40, 0), Vector3d(0, 0, 0), Vector3d(-0, 0, 1));
	Matrix4d vv = LookAt(Vector3d(0.6578295937305972, -40.42993121786281, 11.209376343062189), Vector3d(0, 0, 0), Vector3d(-0.07236996585583849, 0.26538139728622623, 0.9614235809550519));
	//Matrix4d vv = LookAt(Vector3d(0.40291490215479747, -24.808927311844965, 6.720616516837376), Vector3d(-0.04628709815793053, 2.850063990026231, -0.772068331881024), Vector3d(-0.05881400276310529, 0.2601283996888415, 0.9637811622740423));
	Matrix4d pp = Perspective(30, 1280/ 960.0, 0.1, 1000);

	while (true)
	{

		paramRead();

		getFiles(readpath_, binaryfilespath);
		float  px, py, pz, pr;
		for (auto binaryfile : binaryfilespath)
		{
			// allocate 4 MB buffer (only ~ 130*4*4 KB are needed)
			int32_t num = 1000000;
			float *data = (float*)malloc(num * sizeof(float));

			// pointers
			px = *(data + 0);
			py = *(data + 1);
			pz = *(data + 2);
			pr = *(data + 3);

			// load point cloud
			FILE *stream;
			stream = fopen(binaryfile.c_str(), "rb");
			num = fread(data, sizeof(float), num, stream) / 4;
			for (int32_t i = 0; i<num; i++) {

				px = *(data + 4 * i + 0);
				py = *(data + 4 * i + 1);
				pz = *(data + 4 * i + 2);
				pr = *(data + 4 * i + 3);
				points.push_back(tPoint(py, px, pz, pr));

			}
			free(data);
			
			for (auto &x : points)
			{
				//三维坐标
				Mat temppoint3 = Mat::ones(3, 1, CV_32FC1);
				temppoint3.at<float>(0, 0) = x.px;
				temppoint3.at<float>(1, 0) = x.py;
				temppoint3.at<float>(2, 0) = x.pz;

				int tensity = x.pr * 255;
				int laserid = 0;//x.laserid;
				float height_threshold = -1.2;
				int isbackground = 0;
				if (x.pz > height_threshold) {
					isbackground = 1;
				}
				EnterCriticalSection(&g_cs);

				Vector4d clipPos = pp*vv*Vector4d(x.px, x.py, x.pz, 1);
				Vector3d ndcPos0 = Vector3d(clipPos.x() / clipPos.w(), clipPos.y() / clipPos.w(), clipPos.z() / clipPos.w());


				int x = 0, y = 0, w = 1280, h = 960, n = 0.1, f = 1000;
				Vector3d screenPos0(w * 0.5f * ndcPos0.x() + x + w * 0.5f, h* 0.5f * ndcPos0.y() + y + h *0.5f, 0.5f *(f - n) * ndcPos0.z() + 0.5f * (f + n));;

				LeaveCriticalSection(&g_cs);

				laserPro topush;

				topush.x = screenPos0(1);
				topush.y = screenPos0(0);
				topush.intensity = tensity;
				topush.laserid = laserid;
				topush.isBackground = isbackground;
				CvPointseq.push_back(topush);
			}
			points.clear();
			glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
			glutInitWindowPosition(0, 0);//设置窗口左上角的位置  
			glutInitWindowSize(1280, 960);//设置窗口的宽高  

			int  f = binaryfile.find("0000");
			string window = "pointcloud" + binaryfile.substr(f, 9);

			int window_id = glutCreateWindow(window.c_str());

			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);    //设置窗口背景颜色为黑色 

													 //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
			init();
			glMatrixMode(GL_PROJECTION);       //指定设置投影参数
			gluOrtho2D(0.0, 1280.0, 0.0, 960.0);   //设置投影参数
			//index = i;

			glutDisplayFunc(&myDisplay);

			//glutKeyboardFunc(keyboard);

			glutMainLoopEvent();

			//glutDestroyWindow(window_id);

			//glutMainLoop(); // glut事件处理循环

			GLint viewPort[4] = { 0 };
			glGetIntegerv(GL_VIEWPORT, viewPort);



			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);//设置4字节对齐
			glReadBuffer(GL_FRONT);

			glReadPixels(viewPort[0], viewPort[1], viewPort[2], viewPort[3], GL_RGB, GL_UNSIGNED_BYTE, colorArr);

			string pngfilename = "D:\\KITTI velodynedata\\2011_09_29_drive_0004_sync\\2011_09_29\\2011_09_29_drive_0004_sync\\velodyne_points\\png\\" + binaryfile.substr(f, 10) + ".png";

			cout << "png filename :" << pngfilename << endl;

			saveColorData2img(colorArr, pngfilename);

			cout << "save " << pngfilename << endl;

			glutDestroyWindow(window_id);


		}

		/*
		char filename[100];
		memset(filename, 0, 100);
		string folder = "D:\\KITTI velodynedata\\2011_09_26_drive_0101_sync\\velodyne_points\\xyzi\\";
		string folderpath = "D:\\KITTI velodynedata\\2011_09_26_drive_0101_sync\\velodyne_points\\xyzi\\*";
		strcpy(filename, folder.c_str());
		GLsizei numComponet = 3;
		GLsizei bufferSize = 1280 * 960 * sizeof(GLfloat)*numComponet;
		unsigned char*  data = new unsigned char[bufferSize];

		GLfloat* _data = new GLfloat[bufferSize];
		vector<string> files = getFiles(folderpath);
		for (int i = 0; i<files.size(); i++)
		{
			strcat(filename, files[i].c_str());
			cout << filename << endl;

			read3DPoints(filename, pointSeq2);

			memset(filename, 0, 100);
			strcpy(filename, folder.c_str());

			EnterCriticalSection(&vector_cs);
			for (auto &x : pointSeq2)
			{
				//三维坐标
				Mat temppoint3 = Mat::ones(3, 1, CV_32FC1);
				temppoint3.at<float>(0, 0) = x.y;
				temppoint3.at<float>(1, 0) = x.x;
				temppoint3.at<float>(2, 0) = x.z;
		
				int tensity = x.intensity * 255;
				int laserid = x.laserid;
				float height_threshold = -1.2;
				int isbackground = 0;
				if (x.z > height_threshold) {
					isbackground = 1;
				}
				EnterCriticalSection(&g_cs);

				Vector4d clipPos = pp*vv*Vector4d(x.y, x.x, x.z, 1);
				Vector3d ndcPos0 = Vector3d(clipPos.x() / clipPos.w(), clipPos.y() / clipPos.w(), clipPos.z() / clipPos.w());


				int x = 0, y = 0, w = 1280, h = 960, n = 0.1, f = 1000;
				Vector3d screenPos0(w * 0.5f * ndcPos0.x() + x + w * 0.5f, h* 0.5f * ndcPos0.y() + y + h *0.5f, 0.5f *(f - n) * ndcPos0.z() + 0.5f * (f + n));;

				LeaveCriticalSection(&g_cs);

				laserPro topush;

				topush.x = screenPos0(1);
				topush.y = screenPos0(0);
				topush.intensity = tensity;
				topush.laserid = laserid;
				topush.isBackground = isbackground;
				CvPointseq.push_back(topush);
			}
			pointSeq2.clear();
			
			//glutInitDisplayMode(GLUT_DEPTH | GLUT_RGB | GLUT_SINGLE);
			glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
			glutInitWindowPosition(0, 0);//设置窗口左上角的位置  
			glutInitWindowSize(1280, 960);//设置窗口的宽高  
			string window = "pointcloud" + to_string(i);
			int window_id = glutCreateWindow(window.c_str());

			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);    //设置窗口背景颜色为黑色 

			//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
			init();
			glMatrixMode(GL_PROJECTION);       //指定设置投影参数
			gluOrtho2D(0.0, 1280.0, 0.0, 960.0);   //设置投影参数
			index = i;
			
			glutDisplayFunc(&myDisplay);

			//glutKeyboardFunc(keyboard);

			glutMainLoopEvent();
			
			//glutDestroyWindow(window_id);

			//glutMainLoop(); // glut事件处理循环

			/*GLint viewPort[4] = { 0 };
			glGetIntegerv(GL_VIEWPORT, viewPort);

			
			
			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);//设置4字节对齐  
			glReadBuffer(GL_FRONT);

			glReadPixels(viewPort[0], viewPort[1], viewPort[2], viewPort[3], GL_RGB, GL_UNSIGNED_BYTE, colorArr);
			
			string pngfilename = "D:\\KITTI velodynedata\\2011_09_26_drive_0101_sync\\velodyne_points\\png\\" + to_string(i) + ".png";
		
			cout << "png filename :" << pngfilename << endl;

			saveColorData2img(colorArr, pngfilename);

			cout << "save " << pngfilename << endl;*/
			
			/*glutDestroyWindow(window_id);
			
		}*/
		LeaveCriticalSection(&vector_cs);
		Sleep(1);
		cvReleaseImage(&pScr);
	}
}

void matinit()
{
	Rot.at<float>(0, 0) = 0.9998579;
	Rot.at<float>(0, 1) = 0.015362915;
	Rot.at<float>(0, 2) = -0.0069392053;
	Rot.at<float>(1, 0) = -0.0065850243;
	Rot.at<float>(1, 1) = -0.022986056;
	Rot.at<float>(1, 2) = -0.99971408;
	Rot.at<float>(2, 0) = -0.015518027;
	Rot.at<float>(2, 1) = 0.99961776;
	Rot.at<float>(2, 2) = -0.022881625;

	Tra.at<float>(0, 0) = 0.19722891;
	Tra.at<float>(1, 0) = -0.428399;
	Tra.at<float>(2, 0) = -0.8775841;


	//1.0e+03 *
	//1.5202         0    0.6262
	//0    1.5294    0.4631
	//0         0    0.0010
	Intri.at<float>(0, 0) = 1.5202 * 1000;
	Intri.at<float>(0, 1) = 0 * 1000;
	Intri.at<float>(0, 2) = 0.6262 * 1000;
	Intri.at<float>(1, 0) = 0 * 1000;
	Intri.at<float>(1, 1) = 1.5294 * 1000;
	Intri.at<float>(1, 2) = 0.4631 * 1000;
	Intri.at<float>(2, 0) = 0;
	Intri.at<float>(2, 1) = 0;
	Intri.at<float>(2, 2) = 0.001 * 1000;
}

void readcfgfile()
{
	cameraPath = configSettings2.Read("cameraPath", cameraPath);
	liadrPath = configSettings2.Read("liadrPath", liadrPath);
	ParamPath = configSettings2.Read("ParamPath", ParamPath);

	cout << "***Result of reading cfg file:*** " << endl;
	cout << "camera frame path	:" << cameraPath << endl;
	cout << "liadr  frame path	:" << cameraPath << endl;
	cout << "intri & extri Param path:" << ParamPath << endl;
	cout << endl;
}

void showParam()
{
	cout << " Rot:" << Rot << endl;
	cout << " Tra:" << Tra << endl;
}

void main(int argc, char *argv[])
{

	glutInit(&argc, argv);

	InitializeCriticalSection(&g_cs);
	InitializeCriticalSection(&vector_cs);
	//readcfgfile();
	//matinit(); 

	//thread manualcontrol(Manual_Control);

	thread maincontrol(maintthread, &Rot, &Tra);
	maincontrol.join();

}

