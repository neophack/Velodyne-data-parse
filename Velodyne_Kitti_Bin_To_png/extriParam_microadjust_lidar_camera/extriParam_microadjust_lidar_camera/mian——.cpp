#include <iostream>
#include <opencv2\opencv.hpp>
#include <fstream>
#include <thread>
#include <windows.h>
#include<stdio.h>
#include<conio.h>
#include <math.h>
#include <../Eigen/Dense>  

#include "Config.h" 
#include "mvp.h" 

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace cv;


#define RADIAN 3.14159265358979 / 180
#define DEGREE  180 / 3.1415926 
#define PI  3.1415926 

void showParam();

CRITICAL_SECTION g_cs;
CRITICAL_SECTION vector_cs;

//% Transformation matrix specifies laser coordinate frame
//% in the reference frame of the camera
//%--Translation vector(t)
//t = [-0.102771; 0.171601; -0.577584]
//% --Rotation matrix(R)
//R = ...
//[0.996860  0.069389  0.038153; ...
//0.054332 - 0.248847 - 0.967018; ...
//- 0.057606  0.966054 - 0.251836]

//intri param
//1.0e+03 *
//1.5202         0    0.6262
//0    1.5294    0.4631
//0         0    0.0010

Mat Rot = Mat::ones(3, 3, CV_32FC1);
Mat Tra = Mat::ones(3, 1, CV_32FC1);
Mat Intri = Mat::ones(3, 3, CV_32FC1);
Mat pixel = Mat::ones(3, 1, CV_32FC1);
Mat cameraCoor = Mat::ones(3, 1, CV_32FC1);

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

const char ConfigFile[] = "configsetup.txt";
Config configSettings(ConfigFile);

void read3DPoints(string path, std::vector<Point3f> &pointSeq)
{
	cout << path << endl;
	ifstream infile(path);
	if (!infile)
	{
		cout << "erreor" << endl;
	}

	float x = 0, y = 0, z = 0;
	while (!infile.eof()){
		infile >> x >> y >> z;
		Point3f p3{x,y,z};
		pointSeq.push_back(p3);
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


Matrix4f ViewMatrix2(const Vector3f& eye, const Vector3f& target, const Vector3f& up)
{
	Vector3f z((eye - target).normalized());
	Vector3f x((z.cross(up).normalized()));
	Vector3f y(x.cross(z));

	Matrix4f result;

	result(0, 0) = x(0);
	result(1, 0) = x(1);
	result(2, 0) = x(2);
	result(3, 0) = -x.dot(eye);

	result(0, 1) = y(0);
	result(1, 1) = y(1);
	result(2, 1) = y(2);
	result(3, 1) = -(y.dot(eye));

	result(0, 2) = z(0);
	result(1, 2) = z(1);
	result(2, 2) = z(2);
	result(3, 2) = -(z.dot(eye));

	result(0, 3) = result(1, 3) = result(2, 3) = 0.0f;
	result(3, 3) = 1.0f;
	return result.transpose();
}

/*view相机矩阵，输入：
Vector3f eye(x,y,z)
Vector3f target(0,0,0)
Vector3f up()
*/
Matrix4f ViewMatrix(const Vector3f& eye, const Vector3f& target, const Vector3f& up)
{
	Vector3f f((target - eye).normalized());
	Vector3f u(up.normalized());
	Vector3f s((f.cross(u).normalized()));
	u = s.cross(f);

	Matrix4f result = Matrix4f::Zero(4, 4);

	result(0, 0) = s(0);
	result(1, 0) = s(1);
	result(2, 0) = s(2);
	result(3, 0) = -(s.dot(eye));

	result(0, 1) = u(0);
	result(1, 1) = u(1);
	result(2, 1) = u(2);
	result(3, 1) = -(u.dot(eye));

	result(0, 2) = -f(0);
	result(1, 2) = -f(1);
	result(2, 2) = -f(2);
	result(3, 2) = (f.dot(eye));

	result(0, 3) = result(1, 3) = result(2, 3) = 0.0f;
	result(3, 3) = 1.0f;
	return result.transpose();
}
//投影矩阵，输入垂直视角fov度
Matrix4f projectionMatrix(float fovy, float aspect, float zNear, float zFar)
{

	float tanHalfFovy = tan(RADIAN * fovy / 2);

	Matrix4f result = Matrix4f::Zero(4, 4);
	result(0,0) = 1.0f / (aspect * tanHalfFovy);
	result(1,1) = 1.0f / (tanHalfFovy);
	result(2,3) = -1.0f;

	result(2,2) = -(zFar + zNear) / (zFar - zNear);
	result(3,2) = -(2.0f * zFar * zNear) / (zFar - zNear);
	return result.transpose();
}

//调用 projectionMatrix * ViewMatrix * vec4( position, 1.0 )



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
				pointSeq.clear();
				read3DPoints(liadrPath + "/" + to_string(file_suffix) + ".txt", pointSeq);
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
/*
Matrix4f setColumn(int index, const float col[4],Matrix4f &m)
{
	m[index * 4] = col[0];  m[index * 4 + 1] = col[1];  m[index * 4 + 2] = col[2];  m[index * 4 + 3] = col[3];
	return m;
}*/

void maintthread(Mat *rota, Mat *trans)
{
	/*
	1、读三维点
	2、由三维计算二维
	3、流出6个参数接口，可以slider，可以键盘加减
	4、将二维像素点画在图像上
	*/
	while (true)
	{
		//load image
		//read3DPoints(liadrPath + "/" + to_string(file_suffix) + ".txt", pointSeq);
		pScr = cvLoadImage(string(cameraPath + "/left" + to_string(file_suffix) + ".bmp").c_str(), 1);
		if (pScr->imageSize == 0)
		{
			cout << "图片加载错误" << endl;
			system("pause");
		}

		//imshow("sky", img);
		vector<CvPoint> CvPointseq;

		EnterCriticalSection(&vector_cs);
		for (auto &x : pointSeq)
		{
			Mat temppoint3 = Mat::ones(3, 1, CV_32FC1);
			Vector4d temppoint4;
			temppoint4(0) = temppoint3.at<float>(0, 0) = x.x;
			temppoint4(1) = temppoint3.at<float>(1, 0) = x.y;
			temppoint4(2) = temppoint3.at<float>(2, 0) = x.z;
			temppoint4(3) = 1;

			EnterCriticalSection(&g_cs);

	
			Vector3d  eye(0.40291490215479747, -24.808927311844965, 6.720616516837376);
			Vector3d  target(-0.04628709815793053, 2.850063990026231, -0.772068331881024);
			Vector3d   up(-0.05881400276310529, 0.2601283996888415, 0.9637811622740423);
			Matrix4d p = Perspective_(30, 1280 / 960.0, 0.3, 100);
			Matrix4d v = LookAt(eye, target, up);

			//Vector4d clipPos0 = p*v* temppoint4;

			//Vector3d ndcPos0 = Vector3d(clipPos0.x() / clipPos0.w(), clipPos0.y() / clipPos0.w(), clipPos0.z() / clipPos0.w());

			//int x = 0, y = 0, w = 1280, h = 960, n = 0.3, f = 100;
			//Vector3d screenPos0(w * 0.5f * ndcPos0.x() + x + w * 0.5f, h* 0.5f * ndcPos0.y() + y + h *0.5f, 0.5f *(f - n) * ndcPos0.z() + 0.5f * (f + n));


			Vector4d clipPos = p*v*Vector4d(x.x, x.y, x.z, 1);

			//cout << " pp*vv: " << pp*vv << endl;

			Vector3d ndcPos0 = Vector3d(clipPos.x() / clipPos.w(), clipPos.y() / clipPos.w(), clipPos.z() / clipPos.w());


			int x = 0, y = 0, w = 1280, h = 960, n = 0.3, f = 100;
			Vector3d screenPos0(w * 0.5f * ndcPos0.x() + x + w * 0.5f, h* 0.5f * ndcPos0.y() + y + h *0.5f, 0.5f *(f - n) * ndcPos0.z() + 0.5f * (f + n));

			//cout << screenPos0 << endl;
			//system("pause");



			LeaveCriticalSection(&g_cs);

			//pixel = (1.0 / cameraCoor.at<float>(2, 0)) * Intri * (cameraCoor);

			CvPoint topush;
			topush.x = screenPos0(0);
			topush.y = screenPos0(1);

			CvPointseq.push_back(topush);
		}

		LeaveCriticalSection(&vector_cs);

		for (auto &x : CvPointseq)
		{
			if (x.x > 0 && x.x <1280 && x.y >0 && x.y < 960)
			{
				//cout << x.x << " " << x.y << endl;
				cvCircle(pScr, x, 1, CV_RGB(0, 0, 255), -1, 8, 3);
			}
		}

		cout << "image show again..." << endl;
		cvNamedWindow("example", CV_WINDOW_AUTOSIZE);
		cvShowImage("example", pScr);
		//if (showCtl)
		//{
		//	waitKey();
		//}
		waitKey(1);

		//cout << "dddddddd" << endl;

		Sleep(20);

		cvReleaseImage(&pScr);
	}
}

void matinit()
{
	//Rot.at<float>(0, 0) = 0.996860;
	//Rot.at<float>(0, 1) = 0.069389;
	//Rot.at<float>(0, 2) = 0.038153;
	//Rot.at<float>(1, 0) = 0.054332;
	//Rot.at<float>(1, 1) = -0.248847;
	//Rot.at<float>(1, 2) = -0.967018;
	//Rot.at<float>(2, 0) = -0.057606;
	//Rot.at<float>(2, 1) = 0.966054;
	//Rot.at<float>(2, 2) = -0.251836;

	//Tra.at<float>(0, 0) = -0.102771;
	//Tra.at<float>(1, 0) = 0.171601;
	//Tra.at<float>(2, 0) = -0.577584;

	//Rot:[0.9998579, 0.015362915, -0.0069392053;
	//	-0.0065850243, -0.022986056, -0.99971408;
	//	-0.015518027, 0.99961776, -0.022881625]
	//	Tra : [0.19722891;
	//	-0.428399;
	//	-0.8775841]

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

	read3DPoints(liadrPath + "/" + to_string(file_suffix) + ".txt", pointSeq);

	system("pause");
}

void readcfgfile()
{
	cameraPath = configSettings.Read("cameraPath", cameraPath);
	liadrPath = configSettings.Read("liadrPath", liadrPath);
	ParamPath = configSettings.Read("ParamPath", ParamPath);

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

void main()
{
	InitializeCriticalSection(&g_cs);
	InitializeCriticalSection(&vector_cs);
	readcfgfile();
	matinit(); 

	thread manualcontrol(Manual_Control);
	thread maincontrol(maintthread, &Rot, &Tra);
	ofstream out("res.txt");
	while (true)
	{
		if (bQuit)
		{
			out << "Rot:" << Rot << "\n Tra:" << Tra << "\n";
			int i = 1;
			break;
		}
	}
	out.close();
	system("pause");
}

