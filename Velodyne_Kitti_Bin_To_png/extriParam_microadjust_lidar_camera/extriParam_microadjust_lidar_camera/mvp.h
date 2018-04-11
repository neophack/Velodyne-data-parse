#include <iostream>

#include "../Eigen/StdVector"
#include "../Eigen/Geometry"

#define Deg2Rad 3.1415926/180.0
using namespace Eigen;
using namespace std;

Eigen::Matrix4d LookAt( Eigen::Vector3d& eye,  Eigen::Vector3d& target,  Eigen::Vector3d& up)
{
	Vector3d z((eye - target).normalized());
	Vector3d x(z.cross(up).normalized());
	Vector3d y(x.cross(z));


	Matrix4d result;

	result(0,0) = x.x();
	result(1,0) = x.y();
	result(2,0) = x.z();
	result(3,0) = -x.dot(eye);

	result(0,1) = y.x();
	result(1,1) = y.y();
	result(2,1) = y.z();
	result(3,1) = -y.dot(eye);

	result(0,2) = z.x();
	result(1,2) = z.y();
	result(2,2) = z.z();
	result(3,2) = -z.dot(eye);

	result(0,3) = result(1,3) = result(2,3) = 0.0f;
	result(3,3) = 1.0f;
	return result.transpose();
}

Matrix4d Perspective(float fovy, float aspect, float zNear, float zFar)
{

	float tanHalfFovy = tan(Deg2Rad * fovy / 2);

	Matrix4d result = Matrix4d::Zero();

	result(0 , 0) = 1.0f / (aspect * tanHalfFovy);
	result(1 , 1) = 1.0f / (tanHalfFovy);
	result(2 , 3) = -1.0f;

	result(2 , 2) = -(zFar + zNear) / (zFar - zNear);
	result(3 , 2) = -(2.0f * zFar * zNear) / (zFar - zNear);
	return result.transpose();
}