#pragma once

#include "BaseSystem.h"
#include <Eigen/Dense>
#include <cmath>
#include<Eigen/StdVector>
#include <iostream>

#define SCALE 2.75

enum Color { Green, Red };

struct Skeleton
{
	// 3 translational at the root
	Vector torso_T;
	// 3 rotational at the shoulder : (first around x, then y, then z)
	Vector shoulder_T;
	Vector shoulder_R; // x,y,z
	// 2 rotational at the elbow(first around x then around y)
	Vector elbow_T;
	Vector elbow_R; // x, y
	// 2 rotational at the wrist(y, z) : first around y then around z * /
	Vector wrist_T;
	Vector wrist_R; // y,z
	// For resting position of other arm (could be expanded to support IK if needed)
	Vector shoulder_L;
	Vector elbow_L;
	// P hand
	Vector phand;
	Vector currentP;
	Vector targetP;
	int NUM_JOINTS = 3; // 3 joints in the arm

	struct Joint {
		Eigen::Vector3d position;    // 关节的位置
		Eigen::Quaterniond orientation; // 关节的旋转方向，使用四元数表示

		// 构造函数，初始化orientation为单位四元数
		Joint() : orientation(Eigen::Quaterniond::Identity()) {}
	};
	std::vector<Joint, Eigen::aligned_allocator<Joint>> joints; //绷不住了，这tm是什么东西


	Skeleton() {
		zeroVector(torso_T);
		setVector(shoulder_T, 0.55 * SCALE, 1.1 * SCALE, 0);
		zeroVector(shoulder_R);
		setVector(elbow_T, 1.3 * SCALE, 0, 0); //这个是距离前一个关节的距离
		zeroVector(elbow_R);
		setVector(wrist_T, 1.2 * SCALE, 0, 0);
		zeroVector(wrist_R);

		zeroVector(currentP);
		zeroVector(targetP);

		setVector(phand, 4, 0, 0); // 1 added in matrix calculation

		zeroVector(shoulder_L);
		zeroVector(elbow_L);

		// starting distance from blackboard for Bob
		torso_T[0] = -2;
		torso_T[1] = -3;
		torso_T[2] = 7.5;

		joints.resize(NUM_JOINTS);
	}
};


class Bob : public BaseSystem
{

public:
	Bob(const std::string& name);

	Skeleton* skel;
	bool flag = false;

	~Bob() {
		delete skel;
	}

	void getState(double* p);
	void setState(double* p);

	void reset(double time);
	void display(GLenum mode = GL_RENDER);

	int command(int argc, myCONST_SPEC char** argv);

	void drawEllipse(double x, double y, Color color);
	void glRotate3D(double x, double y, double z);

	Eigen::Matrix4d getTranslation(Vector position);
	Eigen::Matrix4d getRotationX(double theta);
	Eigen::Matrix4d getRotationY(double theta);
	Eigen::Matrix4d getRotationZ(double theta);
	Eigen::Matrix4d getRotationXDerivative(double theta);
	Eigen::Matrix4d getRotationYDerivative(double theta);
	Eigen::Matrix4d getRotationZDerivative(double theta);

	void setInitAngle();
	void calculateCurrentP(const bool a = false);
	void Pseudo_Inverse_IK();
	void CCD_IK();
};