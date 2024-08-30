#pragma warning(disable:4996)

#include <iostream>
#include <fstream>
#include <random>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>   
#include <pcl/filters/passthrough.h>
#include <mutex>
#include <pcl/common/common.h>

#include <pcl/visualization/cloud_viewer.h>

#include <ctime>


#include "utils.h"
#include "getTime.h"

using namespace std;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcds;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcds_aligned;

std::mutex mutexT;
std::string currentAxis = "x";
float mX = 0;
float mY = 0;
float mZ = 0;

float mRoll = 0;
float mYaw = 0;
float mPitch = 0;

float mStep = 1;

int mSelectedPcdId = 0;
Eigen::Matrix4f globalT = Eigen::Matrix4f::Identity();
Eigen::Matrix4f constructT()
{
	Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();

	ret.block<3, 1>(0, 3) = Eigen::Vector3f(mX, mY, mZ);

	Eigen::AngleAxisf rollAngle(mRoll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitchAngle(mPitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yawAngle(mYaw, Eigen::Vector3f::UnitZ());

	// �����ܵ���ת����ע��˳����ZYX��  
	Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
	ret.block<3, 3>(0, 0) = q.matrix();

	PCL_INFO("current dx,dy,dz:%f,%f,%f, current dr,dp,da:%f,%f,%f\n", mX, mY, mZ, mRoll, mPitch,mYaw);
	// globalT = ret;
	return ret;
}
void reset()
{
	mX = 0;
	mY = 0;
	mZ = 0;

	mRoll = 0;
	mYaw = 0;
	mPitch = 0;

	alignPcds(pcds_aligned, pcds, fillTransforms());
				std::unique_lock<std::mutex> lock(mutexT);
	globalT = constructT();
}


void apply(bool minus)
{
	float delta = minus ? -mStep : mStep;
	if ("x" == currentAxis) {
		mX += delta;
	}
	else if ("y" == currentAxis) {
		mY += delta;
	}
	else if ("z" == currentAxis) {
		mZ += delta;
	}
	else if ("r" == currentAxis) {
		mRoll += delta * 0.001;
	}
	else if ("a" == currentAxis) {
		mYaw += delta * 0.01;
	}
	else if ("p" == currentAxis) {
		mPitch += delta * 0.0005;
	}
	// std::unique_lock<std::mutex> lock(mutexT);

	globalT = constructT();
}

static int writeRts2Json()
{
	//todo: ...
	std::string configFileName = CONFIG_PATH;
	std::ifstream file(configFileName);
	if (!file.is_open()) {
		throw std::runtime_error("config file cannot open...");
	}
	std::stringstream buffer;
	buffer << file.rdbuf();

	file.close();
	cout << buffer.str() << endl;
	cJSON* root = cJSON_Parse(buffer.str().c_str());
	if (root == NULL)
	{
		PCL_ERROR("cannot create json\n");
		exit(-1);
	}

	cJSON* configsArr = cJSON_GetObjectItem(root, "RTs");

	cJSON* t3r3Arr = cJSON_GetArrayItem(configsArr, mSelectedPcdId);
	if (t3r3Arr) {
		cJSON_ReplaceItemInArray(t3r3Arr, 0, cJSON_CreateNumber(mX + cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 0))));
		cJSON_ReplaceItemInArray(t3r3Arr, 1, cJSON_CreateNumber(mY + cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 1))));
		cJSON_ReplaceItemInArray(t3r3Arr, 2, cJSON_CreateNumber(mZ + cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 2))));
		cJSON_ReplaceItemInArray(t3r3Arr, 3, cJSON_CreateNumber(mRoll + cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 3))));
		cJSON_ReplaceItemInArray(t3r3Arr, 4, cJSON_CreateNumber(mPitch + cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 4))));
		cJSON_ReplaceItemInArray(t3r3Arr, 5, cJSON_CreateNumber(mYaw + cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 5))));
	}

	char* str = cJSON_Print(root);
	if (str == NULL)
	{
		fprintf(stderr, "Failed to print root.\n");
	}
	else {
		PCL_INFO("===%s\n", str);
	}

	cJSON_Delete(root);

	std::ofstream outFile(configFileName);
	if (!outFile.is_open()) {
		std::cerr << "Unable to open file " << configFileName << std::endl;
		return 1; // ���ش������  
	}
	outFile << str;
	free(str);
	outFile.close();

	reset();
	return 0;
}
static void pointPicking(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	if(event.getPointIndex()==-1) return;
	float x,y,z;
	event.getPoint(x,y,z);
	cout<<"Point Picked"<<x<<","<<y<<","<<z<<","<<endl;
}
static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	if (event.keyDown())
	{
		//PCL_INFO("key info: %d\n", event.getKeyCode());
		{
			switch (event.getKeySym()[0])
			{
			case 'x':
			{ currentAxis = "x"; PCL_INFO("set axis x"); }
			break;
			case 'y':
			{ currentAxis = "y"; PCL_INFO("set axis y"); }
			break;
			case 'z':
			{ currentAxis = "z"; PCL_INFO("set axis z"); }
			break;
			case 'r':
			{ currentAxis = "r"; PCL_INFO("set rotate axis roll"); }
			break;
			case 'a':
			{ currentAxis = "a"; PCL_INFO("set rotate axis yaw"); }
			break;
			case 'p':
			{ currentAxis = "p"; PCL_INFO("set rotate axis pitch"); }
			break;
			case 'n':
				apply(true);
				break;
			case 'm':
				apply(false);
				break;
			case 'k':
				mStep /= 5;
				break;
			case 'l':
				mStep *= 5;
				break;
			case 'j':
			{
				mSelectedPcdId = (++mSelectedPcdId) % N;
				PCL_INFO("update mSelectedPcdId %d\n", mSelectedPcdId);
			}
			break;
			case 's':
				writeRts2Json();
				break;
			default:
				break;
			}
		}
	}
}

int main()
{
	int nProc = N;
	for (int i = 0; i < nProc; i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);

		if (!pcl::io::loadPCDFile((PCD_DIR+ to_string(i)+".pcd").c_str(), *pcd) == -1)
		{
			PCL_ERROR("load failed");
			return -1;
		}

		pcds.push_back(pcd);
	}


	Timer t2;
	//processing ...

	auto result = alignPcds(pcds_aligned, pcds, fillTransforms()); //
	cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>alignPcds time: " << t2.elapsed() << "s" << endl;

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	viewer->registerKeyboardCallback(keyboardEventOccurred, &viewer);
	viewer->registerPointPickingCallback(pointPicking);
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	viewer->setCameraPosition(0, 0, -1,  // Camera position
		0, 0, 0,   // Focal point
		0, 1, 0); // Up vector
	viewer->setBackgroundColor(0, 0, 0, v1);

	viewer->setWindowName("RTS ToOl");
	viewer->addCoordinateSystem(1.0);

	std::random_device rd;  // ��ȷ���������������  
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, 255);

	Eigen::Vector3f colors[N];
	for (int i = 0; i < N; ++i)
	{
		colors[i] = Eigen::Vector3f(dis(gen), dis(gen), dis(gen));
	}
	while (!viewer->wasStopped())
	{
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
			if (0 == mSelectedPcdId) {
				std::unique_lock<std::mutex> lock(mutexT);
				pcl::transformPointCloud(*pcds_aligned[0], *tmp, globalT);
				*result = *tmp;
			}
			else {
				*result = *pcds_aligned[0];
			}
		}

		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> vtmps;

		for (int i = 1; i < nProc; ++i)
		{
			if(i!=mSelectedPcdId) continue;
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
			if (i == mSelectedPcdId) {
				std::unique_lock<std::mutex> lock(mutexT);
				pcl::transformPointCloud(*pcds_aligned[i], *tmp, globalT);
			}
			else {
				*tmp = *pcds_aligned[i];
			}
			vtmps.push_back(tmp);
			*result += *tmp;
		}

		// û�е��Ƶ�ʱ��������ȴ�
		if (result->size() == 0)
			continue;

		viewer->removeAllShapes(v1);
		viewer->removePointCloud("cloud_in1");

		viewer->removeAllPointClouds();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(result, 255, 255, 20);
		viewer->addPointCloud(result, color1, "cloud_in1", v1);

		int i = 0;

		for (auto&& tmp : vtmps)
		{
			std::string tag = "facet" + std::to_string(i + 1);
			viewer->removePointCloud(tag.c_str());
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(tmp, colors[i].x(), colors[i].y(), colors[i++].z());
			viewer->addPointCloud(tmp, color2, tag.c_str(), v1);
		}

		viewer->spinOnce(10);
	}

	return 0;
}