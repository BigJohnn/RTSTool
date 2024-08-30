
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>   
#include<pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/octree/octree.h>
#include "cJSON.h"
#define N 4 //How many pcds do you have?
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr pcdPreprocess(PointCloud::Ptr const& input)
{
	PointCloud::Ptr cloud_filtered(new PointCloud);
	for (int i = 0; i < input->points.size(); i++)
	{
		pcl::PointXYZ point;
		if (input->points[i].y < 5.0)
		{
			point.x = input->points[i].x;
			point.y = input->points[i].y;
			point.z = input->points[i].z;
		}
		cloud_filtered->push_back(point);
		
	}
	cloud_filtered->width = (int)cloud_filtered->points.size();
	cloud_filtered->height = 1;
	//TODO: coordinate scales to 1m

	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	float res = 0.40f; //若点云尺度已经缩放到1m，每个格子20cm
	voxel_filter.setInputCloud(cloud_filtered);
	voxel_filter.setLeafSize(res, res, res);

	// downsampling
	PointCloud::Ptr downsampled_cloud(new PointCloud);
	voxel_filter.filter(*downsampled_cloud);
	cout << "+++++++++++++before/after downsample points: " << cloud_filtered->points.size() << "," << downsampled_cloud->points.size() << endl;

	PointCloud::Ptr ret(new PointCloud);
	ret->swap(*downsampled_cloud);

	return ret;
}

PointCloud::Ptr alignPcds(std::vector<PointCloud::Ptr>& out_pcds_aligned, std::vector<PointCloud::Ptr> const& pcds, std::vector<Eigen::Matrix4f> const& RTs)
{
	PointCloud::Ptr ret(new PointCloud);
	for (int i = 0; i < pcds.size(); ++i)
	{
		auto&& pcd = pcds[i];
		auto&& RT = RTs[i];
		if(out_pcds_aligned.size()!=pcds.size()) {
			out_pcds_aligned.push_back(PointCloud::Ptr(new PointCloud));

		}
		pcl::transformPointCloud(*pcd, *(out_pcds_aligned[i]), RT);
		*ret += *pcd;
	}
	return ret;
}

void updateMat(Eigen::Matrix4f& input, float x, float y, float z, float roll, float pitch, float yaw)
{
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

	// 计算总的旋转矩阵（注意顺序是ZYX）  
	Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
	input.block<3, 3>(0, 0) = q.matrix();
	input.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
}

std::vector<Eigen::Matrix4f> fillTransforms()
{
	std::vector<Eigen::Matrix4f> ret(8, Eigen::Matrix4f::Identity());
	
	std::string configFileName = CONFIG_PATH;
	std::ifstream file(configFileName);
	if (!file.is_open()) {
		throw std::runtime_error("无法打开文件");
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

	for (int i = 0; i < N; ++i) {
		cJSON* t3r3Arr = cJSON_GetArrayItem(configsArr, i);
		if (t3r3Arr) {
			updateMat(ret[i], 
				
				cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 0)),
				cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 1)),
				cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 2)),
				cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 3)),
				cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 4)),
				cJSON_GetNumberValue(cJSON_GetArrayItem(t3r3Arr, 5))
			);
		}
	}
	
	return ret;
}