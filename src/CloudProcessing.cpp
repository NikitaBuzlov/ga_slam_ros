/*
 * This file is part of GA SLAM.
 * Copyright (C) 2018 Dimitris Geromichalos,
 * Planetary Robotics Lab (PRL), European Space Agency (ESA)
 *
 * GA SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GA SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GA SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "CloudProcessing.h"

// GA SLAM
#include "TypeDefs.h"
#include "mapping/Map.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

// STL
#include <vector>

///ros
#include <ros/ros.h>

namespace ga_slam {

void CloudProcessing::processCloud(
        const Cloud::ConstPtr& inputCloud,
        Cloud::Ptr& outputCloud,
        std::vector<float>& cloudVariances,
        const Pose& robotPose,
        const Pose& mapToSensorTF,
        const MapParameters& mapParameters,
        double voxelSize,
        double depthSigmaCoeff1,
        double depthSigmaCoeff2,
        double depthSigmaCoeff3) {
    downsampleCloud(inputCloud, outputCloud, voxelSize);
    transformCloudToMap(outputCloud, mapToSensorTF);
    std::cout << outputCloud->size() << " output after transformcloudmap " << std::endl;
 //   cropCloudToMap(outputCloud, robotPose, mapParameters);
    std::cout << outputCloud->size() << " output after cropclludtomap " << std::endl;

    calculateCloudVariances(outputCloud, cloudVariances,
            depthSigmaCoeff1, depthSigmaCoeff2, depthSigmaCoeff3);
}
/*
void CloudProcessing::processCloud(const sensor_msgs::LaserScanConstPtr &inputCloud,
                                   sensor_msgs::LaserScanPtr &outputCloud,
                                   std::vector<float> &cloudVariances,
                                   const Pose &sensorToMapTF,
                                   const MapParameters &mapParameters,
                                   double voxelSize,
                                   double depthSigmaCoeff1,
                                   double depthSigmaCoeff2,
                                   double depthSigmaCoeff3)
{
    downsampleCloud(inputCloud, outputCloud, voxelSize);
    transformCloudToMap(outputCloud, mapToSensorTF);
    cropCloudToMap(outputCloud, robotPose, mapParameters);
    calculateCloudVariances(outputCloud, cloudVariances,
            depthSigmaCoeff1, depthSigmaCoeff2, depthSigmaCoeff3);
}
*/
void CloudProcessing::downsampleCloud(
        const Cloud::ConstPtr& inputCloud,
        Cloud::Ptr& outputCloud,
        double voxelSize) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*outputCloud);
    std::cout << outputCloud->size() << " size output " << inputCloud->size() << std::endl;
}

void CloudProcessing::transformCloudToMap(Cloud::Ptr& cloud, const Pose& tf) {
    if (tf.matrix().isIdentity()) return;

    pcl::transformPointCloud(*cloud, *cloud, tf);
}

void CloudProcessing::cropCloudToMap(
        Cloud::Ptr& cloud,
        const Pose& robotPose,
        const MapParameters& params) {
    const double positionZ = robotPose.translation().z();
    const float minPointX = params.positionX - params.length / 2;
    const float minPointY = params.positionY - params.length / 2;
    const float minElevation = positionZ + params.minElevation;
    const float maxPointX = params.positionX + params.length / 2;
    const float maxPointY = params.positionY + params.length / 2;
    const float maxElevation = positionZ + params.maxElevation;
    ROS_DEBUG_STREAM(positionZ << " position z " << minPointY << " min point y " << minPointX << " min point x " << minElevation << " min elevatro " << maxPointX );
    Eigen::Vector4f minCutoffPoint(minPointX, minPointY, minElevation, 0.);
    Eigen::Vector4f maxCutoffPoint(maxPointX, maxPointY, maxElevation, 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*cloud);
}

void CloudProcessing::calculateCloudVariances(
        const Cloud::ConstPtr& cloud,
        std::vector<float>& variances,
        double depthSigmaCoeff1,
        double depthSigmaCoeff2,
        double depthSigmaCoeff3) {
    variances.clear();
    variances.reserve(cloud->size());

    for (const auto& point : cloud->points) {
        const auto depth = Eigen::Vector3d(point.x, point.y, point.z).norm();
        const auto sigma = depthSigmaCoeff1 * (depth * depth) +
                depthSigmaCoeff2 * depth + depthSigmaCoeff3;

        variances.push_back(sigma * sigma);
    }
}

void CloudProcessing::convertMapToCloud(const Map& map, Cloud::Ptr& cloud) {
    cloud->clear();

    std::cout << " convert map to cloud" << std::endl;
    if (!map.isValid()) return;

    const auto params = map.getParameters();

    cloud->reserve(params.size * params.size);
    cloud->is_dense = true;
    cloud->header.stamp = map.getTimestamp();

    const auto& meanData = map.getMeanZ();
    Eigen::Vector3d point;

    std::cout << " convert map to cloud begin cycle" << std::endl;
    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        map.getPointFromArrayIndex(*it, meanData, point);
        cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }

    std::cout << " end cycle " << cloud->size()<< std::endl;
}

double CloudProcessing::matchClouds(
        const Cloud::ConstPtr& cloud1,
        const Cloud::ConstPtr& cloud2) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(1);
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);

    Cloud transformedCloud;
    icp.align(transformedCloud);

    return icp.getFitnessScore();
}

}  // namespace ga_slam

