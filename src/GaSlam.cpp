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

#include "GaSlam.h"


// GA SLAM
#include "TypeDefs.h"


// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


// STL
#include <vector>
#include <mutex>
#include <future>

namespace ga_slam {

GaSlam::GaSlam(ros::NodeHandle& Node_handle)
        : poseEstimation_(),
          poseCorrection_(),
          dataRegistration_(),
          poseInitialized_(false),
          nh(Node_handle){


    double slopeSumThresholdMultiplier = 10.;
    double matchAcceptanceThreshold = 0.9;
    bool matchYaw = true;
    double matchYawRange = 20 * M_PI / 180;
    double matchYawStep = M_PI / 180;
    double globalMapLength = 300.;
    double globalMapResolution = 0.2;
    double mapLength;
    double mapResolution;
    double minElevation,maxElevation;
    double voxelSize;
    double depthSigmaCoeff1,depthSigmaCoeff2,depthSigmaCoeff3;
    int numParticles,resampleFrequency;
    double initialSigmaX,initialSigmaY,initialSigmaZ,initialSigmaYaw;
    double predictSigmaX,predictSigmaY,predictSigmaZ,predictSigmaYaw;
    priv_nh.param("mapLength",mapLength,100.0);
    priv_nh.param("mapResolution",mapResolution,0.2);
    priv_nh.param("minElevation",minElevation,-2.);
    priv_nh.param("maxElevation",maxElevation,2.);
    priv_nh.param("voxelSize",voxelSize,1.);
    priv_nh.param("depthSigmaCoeff1",depthSigmaCoeff1,1.);
    priv_nh.param("depthSigmaCoeff1",depthSigmaCoeff2,1.);
    priv_nh.param("depthSigmaCoeff1",depthSigmaCoeff3,1.);
    priv_nh.param("numParticles",numParticles,1);
    priv_nh.param("resampleFrequency",resampleFrequency,1);
    priv_nh.param("initialSigmaX",initialSigmaX,0.);
    priv_nh.param("initialSigmaY",initialSigmaY,0.);
    priv_nh.param("initialSigmaZ",initialSigmaZ,0.);
    priv_nh.param("initialSigmaZ",initialSigmaYaw,0.);
    priv_nh.param("initialSigmaX",predictSigmaX,0.);
    priv_nh.param("initialSigmaY",predictSigmaY,10.);
    priv_nh.param("initialSigmaZ",predictSigmaZ,10.);
    priv_nh.param("initialSigmaZ",predictSigmaYaw,0.5);
    priv_nh.param("slopeSumThresholdMultiplier",slopeSumThresholdMultiplier,10.0);
    priv_nh.param("matchAcceptanceThreshold",matchAcceptanceThreshold,10.0);
    priv_nh.param("traversedDistanceThreshold",matchYaw,true);
    priv_nh.param("traversedDistanceThreshold",matchYawRange,10.0);
    priv_nh.param("traversedDistanceThreshold",matchYawStep, M_PI / 180);
    priv_nh.param("globalMapLength",globalMapLength, 300.0);
    priv_nh.param("traversedDistanceThreshold",globalMapResolution,1.0);


    configure(mapLength, mapResolution, minElevation,maxElevation, voxelSize,
              depthSigmaCoeff1, depthSigmaCoeff2, depthSigmaCoeff3,
              numParticles, resampleFrequency,
              initialSigmaX, initialSigmaY, initialSigmaZ, initialSigmaYaw,
                    predictSigmaX, predictSigmaY, predictSigmaZ, predictSigmaYaw,
                    slopeSumThresholdMultiplier, matchAcceptanceThreshold,
                    matchYaw, matchYawRange, matchYawStep,
                    globalMapLength, globalMapResolution);

    Cloud::Ptr global_map_cloud (new Cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/nikita/maps_save/map.pcd", *global_map_cloud);
    createGlobalMap(global_map_cloud, Pose::Identity());
    cloudSub = nh.subscribe("velodyne_points",10,&GaSlam::cloudCallback,this);
    odoSub = nh.subscribe("/odometry/filtered_map",10,&GaSlam::poseCallback,this);
    imuSub = nh.subscribe("/imu_rob_loc",10,&GaSlam::imuCallback,this);
    pubPose = nh.advertise<nav_msgs::Odometry>("/pose_ga",10);
}
void GaSlam::configure(
        double mapLength, double mapResolution,
        double minElevation, double maxElevation, double voxelSize,
        double depthSigmaCoeff1, double depthSigmaCoeff2,
        double depthSigmaCoeff3, int numParticles, int resampleFrequency,
        double initialSigmaX, double initialSigmaY, double initialSigmaYaw,
        double predictSigmaX, double predictSigmaY, double predictSigmaYaw,
        double traversedDistanceThreshold, double minSlopeThreshold,
        double slopeSumThresholdMultiplier, double matchAcceptanceThreshold,
        bool matchYaw, double matchYawRange, double matchYawStep,
        double globalMapLength, double globalMapResolution) {
    voxelSize_ = voxelSize;
    depthSigmaCoeff1_ = depthSigmaCoeff1;
    depthSigmaCoeff2_ = depthSigmaCoeff2;
    depthSigmaCoeff3_ = depthSigmaCoeff3;



    poseEstimation_.configure(numParticles, resampleFrequency,
            initialSigmaX, initialSigmaY, initialSigmaYaw,
            predictSigmaX, predictSigmaY, predictSigmaYaw);

    poseCorrection_.configure(traversedDistanceThreshold, minSlopeThreshold,
            slopeSumThresholdMultiplier, matchAcceptanceThreshold,
            matchYaw, matchYawRange, matchYawStep,
            globalMapLength, globalMapResolution);

    dataRegistration_.configure(mapLength, mapResolution, minElevation,
            maxElevation);
}

void GaSlam::poseCallback(const nav_msgs::Odometry/*Pose*/& odometryDeltaPose) {
    if (!poseInitialized_) poseInitialized_ = true;

    poseEstimation_.predictPose(odometryDeltaPose);
    dataRegistration_.translateMap(getPose());
}

void GaSlam::imuCallback(const sensor_msgs::Imu&/* Pose&*/ imuOrientation) {
    if (!poseInitialized_) return;

    poseEstimation_.fuseImuOrientation(imuOrientation);
}

void GaSlam::cloudCallback(
        const Cloud::ConstPtr& cloud,
        const Pose& bodyToSensorTF) {
    if (!poseInitialized_) return;

    const auto mapToSensorTF = getPose() * bodyToSensorTF;
    const auto mapParameters = getLocalMap().getParameters();
    Cloud::Ptr processedCloud(new Cloud);
    std::vector<float> cloudVariances;

    CloudProcessing::processCloud(cloud, processedCloud, cloudVariances,
            getPose(), mapToSensorTF, mapParameters, voxelSize_,
            depthSigmaCoeff1_, depthSigmaCoeff2_, depthSigmaCoeff3_);
    dataRegistration_.updateMap(processedCloud, cloudVariances);


    if (isFutureReady(scanToMapMatchingFuture_))
        scanToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToRawCloud, this, processedCloud);

    if (isFutureReady(mapToMapMatchingFuture_))
        mapToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToGlobalMap, this);
}
void GaSlam::cloudCallback(const sensor_msgs::PointCloud2Ptr &cloud)
{
    Cloud::Ptr pcl_cloud(new Cloud);
  //  pcl::PCLPointCloud2 l;
    pcl::fromROSMsg(*cloud,*pcl_cloud);
    const Pose mapToSensorTF = getPose();// * bodyToSensorTF;
    const auto mapParameters = getLocalMap().getParameters();
    Cloud::Ptr processedCloud(new Cloud);
    std::vector<float> cloudVariances;
    ROS_DEBUG_STREAM("begin process cloud" <<pcl_cloud->size());
    CloudProcessing::processCloud(pcl_cloud, processedCloud, cloudVariances,
            getPose(), mapToSensorTF, mapParameters, voxelSize_,
            depthSigmaCoeff1_, depthSigmaCoeff2_, depthSigmaCoeff3_);

    ROS_DEBUG_STREAM("process cloud is finished" << processedCloud->size() << " cloud variance " << cloudVariances.size());
    dataRegistration_.updateMap(processedCloud, cloudVariances);

    ROS_DEBUG_STREAM("map_update is finished" << processedCloud->size());

    if (isFutureReady(scanToMapMatchingFuture_))
        scanToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToRawCloud, this, processedCloud);

    if (isFutureReady(mapToMapMatchingFuture_))
        mapToMapMatchingFuture_ = std::async(std::launch::async,
                &GaSlam::matchLocalMapToGlobalMap, this);


 ;
tf::Transform transform;
tf::transformEigenToTF(poseEstimation_.getPose(),transform);
nav_msgs::Odometry odomsg;
tf::poseTFToMsg(transform,odomsg.pose.pose);
odomsg.header.frame_id = "/map";
odomsg.header.stamp = ros::Time().now();
pubPose.publish(odomsg);
}

void GaSlam::createGlobalMap(
            const Cloud::ConstPtr& globalCloud,
            const Pose& globalCloudPose) {
    poseCorrection_.createGlobalMap(globalCloud, globalCloudPose);
}

void GaSlam::matchLocalMapToRawCloud(const Cloud::ConstPtr& rawCloud) {
    Cloud::Ptr mapCloud(new Cloud);

    std::unique_lock<std::mutex> guard(getLocalMapMutex());
    const auto& map = getLocalMap();
    ROS_DEBUG_STREAM("convert map" << map.getParameters().size);
    CloudProcessing::convertMapToCloud(map, mapCloud);
    guard.unlock();
    ROS_DEBUG_STREAM(rawCloud->size() << " size raw data " << mapCloud->size());
    poseEstimation_.filterPose(rawCloud, mapCloud);
}

void GaSlam::matchLocalMapToGlobalMap(void) {
    const auto currentPose = getPose();
    if (!poseCorrection_.distanceCriterionFulfilled(currentPose)) return;

    std::unique_lock<std::mutex> guard(getLocalMapMutex());
    const auto& map = getLocalMap();
    if (!poseCorrection_.featureCriterionFulfilled(map)) return;

    Pose correctionDeltaPose;
    const bool matchFound = poseCorrection_.matchMaps(map, currentPose,
            correctionDeltaPose);
    guard.unlock();

    if (matchFound) poseEstimation_.predictPose(correctionDeltaPose);
}

}  // namespace ga_slam

