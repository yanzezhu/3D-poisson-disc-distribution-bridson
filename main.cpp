// Copyright (c) 2019 Martyn Afford
// Licensed under the MIT licence
// Modified by Yanze on March 14, 2025

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <random>
#include <yaml-cpp/yaml.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "sample3d.h"

void visualizePointCloud(const std::vector<Point> &points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    Point bbox_min, bbox_max;
    bbox_max.x = -infinity;
    bbox_max.y = -infinity;
    bbox_max.z = -infinity;

    for (auto p : points)
    {
        // copy data to PCL point cloud object
        cloud->points.emplace_back(p.x, p.y, p.z);
        // compute bounding box
        bbox_max.x = std::max(bbox_max.x, p.x);
        bbox_max.y = std::max(bbox_max.y, p.y);
        bbox_max.z = std::max(bbox_max.z, p.z);
        bbox_min.x = std::min(bbox_min.x, p.x);
        bbox_min.y = std::min(bbox_min.y, p.y);
        bbox_min.z = std::min(bbox_min.z, p.z);
    }
    std::cout << "bounding box: [" << bbox_min.x << "," << bbox_min.y << "," << bbox_min.z << "]<-->[" << bbox_max.x << "," << bbox_max.y << "," << bbox_max.z << "]" << std::endl;

    cloud->width = points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // creating a visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Sample Results"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    // Keep running the visualization window
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main()
{
    // read the configurations from yaml file
    YAML::Node config = YAML::LoadFile("../config.yaml");
    float height = config["height"].as<float>();
    float width = config["width"].as<float>();
    float length = config["length"].as<float>();
    float min_distance = config["min_distance"].as<float>();
    int max_attempts = config["max_attempts"].as<int>();

    std::cout << "width = " << width << "; length = " << length << "; height = " << height << std::endl;

    auto seed = static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine engine{seed};
    std::uniform_real_distribution<float> distribution{0, 1};

    bridson::config conf;
    conf.xx = width;
    conf.yy = height;
    conf.zz = length;
    conf.min_distance = min_distance;
    conf.max_attempts = max_attempts;

    // set the start point to the center of the region
    conf.start.x = width / 2;
    conf.start.y = height / 2;
    conf.start.z = length / 2;

    auto start = std::chrono::high_resolution_clock::now();
    auto points = bridson::poisson_disc_distribution(
        conf,
        // random
        [&engine, &distribution](float range)
        {
            return distribution(engine) * range;
        },
        // in_area
        [width, height, length](Point p)
        {
            return p.x > 0 && p.x < width && p.y > 0 && p.y < height && p.z > 0 && p.z < length;
        });
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "\nSampling based on Poisson Disk method is done." << std::endl;
    std::cout << "Sample number = " << points.size() << "; sample time = " << duration.count() << "ms" << std::endl;

    visualizePointCloud(points); // visualization


    // comparison on uniform sampling
    YAML::Node config_uniform = YAML::LoadFile("../config_uniform.yaml");
    height = config_uniform["height"].as<float>();
    width = config_uniform["width"].as<float>();
    length = config_uniform["length"].as<float>();
    int sample_number = config_uniform["sample_number"].as<int>();

    uniform_comparison::config conf_uniform;
    conf_uniform.xx = width;
    conf_uniform.yy = height;
    conf_uniform.zz = length;
    conf_uniform.sample_number = sample_number;

    start = std::chrono::high_resolution_clock::now();
    auto points_uniform = uniform_comparison::uniform_distribution(
        conf_uniform,
        // random is an anonymous function
        [&engine, &distribution](float range)
        {
            return distribution(engine) * range;
        },
        // in_area
        [width, height, length](Point p)
        {
            return p.x > 0 && p.x < width && p.y > 0 && p.y < height && p.z > 0 && p.z < length;
        });
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "\nSampling based on Uniform Sampling method is done.\n";
    std::cout << "Sample number = " << points_uniform.size() << "; sample time = " << duration.count() << "ms" << std::endl;

    visualizePointCloud(points_uniform);
    return 0;
}