#include "lidar_control.hpp"
#include <iostream>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

// Конструктор класса
LidarControl::LidarControl(ros::NodeHandle &n, const std::string &uavName)
    : n_(n), lidarName_(uavName)
{
    rosNodeInit();
}

void LidarControl::rosNodeInit()
{
    // Инициализация подписки на топик камеры ЛА
    lidarInfo_ = n_.subscribe<sensor_msgs::LaserScan>(
        "/scan",
        1,
        &LidarControl::lidarInfoCallback,
        this);
    // подписка на топик с реальным положением БЛА
    localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose",
        1,
        &LidarControl::realPositionCallback,
        this);
    //  Публикация облака точек (PointCloud2)
    point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "/point_cloud",
        10);
}

// Обработчик положения БЛА
void LidarControl::realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal)
{
    currentPoseLocal_ = *currentPoseLocal;
}

// Обработчик полученной информации
void LidarControl::lidarInfoCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    lidarData_ = *msg;
}

void LidarControl::dataGetter()
{
    // Создаём облако точек PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "base_link";
    cloud->height = 1;
    cloud->width = lidarData_.ranges.size();
    cloud->is_dense = false;

    double current_time = ros::Time::now().toSec();
    static double last_time = current_time;
    double dt = current_time - last_time;
    last_time = current_time;

    // перебор значений
    for (size_t i = 0; i < lidarData_.ranges.size(); ++i)
    {
        // считываем значения
        double range = lidarData_.ranges[i];

        // Пропускаем невалидные значения (NaN, Inf, за пределами min/max)
        if (std::isnan(range) || std::isinf(range))
            continue;
        if (range < lidarData_.range_min || range > lidarData_.range_max)
            continue;

        // Угол текущей точки
        double angle = lidarData_.angle_min + i * lidarData_.angle_increment;

        // Определим координату Х
        double x = range * cos(angle);

        // Определим координату Y
        double y = range * sin(angle);

        tf::Vector3 point_local(x, y, 0.0);

        // заполняем структуру сырыми данными
        pcl::PointXYZ point;
        point.x = point_local.x();
        point.y = point_local.y();
        point.z = point_local.z();

        // добавляем точку в облако
        cloud->points.push_back(point);
    }

    // Обновляем width после фильтрации
    cloud->width = cloud->points.size();

    // Конвертируем в sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);

    // формируем header
    output.header.stamp = ros::Time::now();

    // задаем систему координат
    output.header.frame_id = cloud->header.frame_id;
    

    // Публикуем облако точек
    point_cloud_pub_.publish(output);
    //std::cout << "done" << std::endl;

    /*if (!cloud->points.empty())
    {
        ROS_INFO("First point is (meters): (%.2f, %.2f, %.2f)",
                 cloud->points[0].x,
                 cloud->points[0].y,
                 cloud->points[0].z);
    }*/
}
