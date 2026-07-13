#ifndef vFH2D_HPP
#define vFH2D_HPP
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class VFH2dPlanner
{
public:
    explicit VFH2dPlanner(ros::NodeHandle &nh);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void computeLocalGoal(geometry_msgs::PoseStamped &goal_out);

private:
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_goal_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int num_sectors_ = 360;              // число секторов
    double min_sector_angle_ = 0.0;      // начало сектора (рад)
    double max_sector_angle_ = 2 * M_PI; // конец сектора
    double safety_radius_ = 1.0;         // минимальный безопасный радиус (м)
    double lookahead_distance_ = 2.0;    // дистанция до локальной цели (м)

    std::vector<double> sector_cost_; // стоимость каждого сектора

    void buildPolarHistogram(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    int selectBestSector() const;
    geometry_msgs::PoseStamped makeGoalFromSector(int sector_idx);
};

#endif