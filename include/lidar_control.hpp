#ifndef newLidarControl_HPP
#define newLidarControl_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>


// #include <vector>

// Объявим класс для работы с лидаром коптера
class LidarControl
{
    // Приватные члены класса
private:
    ros::NodeHandle &n_;
    ros::Subscriber lidarInfo_;               // подписка на данные о лидаре
    std::string lidarName_;
    ros::Publisher  point_pub_;               // Публикатор точек 
    ros::Publisher  point_pub2_;              // Публикатор точек 
    ros::Publisher  point_cloud_pub_;         // Публикатор облака точек
    
    std::vector<float> x_recieved;            // координата Х 
    std::vector<float> y_recieved;            //координата У
    std::vector<float> angle_recieved;        //угол
    std::vector<float> valid_range;           //расстояние

    sensor_msgs::LaserScan lidarData_;        //данные от лидара для обработки

    //geometry_msgs::PoseStamped point2_msg;    //Объект сообщения для публикации в топике /points2
    //geometry_msgs::Pose point_msg;            //Объект сообщения для публикации в топике /points
    sensor_msgs::PointCloud2 mycloud;         //Объект сообщения для публикации облака точек в топике /point_cloud   
    geometry_msgs::PoseStamped currentPoseLocal_; // объект сообщения о положении и ориентации      
    ros::Subscriber localPositionSub_;
    
    
    struct Coordinate3d
    {
        double x, y, z;
    };

    void rosNodeInit();

  
 
public:
    LidarControl(ros::NodeHandle &n_, const std::string &lidarName = "mavros");
    void realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal);

    void lidarInfoCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void dataGetter();
};

#endif