#ifndef avoidanceSystem_HPP
#define avoidanceSystem_HPP

#include "flight_controller.hpp"
#include "lidar_control.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "map_viz.hpp"
#include <visualization_msgs/MarkerArray.h>

struct coords
{
    double x;
    double y; // красный
    double z; // зелёный

};

class Avoidance
{
//Открытые члены класса
public:
    //конструктор класса    
    Avoidance(
        ros::NodeHandle &n,
        const std::string &uavName = "mavros");

    bool processLidarAndPlan();

    //Метод управления процессом взлета
	bool takeoff(double target_altitude);

    //Метод управления процессом возврата
	bool go_to_base();

    //Метод управления процессом посадки
	bool land();


    // метод для арминга дрона. На вход принимает команду True/False
	bool arm(bool cmd);

    //Метод управления процессом полета до точки
	bool go_to_target(double target_x, double target_y, double target_z);
        
    //проверка соединения
    bool isConnected() const;

    //Выбор сектора для полета
    int selectBestSector();

    geometry_msgs::PoseStamped makeLocalGoal(int sector_idx);

    //Метод, зля задания конечной точки полета извне
    void setGlobalGoal(double x, double y, double z);

    bool getLastLocalGoal(geometry_msgs::PoseStamped& out_goal) const;

    void lidarDataGetter();

    bool go_to_global(double x, double y, double z);

    void target_pub(double x, double y, double z);

    void map_update();

    // Вспомогательный метод
    void publishVfhHistogram();

    void testFakeScan();

    void testVis();

    void hover ();

    double set_distance(int sector_idx);

    bool isGoalSafe(const geometry_msgs::PoseStamped &goal, double safety_margin) const;



// Закрытые члены класса
private:
    // имя ноды
    std::string uavName_;

    // информация об исполняемой ноде
    ros::NodeHandle &n_;

    // Экземпляр класса для управления лидаром
    LidarControl lidarControl;

    //класс для визуализации
    VoxelVisualizer visualizer;

    // Экземпляр класса для управления БЛА
    uav_controller::UavController flightControl;

    // Объект сообщения о состоянии аппарата
    mavros_msgs::State currentState_;

    // Объект сообщения о положении и ориентации
    geometry_msgs::PoseStamped currentPoseLocal_;

    // Объект сообщения от лидара
    //sensor_msgs::PointCloud2 lidarData_;

    //обработка напрямую
    sensor_msgs::LaserScan scanData_;


    //Объявление объекта‑подписчика на положение БЛА
    ros::Subscriber localPositionSub_; 
 
    //Объявление объекта‑подписчика на облако точек от лидара
    ros::Subscriber pointCloudSub_;

    // издатель для обработанного облака точек
    ros::Publisher pointcloud_pub_; 

    //метод инициализации ноды
    void rosNodeInit();    

    //метод обработки реального положения БЛА
    void realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal);

    //Метод обработки облака точек от лидара
    //void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void scanDataCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // Параметры VFH2D
    int num_sectors_ = 36; //Число секторов
    double sector_width_rad_ = 2.0 * M_PI / num_sectors_; //Угол сектора
    double safety_radius_ = 1.0;        // минимальный безопасный радиус (м)
    double lookahead_distance_ = 2;    // дистанция до локальной цели (м)

    // Гистограмма: стоимость для каждого сектора
    std::vector<double> sector_cost_;

    // Топик для выдачи локальной цели
    ros::Publisher local_goal_pub_;

    // Топик для выдачи глобальной цели
    ros::Publisher global_goal_pub_;

    //Подготовка данных по позиции БЛА
    double uavX = 0.0, uavY = 0.0, uavZ = 0.0;

    //информация о последней цели
    geometry_msgs::PoseStamped last_local_goal_;

    //информация о текущей цели
    geometry_msgs::PoseStamped current_local_goal_;

    // Глобальная точка назначения
    geometry_msgs::PointStamped global_goal_; 

    //Достижение глобальной точки
    bool has_global_goal_ = false;

    //лучший сектор
    int thebest;

    int last_selected_sector_ = -1;

    
    ros::Time last_goal_time_;

    ros::Publisher marker_pub_;                   // Публикатор маркеров
    visualization_msgs::MarkerArray marker_array; // Инициализация массива маркеров
    visualization_msgs::Marker marker;            // Создание маркера для визуализации вокселей
   
    //Максимальный коэффициент для отображения
    double max_cost_for_viz = 2.0; 

    visualization_msgs::Marker 
        arrow_marker_,
        global_goal_marker_,
        local_goal_marker_,
        new_local_target_marker,
        old_local_target_marker,
        global_target_marker,
        trajectory_marker,
        trajectory_line;

    
    geometry_msgs::PointStamped point;

    //траектория
    std::vector<geometry_msgs::PointStamped> trajectory;

    //
    double  distance;

    // Минимум расстояния по каждому сектору (чтобы жёстко отсекать опасные)
    std::vector<double> min_range_in_sector_;

};

#endif