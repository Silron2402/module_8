#ifndef MAP_VISUALIZER_HPP
#define MAP_VISUALIZER_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

// Объявим класс для визуализации вокселей
class VoxelVisualizer
{
    // закрытые члены класса
private:
    // информация об исполняемой ноде
    ros::NodeHandle &n_;  
    // интенсивность цветов, прозрачность,   размер вокселя                      
    float red_, green_, blue_, alpha_, voxel_size;                             // 
    ros::Publisher marker_pub_;                   // Публикатор маркеров
    visualization_msgs::MarkerArray marker_array; // Инициализация массива маркеров
    visualization_msgs::Marker marker;            // Создание маркера для визуализации вокселей

    void rosNodeInit();

    int next_id_ = 0;  // счётчик ID вместо статической переменной


    // открытые члены класса
public:
    // конструктор класса с параметрами
    VoxelVisualizer(ros::NodeHandle &n,
         float r = 0.0,
         float g = 1.0,
         float b = 0.0,
         float a = 0.7,
         float size = 1.0);

    void clear();
    void add_marker(float x, float y, float z);
    void send_update();
    void add_marker_with_color(double x, double y, double z,
                              float r, float g, float b, float a, float size);
};

#endif
