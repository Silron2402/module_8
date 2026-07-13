#include "map_viz.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

// конструктор класса

VoxelVisualizer::VoxelVisualizer(ros::NodeHandle &n, float r, float g, float b, float a, float size)
    : n_(n), red_(r), green_(g), blue_(b), alpha_(a), voxel_size(size) {
    rosNodeInit();
    ROS_INFO("VoxelVisualizer initialized with size %.2f", size);
}

void VoxelVisualizer::rosNodeInit() {
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("voxel_marker_array", 10);
}


// метод для очистки массива точек
void VoxelVisualizer::clear()
{
    marker_array.markers.clear();
    next_id_ = 0;
    //ROS_INFO("Marker array cleared");

}
  

void VoxelVisualizer::add_marker(float x, float y, float z)
{
  
    add_marker_with_color(x, y, z, red_, green_, blue_, alpha_, voxel_size);
  
}


void VoxelVisualizer::send_update() {
    if (!marker_array.markers.empty()) {
        marker_pub_.publish(marker_array);
        ROS_INFO("Sent %zu markers to RViz", marker_array.markers.size());
    } else {
        ROS_WARN("No markers to send - array is empty");
    }
}


void VoxelVisualizer::add_marker_with_color(double x, double y, double z,
                                          float r, float g, float b, float a, float size)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "voxels";
    marker.id = next_id_++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Позиция
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    // Ориентация
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Размер
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    // Цвет
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker_array.markers.push_back(marker);
}