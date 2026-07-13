#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "flight_controller.hpp"
#include <mavros_msgs/State.h>
#include <vector>

int main(int argc, char **argv)
{
    // Инициализация ноды и регистрация ее в master node
    // с именем uav_controller_node
    ros::init(argc, argv, "uav_controller_node");

    // cоздаем экземпляр класса ноды
    ros::NodeHandle n;

    ros::Rate rate(30);

    // создаем экземпляр класса системы управления БЛА
    uav_controller::UavController controller(n);

      

    // Немного ждем пока сообщения начнут приходить на автопилот
    ROS_INFO("Waiting for system connection...");
    // Ждём подключения к автопилоту
    while (ros::ok() && !controller.isConnected()) {
        ros::spinOnce();
        rate.sleep();
    }


    controller.arm(true);

    // Даём время на инициализацию автопилота
    ROS_INFO("Waiting for 2 seconds before takeoff...");
    for (int i = 0; i < 60 && ros::ok(); ++i) {  // 2 секунды при частоте 30 Гц
        ros::spinOnce();
        rate.sleep();
    }

    // Выполняем арминг дрона с проверкой результата
    /*ROS_INFO("Attempting to arm the drone...");
    if (!controller.arm(true)) {
        ROS_ERROR("Failed to arm the drone. Exiting.");
        return 1;
    }
    ROS_INFO("Drone armed successfully.");*/

    //Зададим высоту взлета дрона
    double target_altitude = 2;

    //Выполним взлет дрона
    //controller.do_takeoff(target_altitude);

    // Выполняем взлёт до заданной высоты
    ROS_INFO("Starting takeoff to %.2f m...", target_altitude);
    if (!controller.go_to_target(0, 0, target_altitude)) {
        ROS_ERROR("Takeoff failed. Exiting.");
        return 1;
    }
    ROS_INFO("Takeoff completed successfully.");

   

       // Переходим в точку (0, 3, 2)
    ROS_INFO("Moving to target (0, 3, %.2f)...", target_altitude);
    if (!controller.go_to_target(0.0, 3.0, target_altitude)) {
        ROS_ERROR("Failed to reach target (0, 3, %.2f).", target_altitude);
        return 1;
    }
    ROS_INFO("Target (0, 3, %.2f) reached.", target_altitude);


    
   
    controller.go_to_target(0, 0, target_altitude);
   

    /*while (ros::ok())
    {
        // Делаем шаг системы
       // controller.run();
        
        //controller.mymap();
        //mapper.send_visualization();
        //visualizer.startVisualization();  // запускаем поток мониторинга
        
        ros::spinOnce();
        rate.sleep();
        //k++;
    }*/



    return 0;
};