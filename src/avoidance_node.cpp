#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "test_avoidance.hpp"
#include "flight_controller.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    // Инициализация ноды и регистрация ее в master node
    // с именем avoidance_node_cpp
    ros::init(argc, argv, "avoidance_node_cpp");

    // cоздаем экземпляр класса ноды
    ros::NodeHandle n;

    // Задаем частоту 30 Гц
    ros::Rate rate(30);

    // создаем экземпляр класса системы управления БЛА
    // uav_controller::UavController controller(n);

    Avoidance copterControl(n);

    // Немного ждем пока сообщения начнут приходить на автопилот
    ROS_INFO("Waiting for system connection...");

    // Ждём подключения к автопилоту
    while (ros::ok() && !copterControl.isConnected())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
     ROS_INFO("Connection completed successfully.");

    // Зададим высоту взлета дрона
    double target_altitude = 2;

    /*copterControl.setGlobalGoal(5,0,2);

       while (ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;*/

   



    // Выполняем взлёт до заданной высоты
    ROS_INFO("Starting takeoff to %.2f m...", target_altitude);
    if (!copterControl.takeoff(target_altitude))
    {
        ROS_ERROR("Takeoff failed. Exiting.");
        return 1;
    }
    ROS_INFO("Takeoff completed successfully.");


      
    /* while (ros::ok())
    {
        ros::spinOnce();
        copterControl.go_to_target(0,0,target_altitude);
        copterControl.publishVfhHistogram();
        rate.sleep();
    }*/

       
    // ---------------------------------------------------------
    // ТЕСТ ВИЗУАЛИЗАЦИИ: один раз публикуем фейковый скан
    // ---------------------------------------------------------
    //ROS_INFO("Running one-time fake scan test for VFH visualization...");
    //copterControl.testVis();
    //copterControl.testFakeScan();


    //copterControl.processLidarAndPlan();
    //copterControl.publishVfhHistogram();

    //copterControl.go_to_global(5,2,target_altitude);
    copterControl.go_to_global(-5, 18,target_altitude);
    copterControl.go_to_global(2, 22,target_altitude);
    copterControl.go_to_global(3, 6,target_altitude);

    // Даём время на повисеть
    /*ROS_INFO("Waiting for 5 seconds...");
    ros::Time t1 = ros::Time::now();

    double dt_sec  = 0;
    
    while (dt_sec < 5)
    {
        ros::spinOnce();
        copterControl.hover();
        rate.sleep();
        ros::Time t2 = ros::Time::now();
        ros::Duration dt = t2 - t1;           // интервал как ros::Duration
        dt_sec = dt.toSec();          // интервал в секундах (double)
    }*/


    copterControl.land();
  
    /*while (ros::ok())
    {
        ros::spinOnce();

        copterControl.go_to_target(1,0,target_altitude);
        copterControl.processLidarAndPlan();
        copterControl.publishVfhHistogram();
        rate.sleep();
    }*/

    // Арминг дрона
    //copterControl.arm(true);

    // Даём время на инициализацию автопилота
  /*  ROS_INFO("Waiting for 2 seconds before takeoff...");
    for (int i = 0; i < 60 && ros::ok(); ++i)
    { // 2 секунды при частоте 30 Гц
        ros::spinOnce();
        rate.sleep();
    }

    

    ROS_INFO("Setting global goal to (2.0, 2.0, 2.0)");
    copterControl.setGlobalGoal(2.0, 5.0, 2.0); // Высота 2.0 м (или оставь текущую)

    ROS_INFO("Starting obstacle avoidance loop...");

    geometry_msgs::PoseStamped local_goal;

    copterControl.getLastLocalGoal(local_goal);

    ROS_INFO("local goal coords %.3f, %.3f, %.3f", 
        local_goal.pose.position.x,
        local_goal.pose.position.y,
        local_goal.pose.position.z);
        
        ROS_INFO("local goal orientation %.3f, %.3f, %.3f", 
        local_goal.pose.orientation.w,
        local_goal.pose.orientation.x,
        local_goal.pose.orientation.y,
        local_goal.pose.orientation.z);*/


   

    //copterControl.go_to_global(2, 5, 2);

    /*hile (ros::ok())
    {
        ros::spinOnce();

        copterControl.lidarDataGetter();

        geometry_msgs::PoseStamped local_goal;

         ROS_INFO("local goal coords %.3f, %.3f, %.3f", 
        local_goal.pose.position.x,
        local_goal.pose.position.y,
        local_goal.pose.position.z);
        
        ROS_INFO("local goal orientation %.3f, %.3f, %.3f", 
        local_goal.pose.orientation.w,
        local_goal.pose.orientation.x,
        local_goal.pose.orientation.y,
        local_goal.pose.orientation.z);

        if (copterControl.getLastLocalGoal(local_goal))
        {
            copterControl.go_to_target(
                local_goal.pose.position.x,
                local_goal.pose.position.y,
                local_goal.pose.position.z);
        }
        else
        {
            ROS_WARN("No valid goal. Hovering.");
        }

        /*double dist_to_goal = sqrt(pow(uavX - global_goal_.x, 2) + pow(uavY - global_goal_.y, 2));
        if (dist_to_goal < 0.3)
        { // Если ближе 30 см
            ROS_INFO("Goal reached!");
            // Тут можно либо выйти из цикла, либо просто перестать отправлять цели (зависнуть)
        }*/
        /*rate.sleep();
    }*/

    // Выполняем посадку
    /*ROS_INFO("Starting landing...");
    if (!copterControl.land())
    {
        ROS_ERROR("Landing failed. Exiting.");
        return 1;
    }
    ROS_INFO("Landing completed successfully.");*/

    return 0;
}