#ifndef myflightController_HPP
#define myflightController_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

namespace uav_controller
{
    class UavController
    {
    public:
        UavController(ros::NodeHandle &n, const std::string &uavName = "mavros"); // конструктор класса
 
        // метод для арминга дрона. На вход принимает команду True/False
		void arm(bool cmd);

        //Метод управления процессом взлета
		bool do_takeoff(double target_altitude);

		//Метод управления процессом полета до точки
		bool go_to_target(double target_x, double target_y, double target_z);
        
        bool isConnected() const;

		bool isArmed() const;

    private:
        // Закрытые члены класса
        ros::NodeHandle &n_;

        std::string uavName_;

        double target_alt;  //Целевая высота взлета дрона

		mavros_msgs::PositionTarget setPoint_;        // объект сообщения для задающего воздействия*/

		mavros_msgs::State currentState_;              // объект сообщения о состоянии аппарата

		geometry_msgs::PoseStamped currentPoseLocal_;  // объект сообщения о положении и ориентации

		geometry_msgs::PoseStamped desPose_;           // объект сообщения о требуемом положении БЛА

		ros::Subscriber localPositionSub_;

		//ros::Subscriber desPoseSub_;
		ros::Subscriber stateSub_;

		ros::Publisher setPointPub_;
		//ros::Publisher setPosePub_;  //паблишер для положения по данным с камеры

		ros::Publisher setVelPub_;   //паблишер для скорости

		void rosNodeInit();

		void setPointTypeInit();

		void uavStateCallback(const mavros_msgs::State::ConstPtr &msg);
	
		void realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal);
			
		void offboard_enable(bool enable);

		bool change_mode(const std::string &mode);

    };

};

#endif