#include "flight_controller.hpp"

#include <vector>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

namespace uav_controller
{

	UavController::UavController(ros::NodeHandle &n, const std::string &uavName)
		: n_(n), uavName_(uavName)
	{
		rosNodeInit();
	}

	void UavController::rosNodeInit()
	{
		// Подписываемся на состояние и положение БЛА
		stateSub_ = n_.subscribe<mavros_msgs::State>("/mavros/state", 10, &UavController::uavStateCallback, this);

		// Инициализация подписки на топик реального положения ЛА
		localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &UavController::realPositionCallback, this);

		// Инициализируем publisher для целевого состояния ЛА
		setPointPub_ = n_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	}

	void UavController::uavStateCallback(const mavros_msgs::State::ConstPtr &msg)
	{
		currentState_ = *msg;
	}

	// метод для обработки реального положения БЛА
	void UavController::realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal)
	{
		currentPoseLocal_ = *currentPoseLocal;
	}

	// метод для установки ограничений значений в пределах заданного диапазона.
	double clip(double value, double min_val, double max_val)
	{
		if (value < min_val)
		{
			return min_val;
		}
		else if (value > max_val)
		{
			return max_val;
		}
		else
		{
			return value;
		}
	}

	// Метод, выполняющий арминг аппарата
	void UavController::arm(bool cmd)
	{
		if (!currentState_.armed)
		{
			mavros_msgs::CommandBool arm_cmd; // переменная mavros отвечающая за команду, принимающая значение истина/ложь
			arm_cmd.request.value = cmd;	  // передача значения команды в пространство сообщений  MavRos
											  // определим сервис, с нодой n_ ,отвечающий за арминг дрона
			ros::ServiceClient arming_client = n_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
			arming_client.call(arm_cmd); // непосредстванно арминг путем вызова сервиса arming_client
			// проверим результат работы команды
			if (arm_cmd.response.success) // отклик на вызов сервиса True (арминг получился)
			{
				std::cout << "Successfull Arming!" << std::endl; // сообщение в консоль об успехе
			}
			else
			{
				std::cerr << "Fail Arming!" << std::endl; // вывод сообщения об ошибке
			}
		}
	}

	// Метод выполняет взлет аппарата. выполняется взлет до целевой высоты с управлением по каналу высоты
	bool UavController::do_takeoff(double target_altitude)
	{
		double err_x, err_y, err_z, target_yaw, err_yaw, V_x, V_y, V_z, yaw_rate;

		if (!currentState_.armed)
		{
			arm(true);
		}

		auto start_time = ros::Time::now();
		bool altitude_reached = false;
		ros::Rate rate(20); // Частота ≥5 Гц

		bool result = false;
		target_alt = target_altitude;

		// Предварительная отправка setpoints в течение 2 секунд
		for (int i = 0; i < 40 && ros::ok(); ++i) // 20 Гц × 2 с = 40 итераций
		{
			setPointTypeInit();
			setPoint_.velocity.x = 0.0; // Начальное положение
			setPoint_.velocity.y = 0.0; // Начальное положение
			setPoint_.velocity.z = 0.0; // Начальное положение
			setPointPub_.publish(setPoint_);
			ros::spinOnce();
			rate.sleep();
		}

		// Переключение в OFFBOARD
		if (change_mode("OFFBOARD"))
		{
			ROS_INFO("Entered OFFBOARD mode");
		}
		else
		{
			ROS_ERROR("Failed to enter OFFBOARD mode");
			return false;
		}

		while (ros::ok() && !altitude_reached)
		{
			// расчет ошибки по положению
			err_z = target_alt - currentPoseLocal_.pose.position.z;
			err_x = 0.0 - currentPoseLocal_.pose.position.x;
			err_y = 0.0 - currentPoseLocal_.pose.position.y;
			// Расчет целевой скорости в глобальной системе координат
			V_x = 0.75 * err_x; // 0.75 - коэффициент П-регулятора
			V_y = 0.75 * err_y; // 0.75 - коэффициент П-регулятора
			V_z = 0.22 * err_z; // 0.22 - коэффициент П-регулятора

			// Установим лимиты целевых скоростей
			double V_x1 = clip(V_x, -2, 2);
			double V_y1 = clip(V_y, -2, 2);
			double V_z1 = clip(V_z, -1, 1);

			// Обновляем setpoint с целевой скоростью
			setPointTypeInit();
			// setPoint_.position.z = target_altitude;
			setPoint_.velocity.z = V_z1;
			setPoint_.velocity.x = V_x1;
			setPoint_.velocity.y = V_y1;
			// setPoint_.yaw_rate = yaw_rate;
			setPointPub_.publish(setPoint_);
			ROS_INFO("State: armed=%d, mode='%s', connected=%d",
					 currentState_.armed,
					 currentState_.mode.c_str(),
					 currentState_.connected);
			ROS_INFO("Current altitude: %.2f m", currentPoseLocal_.pose.position.z);

			// run();
			change_mode("OFFBOARD");
			if (currentState_.mode == "OFFBOARD" && currentState_.armed)
			{
				ROS_INFO("Ready for OFFBOARD control!");
			}
			else
			{
				ROS_ERROR("Failed to enter OFFBOARD mode");
				change_mode("OFFBOARD");
				// arm(true);
			}
			ros::spinOnce();

			//  Проверим достижение заданной высоты с точностью в 10 см
			if (std::abs(currentPoseLocal_.pose.position.z - target_altitude) < 0.1)
			{
				altitude_reached = true;
				std::cout << "takeoff altitude" << currentPoseLocal_.pose.position.z << std::endl;
				ROS_INFO("Target altitude reached.");
				result = true;
			}

			// Таймаут
			if ((ros::Time::now() - start_time).toSec() > 60.0)
			{
				ROS_WARN("Timeout (60 s) reached without achieving target altitude.");
				return false;
			}
		}
		return result;
	}

	//Метод для задания формата сообщения для параметров управления БЛА
	void UavController::setPointTypeInit()
	{
		// задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
		// приведенная ниже конфигурация соответствует управлению линейной скоростью ЛА
		// и угловой скоростью аппарата в канале рыскания(yaw)
		setPoint_.type_mask =
			mavros_msgs::PositionTarget::IGNORE_PX |
			mavros_msgs::PositionTarget::IGNORE_PY |
			mavros_msgs::PositionTarget::IGNORE_PZ |
			mavros_msgs::PositionTarget::IGNORE_AFX |
			mavros_msgs::PositionTarget::IGNORE_AFY |
			mavros_msgs::PositionTarget::IGNORE_AFZ |
			mavros_msgs::PositionTarget::IGNORE_YAW;

		// Конфигурация системы координат в соответствии с которой задаются параметры управления ЛА
		// при setpointCoordinateFrame = 1 управление происходит в неподвижной СК
		uint16_t setpointCoordinateFrame = 1;
		setPoint_.coordinate_frame = setpointCoordinateFrame;

		// setPoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	}

	// Смена режима полета
	bool UavController::change_mode(const std::string &mode)
	{
		mavros_msgs::SetMode sm;
		sm.request.custom_mode = mode;

		ros::ServiceClient client = n_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

		if (client.call(sm))
		{
			if (sm.response.mode_sent)
			{
				ROS_INFO("Mode changed to %s", mode.c_str());
				return true;
			}
			else
			{
				ROS_ERROR("Failed to change mode to %s", mode.c_str());
				return false;
			}
		}
		else
		{
			ROS_ERROR("Service call failed for mode change");
			return false;
		}
	}

	// Активация автоматического режима полета
	void UavController::offboard_enable(bool enable)
	{
		if (enable)
		{
			// Переключение режима на OFFBOARD
			change_mode("OFFBOARD");
		}
	}

	bool UavController::isConnected() const
	{
		return currentState_.connected;
	}

	bool UavController::isArmed() const
	{
		return currentState_.armed;
	}



	// Метод выполняет взлет аппарата. выполняется взлет до целевой высоты с управлением по каналу высоты
	bool UavController::go_to_target(double target_x, double target_y, double target_z)
	{
		double err_x, err_y, err_z, target_yaw, err_yaw, V_x, V_y, V_z, yaw_rate;

		if (!currentState_.armed)
		{
			arm(true);
		}

		auto start_time = ros::Time::now();
		bool altitude_reached = false, x_reached = false, y_reached = false;
		ros::Rate rate(20); // Частота ≥5 Гц

		bool result = false;

		// Предварительная отправка setpoints в течение 2 секунд
		for (int i = 0; i < 40 && ros::ok(); ++i) // 20 Гц × 2 с = 40 итераций
		{
			setPointTypeInit();
			setPoint_.velocity.x = 0.0; // Начальное положение
			setPoint_.velocity.y = 0.0; // Начальное положение
			setPoint_.velocity.z = 0.0; // Начальное положение
			setPointPub_.publish(setPoint_);
			ros::spinOnce();
			rate.sleep();
		}

		// Переключение в OFFBOARD
		if (change_mode("OFFBOARD"))
		{
			ROS_INFO("Entered OFFBOARD mode");
		}
		else
		{
			ROS_ERROR("Failed to enter OFFBOARD mode");
			return false;
		}

		while (ros::ok() && !(altitude_reached && x_reached && y_reached))
		{
			// расчет ошибки по положению
			err_z = target_z - currentPoseLocal_.pose.position.z;
			err_x = target_x - currentPoseLocal_.pose.position.x;
			err_y = target_y - currentPoseLocal_.pose.position.y;
			// Расчет целевой скорости в глобальной системе координат
			V_x = 0.75 * err_x; // 0.75 - коэффициент П-регулятора
			V_y = 0.75 * err_y; // 0.75 - коэффициент П-регулятора
			V_z = 0.22 * err_z; // 0.22 - коэффициент П-регулятора

			// Установим лимиты целевых скоростей
			double V_x1 = clip(V_x, -2, 2);
			double V_y1 = clip(V_y, -2, 2);
			double V_z1 = clip(V_z, -1, 1);

			// Обновляем setpoint с целевой скоростью
			setPointTypeInit();
			// setPoint_.position.z = target_altitude;
			setPoint_.velocity.z = V_z1;
			setPoint_.velocity.x = V_x1;
			setPoint_.velocity.y = V_y1;
			// setPoint_.yaw_rate = yaw_rate;
			setPointPub_.publish(setPoint_);
			ROS_INFO("State: armed=%d, mode='%s', connected=%d",
					 currentState_.armed,
					 currentState_.mode.c_str(),
					 currentState_.connected);
			ROS_INFO("Current altitude: %.2f m", currentPoseLocal_.pose.position.z);
			ROS_INFO("Current x: %.2f m", currentPoseLocal_.pose.position.x);
			ROS_INFO("Current y: %.2f m", currentPoseLocal_.pose.position.y);

			// run();
			change_mode("OFFBOARD");
			if (currentState_.mode == "OFFBOARD" && currentState_.armed)
			{
				ROS_INFO("Ready for OFFBOARD control!");
			}
			else
			{
				ROS_ERROR("Failed to enter OFFBOARD mode");
				change_mode("OFFBOARD");
				// arm(true);
			}
			ros::spinOnce();

			//  Проверим достижение заданной высоты с точностью в 10 см
			if (std::abs(currentPoseLocal_.pose.position.z - target_z) < 0.1)
			{
				altitude_reached = true;
				std::cout << "takeoff altitude" << currentPoseLocal_.pose.position.z << std::endl;
				ROS_INFO("Target altitude reached.");
				if (std::abs(currentPoseLocal_.pose.position.y - target_y) < 0.1)
				{
					y_reached = true;
					std::cout << "y position" << currentPoseLocal_.pose.position.y << std::endl;
					ROS_INFO("Target y reached.");
					if (std::abs(currentPoseLocal_.pose.position.x - target_x) < 0.1)
					{
						x_reached = true;
						std::cout << "x position" << currentPoseLocal_.pose.position.x << std::endl;
						ROS_INFO("Target x reached.");

						result = true;
					}
				}
			}

			// Таймаут
			if ((ros::Time::now() - start_time).toSec() > 60.0)
			{
				ROS_WARN("Timeout (60 s) reached without achieving target position");
				return false;
			}
		}
		return result;
	}

}