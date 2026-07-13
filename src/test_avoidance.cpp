#include "test_avoidance.hpp"
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "flight_controller.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>

// конструктор класса
Avoidance::Avoidance(ros::NodeHandle &n, const std::string &uavName)
    : n_(n),
      uavName_(uavName),
      lidarControl(n),
      flightControl(n),
      visualizer(n)
{
    // Инициализация отладочного маркера (стрелка направления сектора 0)
    arrow_marker_.header.frame_id = "map";
    arrow_marker_.ns = "vfh_debug";
    arrow_marker_.id = 999; // уникальный ID, чтобы не дублировался
    arrow_marker_.type = visualization_msgs::Marker::ARROW;
    arrow_marker_.action = visualization_msgs::Marker::ADD;
    arrow_marker_.pose.orientation.w = 1.0; // без вращения пока
    arrow_marker_.scale.x = 0.01;           // длина стрелки
    arrow_marker_.scale.y = 0.02;           // толщина «острия»
    arrow_marker_.scale.z = 0.02;
    arrow_marker_.color.r = 1.0;
    arrow_marker_.color.g = 0.0;
    arrow_marker_.color.b = 0.0;
    arrow_marker_.color.a = 1.0;

    // Инициализация маркера направленного на глобальную цель
    global_goal_marker_.header.frame_id = "map";
    global_goal_marker_.ns = "global_direction";
    global_goal_marker_.id = 777; // уникальный ID, чтобы не дублировался
    global_goal_marker_.type = visualization_msgs::Marker::ARROW;
    global_goal_marker_.action = visualization_msgs::Marker::ADD;
    global_goal_marker_.pose.orientation.w = 1.0; // без вращения пока
    global_goal_marker_.scale.x = 0.02;           // длина стрелки
    global_goal_marker_.scale.y = 0.04;           // толщина «острия»
    global_goal_marker_.scale.z = 0.04;
    global_goal_marker_.color.r = 0.0;
    global_goal_marker_.color.g = 0.0;
    global_goal_marker_.color.b = 1.0;
    global_goal_marker_.color.a = 1.0;

    // Инициализация маркера направленного на локальную цель
    local_goal_marker_.header.frame_id = "map";
    local_goal_marker_.ns = "local_direction";
    local_goal_marker_.id = 888; // уникальный ID, чтобы не дублировался
    local_goal_marker_.type = visualization_msgs::Marker::ARROW;
    local_goal_marker_.action = visualization_msgs::Marker::ADD;
    local_goal_marker_.pose.orientation.w = 1.0; // без вращения пока
    local_goal_marker_.scale.x = 0.02;           // длина стрелки
    local_goal_marker_.scale.y = 0.04;           // толщина «острия»
    local_goal_marker_.scale.z = 0.04;
    local_goal_marker_.color.r = 0.0;
    local_goal_marker_.color.g = 1.0;
    local_goal_marker_.color.b = 0.0;
    local_goal_marker_.color.a = 1.0;

    // Инициализация маркера локальной цели
    new_local_target_marker.header.frame_id = "map";
    new_local_target_marker.ns = "new_target_point";
    new_local_target_marker.id = 555; // уникальный ID, чтобы не дублировался
    new_local_target_marker.type = visualization_msgs::Marker::SPHERE;
    new_local_target_marker.action = visualization_msgs::Marker::ADD;
    new_local_target_marker.pose.orientation.w = 1.0; // без вращения пока
    new_local_target_marker.scale.x = 0.12;           // длина стрелки
    new_local_target_marker.scale.y = 0.12;           // толщина «острия»
    new_local_target_marker.scale.z = 0.12;
    new_local_target_marker.color.r = 1.0;
    new_local_target_marker.color.g = 1.0;
    new_local_target_marker.color.b = 0.0;
    new_local_target_marker.color.a = 1.0;

    // Инициализация old маркера локальной цели
    old_local_target_marker.header.frame_id = "map";
    old_local_target_marker.ns = "old_target_point";
    old_local_target_marker.id = 444; // уникальный ID, чтобы не дублировался
    old_local_target_marker.type = visualization_msgs::Marker::SPHERE;
    old_local_target_marker.action = visualization_msgs::Marker::ADD;
    old_local_target_marker.pose.orientation.w = 1.0; // без вращения пока
    old_local_target_marker.scale.x = 0.1;            // длина стрелки
    old_local_target_marker.scale.y = 0.1;            // толщина «острия»
    old_local_target_marker.scale.z = 0.1;
    old_local_target_marker.color.r = 0;
    old_local_target_marker.color.g = 1.0;
    old_local_target_marker.color.b = 1.0;
    old_local_target_marker.color.a = 1.0;

    // Инициализация маркера глобальной цели
    global_target_marker.header.frame_id = "map";
    global_target_marker.ns = "old_target_point";
    global_target_marker.id = 333; // уникальный ID, чтобы не дублировался
    global_target_marker.type = visualization_msgs::Marker::SPHERE;
    global_target_marker.action = visualization_msgs::Marker::ADD;
    global_target_marker.pose.orientation.w = 1.0; // без вращения пока
    global_target_marker.scale.x = 0.2;            // длина стрелки
    global_target_marker.scale.y = 0.2;            // толщина «острия»
    global_target_marker.scale.z = 0.2;
    global_target_marker.color.r = 1.0;
    global_target_marker.color.g = 0;
    global_target_marker.color.b = 0;
    global_target_marker.color.a = 1.0;

    // Инициализация маркера траектории
    trajectory_marker.header.frame_id = "map";
    trajectory_marker.ns = "trajectory_points";
    trajectory_marker.id = 121; // уникальный ID, чтобы не дублировался
    trajectory_marker.type = visualization_msgs::Marker::SPHERE;
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.pose.orientation.w = 1.0; // без вращения пока
    trajectory_marker.scale.x = 0.15;           // длина стрелки
    trajectory_marker.scale.y = 0.15;           // толщина «острия»
    trajectory_marker.scale.z = 0.15;
    trajectory_marker.color.r = 1.0;
    trajectory_marker.color.g = 1.0;
    trajectory_marker.color.b = 1.0;
    trajectory_marker.color.a = 1.0;

    // Инициализация линии траектории
    trajectory_line.header.frame_id = "map";
    trajectory_line.ns = "trajectory_line";
    trajectory_line.id = 987; // уникальный ID, чтобы не дублировался
    trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_line.action = visualization_msgs::Marker::ADD;
    trajectory_marker.pose.orientation.w = 1.0; // без вращения пока
    trajectory_line.scale.x = 0.05;             // длина стрелки
    // trajectory_marker.scale.y = 0.15;           // толщина «острия»
    // trajectory_marker.scale.z = 0.15;
    trajectory_line.color.r = 1.0;
    trajectory_line.color.g = 1.0;
    trajectory_line.color.b = 0;
    trajectory_line.color.a = 1.0;

    rosNodeInit(); // метод инициализации ноды

    // начальная точка траектории
    point.point.x = 0;
    point.point.y = 0;
    point.point.z = 0;

    trajectory.push_back(point);

    // Инициализируем вектор: num_sectors_ элементов, каждый = 1e9 (очень большое число)
    min_range_in_sector_.assign(num_sectors_, 1e9);
}

bool Avoidance::processLidarAndPlan()
{
    int k = 0;

    if (scanData_.ranges.empty()) // && k > 10)
    {
        ROS_WARN("Processing data: Empty point cloud received");
        return false;
    }

    // СБРОС минимума по секторам перед новым расчётом
    std::fill(min_range_in_sector_.begin(), min_range_in_sector_.end(), 1e9);

    sector_cost_.assign(num_sectors_, 0.0);

    // Подготовка гистограммы
    sector_cost_.assign(num_sectors_, 0.0);

    // Шаг в радианах
    double delta = sector_width_rad_;
    

    // ROS_INFO("angle_min %f", scanData_.angle_min);
    //  ROS_INFO("angle_increment %f", scanData_.angle_increment);
    //  ROS_INFO("points %f", scanData_.ranges.size());

    // перебор значений
    for (size_t i = 0; i < scanData_.ranges.size(); ++i)
    {
        // считываем значения
        double range = scanData_.ranges[i];

        // Пропускаем невалидные значения (NaN, Inf, за пределами min/max)
        if (std::isnan(range) || std::isinf(range))
        {
            double angle = scanData_.angle_min + i * scanData_.angle_increment;
            continue;
        }
        if (range < scanData_.range_min || range > scanData_.range_max)
        {
            double angle = scanData_.angle_min + i * scanData_.angle_increment;
            continue;
        }

        // Угол текущей точки
        double angle = scanData_.angle_min + i * scanData_.angle_increment;

        while (angle < -M_PI)
            angle += 2 * M_PI;
        while (angle > M_PI)
            angle -= 2 * M_PI;

        // Индекс сектора: [0, num_sectors_)
        int number = static_cast<int>((angle + M_PI) / sector_width_rad_);
        number = (number + num_sectors_ / 2) % num_sectors_;
        if (number < 0)
            number = 0;
        if (number >= num_sectors_)
            number = num_sectors_ - 1;

        // ROS_INFO("number %d", number);

        const double max_cost_per_point = max_cost_for_viz;

        double cost = 1.0 / (range * range);
        if (cost > max_cost_per_point)
            cost = max_cost_per_point;

        // Проверка: расстояние меньше безопасного радиуса
        if (range < safety_radius_)
        {
            sector_cost_[number] += max_cost_per_point * 5.0; // усиленный штраф
        }
        else
        {
            sector_cost_[number] += cost;
        }

        // ВОТ ЗДЕСЬ обновляем минимум по сектору:
        if (range < min_range_in_sector_[number]) 
        {
            min_range_in_sector_[number] = range;
        }
    }

    /*// Сглаживание гистограммы (чтобы не было резких скачков)
    std::vector<double> smoothed_cost = sector_cost_;
    for (int i = 0; i < N; ++i)
    {
        smoothed_cost[i] = (sector_cost_[i] + sector_cost_[(i - 1 + num_sectors_) % num_sectors_] + sector_cost_[(i + 1) % num_sectors_]) / 3.0;
    }
    sector_cost_ = smoothed_cost;*/

    int best_sector = selectBestSector();
    thebest = best_sector;
    if (best_sector == -1)
    {
        ROS_WARN("No safe sector found — activating fallback (hover)");
        // Здесь можно добавить логику зависания или разворота
        // publishVfhHistogram();
        return false;
    }

    geometry_msgs::PoseStamped goal = makeLocalGoal(best_sector);

    last_local_goal_ = goal;
    last_goal_time_ = ros::Time::now();

    local_goal_pub_.publish(goal);

    return true;

    // publishVfhHistogram();*/
}

int Avoidance::selectBestSector() const
{
    if (!has_global_goal_)
    {
        int best = -1;
        double min_cost = 1e9;

        const double cost_threshold = 10.0; // порог отсечения «опасных» секторов

        for (int i = 0; i < num_sectors_; ++i)
        {
            if (sector_cost_[i] < cost_threshold && sector_cost_[i] < min_cost)
            {
                min_cost = sector_cost_[i];
                best = i;
            }
        }
        return best;
    }

    // --- ЛОГИКА С ГЛОБАЛЬНОЙ ЦЕЛЬЮ ---
    // текущие координаты БЛА
    double uavX = currentPoseLocal_.pose.position.x;
    double uavY = currentPoseLocal_.pose.position.y;

    // Направление на глобальную цель
    double dx = global_goal_.point.x - uavX;
    double dy = global_goal_.point.y - uavY;

    // Угол на глобальную цель
    double goal_angle = std::atan2(dy, dx);

    // Нормализация угла цели в диапазон [0, 2PI)
    if (goal_angle < 0)
        goal_angle += 2 * M_PI;

    // Получаем угол рыскания (Yaw) из кватерниона ---
    const auto &q = currentPoseLocal_.pose.orientation;
    // Формула для получения Yaw из кватерниона (x, y, z, w)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    // ----------------------------------------------------------------------

    int best_sector = -1;
    double best_score = 1e9; // Чем меньше, тем лучше

    const double bias_weight = 2.0; // Насколько сильно мы хотим идти к цели (подбирается экспериментально)

    for (int i = 0; i < num_sectors_; ++i)
    {
        if (sector_cost_[i] > 100.0)
            continue; // Если сектор слишком опасен — пропускаем

        if (min_range_in_sector_[i] < safety_radius_) 
        {
        continue; // сектор считается непроходимым
        }

        // Угол центра этого сектора
        double local_sector_angle = (i + 0.5) * sector_width_rad_;
        // double sector_angle = (i + 0.5) * sector_width_rad_;

        // Глобальный угол = Угол дрона + Угол сектора относительно дрона
        double global_sector_angle = yaw_rad + local_sector_angle;

        // Нормализация глобального угла сектора
        while (global_sector_angle < -M_PI)
            global_sector_angle += 2 * M_PI;
        while (global_sector_angle > M_PI)
            global_sector_angle -= 2 * M_PI;

        // Разница между углом сектора и углом на цель
        double diff = std::abs(global_sector_angle - goal_angle);

        // Кратчайшее расстояние по кругу
        if (diff > M_PI)
            diff = 2 * M_PI - diff;

        // Штраф за отклонение от курса к цели (чем дальше от цели — тем хуже)
        // Можно сделать квадратичным: diff * diff
        double goal_penalty = bias_weight * (diff * diff);

        // Итоговый счет = опасность сектора + отклонение от курса
        double total_score = sector_cost_[i] + goal_penalty;

        if (total_score < best_score)
        {
            best_score = total_score;
            best_sector = i;
        }
    }

    return best_sector;
}

/*geometry_msgs::PoseStamped Avoidance::makeLocalGoal(int sector_idx)
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    uavX = currentPoseLocal_.pose.position.x;
    uavY = currentPoseLocal_.pose.position.y;

    // угол сектора в локальной системе дрона (лидар смотрит вдоль +X дрона)
    double sector_local_angle = (sector_idx + 0.5) * sector_width_rad_;

    // 2. Извлекаем текущий угол дрона из кватерниона
    const auto &q = currentPoseLocal_.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);

    // 3. Итоговый глобальный угол
    double global_angle = yaw_rad + sector_local_angle;

    // Нормализация
    while (global_angle < -M_PI)
        global_angle += 2 * M_PI;
    while (global_angle > M_PI)
        global_angle -= 2 * M_PI;

    double dst = set_distance(sector_idx);

    goal.pose.position.x = uavX + dst * std::cos(global_angle);
    goal.pose.position.y = uavY + dst * std::sin(global_angle);

    goal.pose.position.z = currentPoseLocal_.pose.position.z; // держим текущую высоту

    // Ориентация: можно смотреть вперёд по направлению цели
    goal.pose.orientation = currentPoseLocal_.pose.orientation;

    return goal;
}*/

geometry_msgs::PoseStamped Avoidance::makeLocalGoal(int sector_idx)
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    double uavX = currentPoseLocal_.pose.position.x;
    double uavY = currentPoseLocal_.pose.position.y;

    double sector_local_angle = (sector_idx + 0.5) * sector_width_rad_;

    const auto &q = currentPoseLocal_.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);

    double global_angle = yaw_rad + sector_local_angle;
    while (global_angle < -M_PI) global_angle += 2 * M_PI;
    while (global_angle >  M_PI) global_angle -= 2 * M_PI;

    // Сначала считаем базовую дистанцию
    double dst = set_distance(sector_idx);

    // Пробуем поставить цель
    goal.pose.position.x = uavX + dst * std::cos(global_angle);
    goal.pose.position.y = uavY + dst * std::sin(global_angle);
    goal.pose.position.z = currentPoseLocal_.pose.position.z;
    goal.pose.orientation = currentPoseLocal_.pose.orientation;

    // Если цель небезопасна — уменьшаем дистанцию и пересчитываем
    double safety = safety_radius_ + 0.2; // небольшой буфер
    int attempts = 0;
    while (!isGoalSafe(goal, safety) && dst > 0.3 && attempts < 5) {
        dst *= 0.7; // уменьшаем дистанцию
        goal.pose.position.x = uavX + dst * std::cos(global_angle);
        goal.pose.position.y = uavY + dst * std::sin(global_angle);
        attempts++;
    }

    return goal;
}


void Avoidance::setGlobalGoal(double x, double y, double z)
{
    global_goal_.point.x = x;
    global_goal_.point.y = y;
    global_goal_.point.z = z;
    has_global_goal_ = true;
    ROS_INFO("Global goal set to (%.2f, %.2f, %.2f)", x, y, z);
}

// метод позволяет взлетать с той позиции, где сейчас находится коптер, чтобы его не тащило по земле
bool Avoidance::takeoff(double target)
{
    ROS_INFO("Starting takeoff to %.2f m...", target);

    setGlobalGoal(currentPoseLocal_.pose.position.x, currentPoseLocal_.pose.position.y, target);

    if (!go_to_target(
            currentPoseLocal_.pose.position.x,
            currentPoseLocal_.pose.position.y,
            target))
    {
        return false;
    };
    return true;
}

// Метод возврата в исходную точку высота та же.

bool Avoidance::go_to_base()
{
    setGlobalGoal(0, 0, currentPoseLocal_.pose.position.z);

    if (!go_to_global(0, 0, currentPoseLocal_.pose.position.z))
    {
        return false;
    };
    return true;
}

// посадка
bool Avoidance::land()
{
    // возврат в исходную точку
    go_to_base();

    setGlobalGoal(0, 0, 0);
    // посадка
    if (!go_to_target(0, 0, 0))
    {
        return false;
    };
    return true;
}

// арминг
bool Avoidance::arm(bool cmd)
{
    flightControl.arm(cmd);
    if (!flightControl.isArmed())
    {
        return false;
    };
    return true;
}

// полет к цели
bool Avoidance::go_to_target(double target_x, double target_y, double target_z)
{
    // устанавливаем глобальную цель как точку
    // setGlobalGoal(target_x, target_y, target_z);

    if (!flightControl.go_to_target(target_x, target_y, target_z))
    {
        return false;
    };
    // начальная точка траектории
    point.point.x = target_x;
    point.point.y = target_y;
    point.point.z = target_z;
    trajectory.push_back(point);
    return true;
}

// проверка соединения
bool Avoidance::isConnected() const
{
    if (!flightControl.isConnected())
    {
        return false;
    };
    return true;
}

// метод для настройки ROS-публикаций и подписок.
void Avoidance::rosNodeInit()
{
    // Инициализация подписки на топик реального положения ЛА
    localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 1, &Avoidance::realPositionCallback, this);

    // Подписка на облако точек с лидара
    pointCloudSub_ = n_.subscribe<sensor_msgs::LaserScan>(
        "/scan", 1, &Avoidance::scanDataCallback, this);

    // публикатор локальной цели
    local_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("local_goal", 10);

    // публикатор  глобальной цели
    global_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("global_goal", 10);

    // публикатор маркеров для визуализации диаграммы
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("voxel_marker_array", 10);

    ROS_INFO("Node initialized, waiting for point cloud data...");
}

// метод для обработки реального положения БЛА
void Avoidance::realPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &currentPoseLocal)
{
    currentPoseLocal_ = *currentPoseLocal;
}

/*void Avoidance::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    lidarData_ = *msg;
    processLidarAndPlan(); // Вызываем обработку данных
}*/

void Avoidance::scanDataCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scanData_ = *msg;
    processLidarAndPlan(); // Вызываем обработку данных
    publishVfhHistogram(); // публикуемся
}

bool Avoidance::getLastLocalGoal(geometry_msgs::PoseStamped &out_goal) const
{
    // Проверяем, что цель не слишком старая (например, старше 1 секунды)
    if ((ros::Time::now() - last_goal_time_).toSec() > 1.0)
    {
        return false; // Данные устарели
    }
    out_goal = last_local_goal_;
    return true;
}

void Avoidance::lidarDataGetter()
{
    lidarControl.dataGetter();
}

bool Avoidance::go_to_global(double x, double y, double z)
{
    bool result = false;
    // Задаем частоту 30 Гц
    ros::Rate rate(30);

    // устанавливаем глобальную цель как точку
    setGlobalGoal(x, y, z);

    while (ros::ok() && !result)
    {
        ros::spinOnce();

        // Получение данных от лидара
        lidarDataGetter();

        // Получение локальной цели
        geometry_msgs::PoseStamped local_goal;

        // go_to_target(uavX, uavY, uavZ);

        int k = 0;

        if (getLastLocalGoal(local_goal) || k < 100)
        {
            // локальная цель
            // new_local_target_marker.header.stamp = ros::Time::now();
            // положение маркера
            // new_local_target_marker.pose.position.x = local_goal.pose.position.x;
            // new_local_target_marker.pose.position.y = local_goal.pose.position.y;
            // new_local_target_marker.pose.position.z = local_goal.pose.position.z;

            // marker_array.markers.push_back(new_local_target_marker);
            current_local_goal_ = local_goal;

            go_to_target(
                local_goal.pose.position.x,
                local_goal.pose.position.y,
                local_goal.pose.position.z);
        }
        else
        {
            ROS_WARN("No valid goal. Hovering.");
            k += 1;
        }

        // оценка расстояния до цели
        double dist_to_goal = sqrt(pow(currentPoseLocal_.pose.position.x - x, 2) + pow(currentPoseLocal_.pose.position.y - y, 2));
        // Если ближе 30 см
        if (dist_to_goal < 0.3)
        {
            ROS_INFO("Goal reached!");
            result = true;
        }

        visualizer.send_update();

        rate.sleep();
    }
    return result;
}

void Avoidance::target_pub(double x, double y, double z)
{
    visualizer.add_marker_with_color(x, y, z, 1, 0, 0, 1, 0.5);
}

void Avoidance::map_update()
{
    visualizer.send_update(); // запускаем поток мониторинга
}

void Avoidance::publishVfhHistogram()
{
    // обязательно очищаем
    marker_array.markers.clear();
    // Координаты центра гистограммы
    double x_center = currentPoseLocal_.pose.position.x;
    double y_center = currentPoseLocal_.pose.position.y;
    double z_center = currentPoseLocal_.pose.position.z;

    if (sector_cost_.empty())
    {
        ROS_WARN("sector_cost_ is empty — no VFH data yet");
    }
    else
    {
        // работаем с данными

        for (int i = 0; i < num_sectors_; ++i)
        {

            // Расчет угла сектора диаграммы
            double angle = (i + 0.5) * sector_width_rad_;

            // координаты радиуса визуализации
            double cx = 0.8 * cos(angle);
            double cy = 0.8 * sin(angle);

            double cost = sector_cost_[i];

            // ROS_INFO("sector %f", sector_width_rad_);

            // расчет коэффициента для отображения в цвете
            float ratio = std::min(1.0, cost / max_cost_for_viz);

            // Создание маркера для визуализации
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "map";
            marker.ns = "vfh_sectors";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            // Позиция
            marker.pose.position.x = currentPoseLocal_.pose.position.x + cx;
            marker.pose.position.y = currentPoseLocal_.pose.position.y + cy;
            marker.pose.position.z = 0.1;
            // Ориентация
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Размер
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            if (cost < 0.01)
                cost = 0.01;
            if (i == thebest)
                cost = cost;
            marker.scale.z = static_cast<float>(cost * 0.02); // масштабируем высоту;

            // Цвет
            if (i != thebest)
            {
                marker.color.r = ratio;
                marker.color.g = 1.0f - ratio;
                marker.color.b = 0.0f;
            }
            else
            {
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1.0f;
            }

            marker.color.a = 1.0f;

            // добавление
            marker_array.markers.push_back(marker);
        }
    }

    // публикация траектории
    if (trajectory.empty())
    {
        ROS_WARN("sector_cost_ is empty — no VFH data yet");
    }
    else
    {
        for (size_t i = 0; i < size(trajectory); i++)
        {
            trajectory_marker.header.stamp = ros::Time::now();
            trajectory_marker.pose.position.x = trajectory[i].point.x;
            trajectory_marker.pose.position.y = trajectory[i].point.y;
            trajectory_marker.pose.position.z = trajectory[i].point.z;
            trajectory_marker.id = static_cast<int>(i + 122);
            marker_array.markers.push_back(trajectory_marker);
        }
    }

    //Заполняем points
    trajectory_line.points.clear();
    for (const auto &pt : trajectory)
    {
        geometry_msgs::Point p;
        p.x = pt.point.x;
        p.y = pt.point.y;
        p.z = pt.point.z;
        trajectory_line.points.push_back(p);
    }

    marker_array.markers.push_back(trajectory_line);

    if (has_global_goal_)
    {
        // Старая локальная цель (если она была)
        if (!last_local_goal_.header.frame_id.empty()) // простая проверка валидности
        {
            old_local_target_marker.header.stamp = ros::Time::now();
            old_local_target_marker.pose.position.x = last_local_goal_.pose.position.x;
            old_local_target_marker.pose.position.y = last_local_goal_.pose.position.y;
            old_local_target_marker.pose.position.z = last_local_goal_.pose.position.z;
            marker_array.markers.push_back(old_local_target_marker);
        }

        // Новая локальная цель
        new_local_target_marker.header.stamp = ros::Time::now();
        new_local_target_marker.pose.position.x = current_local_goal_.pose.position.x;
        new_local_target_marker.pose.position.y = current_local_goal_.pose.position.y;
        new_local_target_marker.pose.position.z = current_local_goal_.pose.position.z;
        marker_array.markers.push_back(new_local_target_marker);

        // глобальная цель
        global_target_marker.header.stamp = ros::Time::now();
        global_target_marker.pose.position.x = global_goal_.point.x;
        global_target_marker.pose.position.y = global_goal_.point.y;
        global_target_marker.pose.position.z = global_goal_.point.z;
        marker_array.markers.push_back(global_target_marker);
    }

    // --- Отладочная стрелка: направление сектора 0 ---
    arrow_marker_.header.stamp = ros::Time::now();
    // Начало стрелки — в центре дрона
    arrow_marker_.pose.position.x = x_center; // currentPoseLocal_.pose.position.x;
    arrow_marker_.pose.position.y = y_center; // currentPoseLocal_.pose.position.y;
    arrow_marker_.pose.position.z = z_center; // currentPoseLocal_.pose.position.z; // 0.1;

    // Конец стрелки — по направлению сектора 0
    double zero_sector_angle = (0 + 0.5) * sector_width_rad_; // угол центра сектора 0

    arrow_marker_.points.clear();
    geometry_msgs::Point p_start, p_end;
    p_start.x = 0;                                               // currentPoseLocal_.pose.position.x;
    p_start.y = 0;                                               // currentPoseLocal_.pose.position.y;
    p_start.z = 0;                                               // currentPoseLocal_.pose.position.z; // относительно pose
    p_end.x = lookahead_distance_ * std::cos(zero_sector_angle); // + currentPoseLocal_.pose.position.x;
    p_end.y = lookahead_distance_ * std::sin(zero_sector_angle); // + currentPoseLocal_.pose.position.y;
    p_end.z = 0;
    //currentPoseLocal_.pose.position.z;

    arrow_marker_.points.push_back(p_start);
    arrow_marker_.points.push_back(p_end);

    // Важно: для ARROW с points нужно убрать ориентацию (она игнорируется)
    arrow_marker_.pose.orientation.x = 0.0;
    arrow_marker_.pose.orientation.y = 0.0;
    arrow_marker_.pose.orientation.z = 0.0;
    arrow_marker_.pose.orientation.w = 1.0;

    marker_array.markers.push_back(arrow_marker_);

    //

    if (has_global_goal_)
    {
        // Отладочная стрелка: направление на цель
        global_goal_marker_.header.stamp = ros::Time::now();
        // Начало стрелки — в центре дрона
        global_goal_marker_.pose.position.x = currentPoseLocal_.pose.position.x;
        global_goal_marker_.pose.position.y = currentPoseLocal_.pose.position.y;
        global_goal_marker_.pose.position.z = currentPoseLocal_.pose.position.z; // 0.1;

        // Конец стрелки — по направлению цели
        // угол к цели
        double target_angle = atan2(global_goal_.point.y - y_center, global_goal_.point.x - x_center);
        global_goal_marker_.points.clear();
        // geometry_msgs::Point p_start, p_end;
        // p_start.x = uavX;
        // p_start.y = uavY;
        // p_start.z = uavZ; // относительно pose
        p_end.x = 1 * std::cos(target_angle);// + currentPoseLocal_.pose.position.x;
        p_end.y = 1 * std::sin(target_angle);// + currentPoseLocal_.pose.position.y;
        p_end.z = 0; //currentPoseLocal_.pose.position.z;

        global_goal_marker_.points.push_back(p_start);
        global_goal_marker_.points.push_back(p_end);

        // Важно: для ARROW с points нужно убрать ориентацию (она игнорируется)
        global_goal_marker_.pose.orientation.x = 0.0;
        global_goal_marker_.pose.orientation.y = 0.0;
        global_goal_marker_.pose.orientation.z = 0.0;
        global_goal_marker_.pose.orientation.w = 1.0;

        marker_array.markers.push_back(global_goal_marker_);
    }

    if (!sector_cost_.empty())
    {
        // cтрелка: направление на локальную цель
        local_goal_marker_.header.stamp = ros::Time::now();
        // Начало стрелки — в центре дрона
        local_goal_marker_.pose.position.x = currentPoseLocal_.pose.position.x;
        local_goal_marker_.pose.position.y = currentPoseLocal_.pose.position.y;
        local_goal_marker_.pose.position.z = currentPoseLocal_.pose.position.z; // 0.1;

        // Конец стрелки — по направлению цели

        // угол к локальной цели
        double local_angle = atan2(last_local_goal_.pose.position.y - currentPoseLocal_.pose.position.y,
                                   last_local_goal_.pose.position.x - currentPoseLocal_.pose.position.x);
        local_goal_marker_.points.clear();
        // geometry_msgs::Point p_start, p_end;
        // p_start.x = uavX;
        // p_start.y = uavY;
        // p_start.z = uavZ; // относительно pose
        p_end.x = 1 * std::cos(local_angle); // + currentPoseLocal_.pose.position.x;
        p_end.y = 1 * std::sin(local_angle); // + currentPoseLocal_.pose.position.y;
        p_end.z = 0;                         // currentPoseLocal_.pose.position.z;

        local_goal_marker_.points.push_back(p_start);
        local_goal_marker_.points.push_back(p_end);

        // Важно: для ARROW с points нужно убрать ориентацию (она игнорируется)
        local_goal_marker_.pose.orientation.x = 0.0;
        local_goal_marker_.pose.orientation.y = 0.0;
        local_goal_marker_.pose.orientation.z = 0.0;
        local_goal_marker_.pose.orientation.w = 1.0;

        marker_array.markers.push_back(local_goal_marker_);
    }

    // публикация
    if (!marker_array.markers.empty())
    {
        marker_pub_.publish(marker_array);
        // ROS_INFO("Sent %zu markers to RViz", marker_array.markers.size());
    }
    else
    {
        ROS_WARN("Visualiser: No markers to send - array is empty");
    }
}

void Avoidance::testFakeScan()
{
    sensor_msgs::LaserScan fakeScan;
    fakeScan.header.frame_id = "map";
    fakeScan.header.stamp = ros::Time::now();

    fakeScan.angle_min = -M_PI;
    fakeScan.angle_max = M_PI;
    fakeScan.angle_increment = M_PI / 180.0 * 2; // 360 секторов по 1 градусу
    fakeScan.range_min = 0.1;
    fakeScan.range_max = 10.0;

    int n = static_cast<int>((fakeScan.angle_max - fakeScan.angle_min) / fakeScan.angle_increment) + 1;
    fakeScan.ranges.resize(n);

    // Создаём препятствие слева (примерно -90 градусов)
    for (int i = 0; i < n; ++i)
    {
        double angle = fakeScan.angle_min + i * fakeScan.angle_increment;
        if (angle > -M_PI / 2 - 0.2 && angle < -M_PI / 2 + 0.2)
        {
            fakeScan.ranges[i] = 0.5; // близко — опасно
        }
        else
        {
            if (angle > M_PI / 2 - 0.2 && angle < M_PI / 2 + 0.2)
            {
                fakeScan.ranges[i] = 1.1; // близко — опасно
            }
            else
            {
                fakeScan.ranges[i] = 5.0; // далеко — безопасно
            }
        }
    }

    scanData_ = fakeScan;
    processLidarAndPlan();
    publishVfhHistogram();
    ROS_INFO("Fake scan published and VFH histogram rendered");
}

// Метод для тестирования потсроителя гистограммы.
// Заполняет случайными числами сектора диаграммы в диапазоне от 0 до 20
void Avoidance::testVis()
{
    // Подготовка гистограммы
    sector_cost_.assign(num_sectors_, 1.0);

    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist_real(0.0, 20.0);

    for (int i = 0; i < sector_cost_.size(); ++i)
    {
        sector_cost_.at(i) = dist_real(gen);
    }

    publishVfhHistogram();
    ROS_INFO("Fake VFH histogram rendered");
}

void Avoidance::hover()
{
    if (currentPoseLocal_.header.frame_id.empty())
    {
        ROS_WARN("No pose received yet. Cannot hover.");
        return;
    }

    double x = currentPoseLocal_.pose.position.x;
    double y = currentPoseLocal_.pose.position.y;
    double z = currentPoseLocal_.pose.position.z;

    // Один раз ставим цель в текущие координаты
    flightControl.go_to_target(x, y, z);
}

double Avoidance::set_distance(int sector_idx)
{
    //установим базовую дистанцию
    double base_distance = lookahead_distance_;

    //рассчитаем величину
    int i = sector_idx;
    double value = sector_cost_[i];

    if (value < 0.2)
    {
        base_distance = lookahead_distance_ * 1.5;
    }

    else
    {
        if (value > 0.8)
        {
            base_distance = lookahead_distance_ * 0.2;
        }
    }


    //проверим положение дрона
    double delta_x = currentPoseLocal_.pose.position.x - global_goal_.point.x;
    double delta_y = currentPoseLocal_.pose.position.y - global_goal_.point.y;
    double delta_z = currentPoseLocal_.pose.position.z - global_goal_.point.z;

    double dist = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

    if (dist < base_distance)
    {
        base_distance = dist;
    } 

    return base_distance;   

}

//проверка безопасности локальной цели
bool Avoidance::isGoalSafe(const geometry_msgs::PoseStamped & goal, double safety_margin) const
{
    if (scanData_.ranges.empty())
        return false;

    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    // Проверяем все точки лидара в их «сырых» углах и считаем их позицию
    // относительно дрона, затем переводим в map через текущую позицию дрона.
    // Это грубая проверка, но достаточная, чтобы не ставить цель в препятствие.
    for (size_t i = 0; i < scanData_.ranges.size(); ++i) {
        double r = scanData_.ranges[i];
        if (std::isnan(r) || std::isinf(r)) continue;
        if (r < scanData_.range_min || r > scanData_.range_max) continue;

        double angle = scanData_.angle_min + i * scanData_.angle_increment;
        // Позиция точки лидара относительно дрона
        double lx = r * std::cos(angle);
        double ly = r * std::sin(angle);

        // Переводим в map (дрон в (uavX, uavY))
        double px = currentPoseLocal_.pose.position.x + lx;
        double py = currentPoseLocal_.pose.position.y + ly;

        double dx = gx - px;
        double dy = gy - py;
        double dist_sq = dx*dx + dy*dy;

        if (dist_sq < (safety_margin * safety_margin)) {
            return false; // Цель слишком близко к точке лидара
        }
    }
    return true;
}
