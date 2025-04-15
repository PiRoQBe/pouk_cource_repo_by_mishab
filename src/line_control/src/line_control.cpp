// #include "line_control.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>    
using namespace std::chrono_literals;


class LineControl : public rclcpp::Node{
    public:
    LineControl() : Node("line_control"){

        // Объявление параметров
        line_y = this->declare_parameter("line_y", -6.0);
        cx = this->declare_parameter("cx", -6);
        cy = this->declare_parameter("cy", 0);
        R = this->declare_parameter("R", 6);
        task_vel = this->declare_parameter("task_vel", 1.0);
        prop_factor = this->declare_parameter("prop_factor", 5.0);
        int_factor = this->declare_parameter("int_factor", 0.001);
        diff_factor = this->declare_parameter("diff_factor", 14);
        min_obstacle_range = this->declare_parameter("min_obstacle_range", 1.0);
        double dt = this->declare_parameter("dt", 0.1);

        // Создаем издателя для команды движения
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_scan", 10, std::bind(&LineControl::laserCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "base_pose_ground_truth", 10, std::bind(&LineControl::poseCallback, this, std::placeholders::_1));
        
        
        err_pub_ = this->create_publisher<std_msgs::msg::Float64>("err", 10);

        // Таймер для периодической отправки команд
        // timer_ = this->create_wall_timer(
        //     500ms, std::bind(&LineControl::timerCallback, this));
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt), std::bind(&LineControl::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Simple driver node started");

    }

    private:
    //функция вычисления ошибки управления для движения вдоль прямой
    double cross_track_err_line(){
        return line_y - y;
    }
    double cross_track_err_line(double l_y){
        return l_y - y;
    }
    //функция вычисления ошибки управления для движения вдоль окружности
    double cross_track_err_circle(){
        double dx = cx - x;
        double dy = cy - y;
        double e = sqrt(dx*dx + dy*dy) - R;
        return  e;
    }

    double cross_track_err_circle(double c_x, double c_y, double r){
        double dx = c_x - x;
        double dy = c_y - y;
        double e = sqrt(dx*dx + dy*dy) - r;
        return  e;
    }

    double cross_track_err_ellipse(){
        double e;
        
        if (x > 6) {
            e = cross_track_err_circle(6, 0, 6);
        }
        else if (x < -6) {
            e = cross_track_err_circle(-6, 0, 6);
        }
        else if (y > 0) {
            e = -cross_track_err_line(6);
        }
        else if(y < 0){
            e = cross_track_err_line(-6); 
        }
        
        return  e;
    }
    /**
     * Функция, которая будет вызвана
     * при получении данных от лазерного дальномера
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        // проверим нет ли вблизи робота препятствия
        const double kMinObstacleDistance = 0.3;
        for (size_t i = 0; i<msg->ranges.size(); i++)
        {
            if ( msg->ranges[i] < kMinObstacleDistance )
            {
                obstacle = true;
                RCLCPP_WARN(this->get_logger(),"OBSTACLE!!!");
                break;
            }
        }
    }
    /**
     * Функция, которая будет вызвана при
     * получении сообщения с текущем положением робота
     */
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        RCLCPP_DEBUG(this->get_logger(),
            "Pose msg: x = %f y = %f theta = %f", 
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

        // обновляем переменные класса, отвечающие за положение робота
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        theta = 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    }
    /**
     * функция обработчик таймера
     */
    void timerCallback(/*const rclcpp::TimerBase::SharedPtr*/){
        RCLCPP_DEBUG(this->get_logger(), "on timer ");
        // сообщение с помощью которого задается
        // управление угловой и линейной скоростью
        geometry_msgs::msg::Twist cmd;
        // при создании структура сообщения заполнена нулевыми значениями

        // если вблизи нет препятствия то задаем команды
        if ( !obstacle )
        {
            //  вычислим текущую ошибку управления
            // double err = cross_track_err_line();
            double err = cross_track_err_ellipse();
            // double err = cross_track_err_circle();
            //  публикация текущей ошибки
            publish_error(err);
            //  интегрируем ошибку
            int_error += err;
            //  диффференцируем ошибку
            double diff_error = err - old_error;
            //   запоминаем значение ошибки для следующего момента времени
            old_error = err;
            cmd.linear.x = task_vel;
            //  ПИД регулятор угловой скорости w = k*err + k_и * инт_err + k_д * дифф_err
            cmd.angular.z = prop_factor * err + int_factor*int_error + diff_error * diff_factor;
            
            RCLCPP_DEBUG(this->get_logger(),"error = %.2f, linear.x=%.2f, w = %.2f ", err, cmd.linear.x, cmd.angular.z);
        }
        //  отправляем (публикуем) команду
        publisher_->publish(cmd);
    }
    // функция публикации ошибки
    void publish_error(double e){
        std_msgs::msg::Float64 err;
        err.data = e;
        err_pub_->publish(err);
    }
    // секция приватных членов
 private:
    // заданная координата линии, вдоль которой должен двигаться робот
    double line_y;
    double cx, cy, R;
    // заданная скорость движения
    double task_vel;
    // пропрциональный коэффициент регулятора обратной связи
    double prop_factor;
    // интегральный коэффициент регулятора
    double int_factor;
    // дифференциальный коэффициент регулятора
    double diff_factor;
    // интеграл ошибки
    double int_error;
    // старое значение ошибки
    double old_error;
    // минимально допустимое значение расстояния до препятствия
    double min_obstacle_range;
    // флаг наличия препятствия
    bool obstacle;
    // положение робота
    double x, y, theta;

    // публикатор команд управления
    // ros::Publisher cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    //  публикатор текущей ошибки управления
    // ros::Publisher err_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub_;
    // подписчики
    // ros::Subscriber laser_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    // ros::Subscriber pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    // ros::Timer timer1;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineControl>());
    rclcpp::shutdown();
    return 0;
}