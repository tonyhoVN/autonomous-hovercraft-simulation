#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "matplotlibcpp.h"
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
/// error -> maybe
//sudo apt-get install python3-dev
//sudo apt-get install python-numpy
namespace plt = matplotlibcpp;

class Evaluate : public rclcpp::Node
{
  public:
    Evaluate()
    : Node("EvaluateStudent"){
//        rclcpp::get_logger("EvaluateTest").set_level(rclcpp::Logger::Level::Error);
//        (void)rcutils_logging_set_logger_level(rclcpp::get_logger("EvaluateTest").get_name(), RCUTILS_LOG_SEVERITY_ERROR);

        repeated = 1;
        this->declare_parameter("evalPath", "/home");
        start_time_ = this->now();
        // Subscribe to the /finished topic
        contact_subscription0_ = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/walls/contacts0", 10,
                std::bind(&Evaluate::contactCallback, this, std::placeholders::_1));
        contact_subscription1_ = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/walls/contacts1", 10,
                std::bind(&Evaluate::contactCallback, this, std::placeholders::_1));
        contact_subscription01_ = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/walls/contacts01", 10,
                std::bind(&Evaluate::contactCallback, this, std::placeholders::_1));
        contact_subscription02_ = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/walls/contacts02", 10,
                std::bind(&Evaluate::contactCallback, this, std::placeholders::_1));
        contact_subscription03_ = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/walls/contacts03", 10,
                std::bind(&Evaluate::contactCallback, this, std::placeholders::_1));
        contact_subscription04_ = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/walls/contacts04", 10,
                std::bind(&Evaluate::contactCallback, this, std::placeholders::_1));
        finished_subscription_ =
                create_subscription<std_msgs::msg::Bool>("/HW5/Finished", 10, std::bind(&Evaluate::finishedCallback, this, std::placeholders::_1));
        trace_subscription_ =
                create_subscription<geometry_msgs::msg::Point>("/HW5/Trace_data", 10, std::bind(&Evaluate::traceCallback, this, std::placeholders::_1));
        // 그래프 초기화
        plt::ion();  // 대화형 모드 설정
        plt::figure();  // 새로운 그래프 생성

//        this->randomMaze();
    }

  private:
    void contactCallback(const gazebo_msgs::msg::ContactsState::SharedPtr msg){

    }

    void finishedCallback(const std_msgs::msg::Bool::SharedPtr msg) {

    }

    void traceCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        x_values.push_back(msg->x);
        y_values.push_back(msg->y);

        double x_min = -10.0;
        double x_max = 10.0;
        double y_min = -10.0;
        double y_max = 15.0;
        plt::xlim(x_min, x_max);
        plt::ylim(y_min, y_max);

        plt::plot(x_values, y_values);  // 새로운 그래프 그리기
        plt::pause(0.1);  // 그래프 업데이트 시간 간격 설정
        plt::show();  // 그래프 표시
    }

    int repeated;
    double lastValueX;
    double lastValueY;
    std::vector<double> x_values;
    std::vector<double> y_values;
    rclcpp::Time start_time_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_subscription0_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_subscription1_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_subscription01_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_subscription02_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_subscription03_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_subscription04_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr trace_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
    auto nodeHW = std::make_shared<Evaluate>();
    rclcpp::spin(nodeHW);
  rclcpp::shutdown();
  return 0;
}

