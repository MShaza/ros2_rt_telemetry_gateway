#include <iostream>
//#include <std_msgs/msg/String.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include "httplib.h"
#include <mutex>
#include <thread>
#include <string>

class ControlServerNode : public rclcpp::Node {
public: 
    ControlServerNode() : Node("control_server"){
        port_ = this->declare_parameter<int>("server_port", 8080);
        host_ = this->declare_parameter<std::string>("server_host", "0.0.0.0");
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vehicle/odometry", qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                speed_ = msg->twist.twist.linear.x;
            });
        sub_streering_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/steering_angle", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                steering_angle_ = msg->data;
            });
        sub_battery_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/battery_level", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                battery_level_ = msg->data;
            });
        pub_emergency_stop_ = this->create_publisher<std_msgs::msg::Bool>(
            "/vehicle/emergency_stop", qos);
        pub_control_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/vehicle/control_velocity", qos);
            server_thread_ = std::thread([this]() {
            server_.listen(host_, port_);
            });
            setup_routes();
    }
    ~ControlServerNode() override{
        server_.stop();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

private:
void setup_routes(){
    using namespace httplib;
    server_.Get("/status", [this](const Request& req, Response& res){
        float speed, steer, batt;
        bool estop;
        float last_target;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            speed = speed_;
            steer = steering_angle_;
            batt = battery_level_;
            estop = estop_active_;
            last_target = last_target_speed_;
        }
       std::string body = 
            "{ \"speed\": " + std::to_string(speed) +
            ", \"steering_angle\": " + std::to_string(steer) +
            ", \"battery_level\": " + std::to_string(batt) +
            ", \"emergency_stop\": " + (estop ? "true" : "false") +
            ", \"last_target_speed\": " + std::to_string(last_target) +
            " }";
        res.set_content(body, "application/json");
        res.status = 200;
   });
   server_.Post("/cmd", [this](const Request& req, Response& res){
        auto content_type = req.get_header_value("Content-Type");
        if (content_type != "application/json") {
            res.status = 400;
            res.set_content("{ \"error\": \"Invalid Content-Type\" }", "application/json");
            return;
        }
        try {
            auto json = nlohmann::json::parse(req.body);
            float target_speed = json.at("target_speed").get<float>();
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = target_speed;
            pub_control_vel_->publish(cmd_msg);
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_target_speed_ = target_speed;
            }
            res.status = 200;
            res.set_content("{ \"status\": \"Command accepted\" }", "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content("{ \"error\": \"Invalid JSON format\" }", "application/json");
        }
    });
        server_.Post("/emergency_stop", [this](const Request& req, Response& res){
        auto content_type = req.get_header_value("Content-Type");
        if (content_type != "application/json") {
            res.status = 400;
            res.set_content("{ \"error\": \"Invalid Content-Type\" }", "application/json");
            return;
        }
        try {
            auto json = nlohmann::json::parse(req.body);
            bool estop = json.at("emergency_stop").get<bool>();
            std_msgs::msg::Bool estop_msg;
            estop_msg.data = estop;
            pub_emergency_stop_->publish(estop_msg);
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                estop_active_ = estop;
            }
            res.status = 200;
            res.set_content("{ \"status\": \"Emergency stop command accepted\" }", "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content("{ \"error\": \"Invalid JSON format\" }", "application/json");
        }
   });
}

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_streering_;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_battery_;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_emergency_stop_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_vel_;

float speed_{0.0f};
float steering_angle_{0.0f};
float battery_level_{0.0f};
bool estop_active_{false};
float last_target_speed_{0.0f};
std::mutex data_mutex_;
int port_{8080};
std::string host_{"0.0.0.0"};
httplib::Server server_;
std::thread server_thread_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}