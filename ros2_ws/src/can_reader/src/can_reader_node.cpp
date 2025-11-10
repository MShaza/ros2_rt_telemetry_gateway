#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

class CanReaderNode : public rclcpp::Node
{
public:
    CanReaderNode() : Node("can_reader_node")
    {
        can_interface = this->declare_parameter<std::string>("can_interface", "vcan0");
        can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 100);
        // Initialize CAN socket
        create_socket();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CanReaderNode::pool_socket, this));
    }

private:
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
    std::string can_interface;
    int socket_fd_{-1};
    rclcpp::TimerBase::SharedPtr timer_;
    void create_socket()
    {
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
            return;
        }
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting interface index for %s", can_interface.c_str());
            close(socket_fd_);
            return;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in socket bind for %s", can_interface.c_str());
            close(socket_fd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully bound to CAN interface %s", can_interface.c_str());
    }
    int pool_socket()
    {
        struct can_frame frame;
        int nBytes = read(socket_fd_, &frame, sizeof(frame));
        if (nBytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame");
            return -1;
        }
        if (nBytes == sizeof(frame))
        {
            can_msgs::msg::Frame can_msg;
            can_msg.id = frame.can_id;
            can_msg.dlc = frame.can_dlc;
            can_msg.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
            can_msg.is_rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
            //.data.resize(frame.can_dlc);
            can_msg.is_error = frame.can_id & CAN_ERR_FLAG;
            std::copy(frame.data, frame.data + frame.can_dlc, can_msg.data.begin());
            can_publisher_->publish(can_msg);
            return 0;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}