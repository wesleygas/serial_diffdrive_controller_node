#ifndef DIFFDRIVE_CONTROLLER_HPP_
#define DIFFDRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <vector>
#include <cmath>

// For serial communication
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


class DiffdriveControllerNode : public rclcpp::Node
{
public:
    DiffdriveControllerNode();
    ~DiffdriveControllerNode();

private:
    // --- ROS 2 Callbacks ---
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_loop();

    // --- Serial Communication ---
    void setup_serial(const std::string &port, int baud_rate);
    void send_packet(uint8_t command_id, const std::vector<uint8_t>& payload);
    void read_and_process_serial();
    void handle_packet(uint8_t cmd_id, const std::vector<uint8_t>& payload);

    // --- MCU Initialization ---
    void initialize_mcu();

    // --- Data Processing & Publishing ---
    void process_odometry(const rclcpp::Time& stamp, int32_t left_ticks, int32_t right_ticks);
    void process_battery_voltage(float voltage);
    void publish_odometry(const rclcpp::Time& stamp, double linear_vel, double angular_vel);
    void publish_tf(const rclcpp::Time& stamp);
    void publish_joint_states(const rclcpp::Time& stamp, int32_t total_left_ticks, int32_t total_right_ticks, int32_t delta_left, int32_t delta_right, double dt);

    // --- Protocol Constants ---
    static constexpr uint8_t START_BYTE = 0x7E;
    static constexpr uint8_t CMD_SET_SPEEDS = 0x01;
    static constexpr uint8_t CMD_ODOMETRY_DATA = 0x03;
    static constexpr uint8_t CMD_GET_BATTERY = 0x04;
    static constexpr uint8_t CMD_BATTERY_DATA = 0x05;
    static constexpr uint8_t CMD_RESET_ENCODERS = 0x06;
    static constexpr uint8_t CMD_SET_ACCEL = 0x07;
    static constexpr uint8_t CMD_SET_TELEMETRY_RATE = 0x08;

    // --- Parameters callbacks ---
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // --- Parameters ---
    std::string odom_frame_id_;
    std::string base_frame_id_;
    double wheel_separation_;
    double wheel_radius_;
    double enc_cpr_;
    double motor_accel_;
    double battery_period;

    // --- Conversion Factors ---
    double ticks_per_meter_;
    double rads_per_tick_;
    double ticks_per_rad_;

    // --- ROS 2 Components ---
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- State Variables ---
    int serial_fd_ = -1; // File descriptor for the serial port
    geometry_msgs::msg::Twist latest_twist_;
    rclcpp::Time last_time_;
    rclcpp::Time last_battery_request_time_;
    
    // Use optional to handle the first reading gracefully
    std::optional<int32_t> last_left_ticks_;
    std::optional<int32_t> last_right_ticks_;

    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    double theta_ = 0.0;
};

#endif // DIFFDRIVE_CONTROLLER_HPP_