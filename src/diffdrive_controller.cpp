#include "serial_diffdrive_controller_node/diffdrive_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <vector>
#include <cstring> // For memcpy

using namespace std::chrono_literals;

DiffdriveControllerNode::DiffdriveControllerNode() : Node("DiffdriveController")
{
    // --- Parameters ---
    this->declare_parameter<std::string>("device_port", "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_64:E8:33:83:BA:2C-if00");
    this->declare_parameter<int>("baud_rate", 230400);
    this->declare_parameter<double>("loop_rate", 60.0);
    this->declare_parameter<double>("wheel_separation", 0.210);
    this->declare_parameter<double>("wheel_radius", 0.034);
    this->declare_parameter<double>("enc_counts_per_rev", 1975.0);
    this->declare_parameter<double>("motor_acceleration", 9000.0);
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<std::string>("twist_topic", "/cmd_vel_out");
    this->declare_parameter<double>("battery_reading_period", 60.0);

    // Get parameters
    auto port = this->get_parameter("device_port").as_string();
    auto baud = this->get_parameter("baud_rate").as_int();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    enc_cpr_ = this->get_parameter("enc_counts_per_rev").as_double();
    motor_accel_ = this->get_parameter("motor_acceleration").as_double();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    auto twist_topic = this->get_parameter("twist_topic").as_string();
    auto loop_rate = this->get_parameter("loop_rate").as_double();
    battery_period = this->get_parameter("battery_reading_period").as_double();

    // --- Conversion Factors ---
    ticks_per_meter_ = enc_cpr_ / (2 * M_PI * wheel_radius_);
    rads_per_tick_ = (2 * M_PI) / enc_cpr_;
    ticks_per_rad_ = enc_cpr_ / (2 * M_PI);

    // --- Publishers and Subscribers ---
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic, 10, std::bind(&DiffdriveControllerNode::cmd_vel_callback, this, std::placeholders::_1));
    
    // -- Parameter Callbacks
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DiffdriveControllerNode::parameters_callback, this, std::placeholders::_1));

    // --- Serial Communication ---
    setup_serial(port, baud);
    if (serial_fd_ != -1) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to %s", port.c_str());
        // Give MCU time to boot up
        rclcpp::sleep_for(2s);
        initialize_mcu();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to %s. Shutting down.", port.c_str());
        rclcpp::shutdown();
        return;
    }

    // --- State variables ---
    last_time_ = this->get_clock()->now();
    last_battery_request_time_ = this->get_clock()->now();

    // --- Main Control Loop ---
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_rate),
        std::bind(&DiffdriveControllerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Custom controller node has been started.");
}

DiffdriveControllerNode::~DiffdriveControllerNode()
{
    RCLCPP_INFO(this->get_logger(), "Stopping robot...");
    // Create payload for zero speed
    std::vector<uint8_t> stop_payload(4);
    int16_t zero_speed = 0;
    memcpy(stop_payload.data(), &zero_speed, sizeof(zero_speed));
    memcpy(stop_payload.data() + 2, &zero_speed, sizeof(zero_speed));
    send_packet(CMD_SET_SPEEDS, stop_payload);

    if (serial_fd_ != -1) {
        close(serial_fd_);
    }
}

void DiffdriveControllerNode::setup_serial(const std::string &port, int baud_rate) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
        return;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // Set baud rate
    speed_t speed;
    switch(baud_rate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud_rate);
            close(serial_fd_);
            serial_fd_ = -1;
            return;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // Set options for raw data
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    // Set timeout (VMIN=0, VTIME=5 -> 0.5s timeout)
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5; // 0.5 seconds

    tcsetattr(serial_fd_, TCSANOW, &options);
    fcntl(serial_fd_, F_SETFL, 0); // Set to blocking read
}

void DiffdriveControllerNode::initialize_mcu()
{
    RCLCPP_INFO(this->get_logger(), "Initializing MCU...");
    send_packet(CMD_RESET_ENCODERS, {});
    rclcpp::sleep_for(100ms);

    std::vector<uint8_t> accel_payload(4);
    uint32_t accel_ticks = static_cast<uint32_t>(motor_accel_);
    memcpy(accel_payload.data(), &accel_ticks, sizeof(accel_ticks));
    send_packet(CMD_SET_ACCEL, accel_payload);
    rclcpp::sleep_for(100ms);

    auto loop_rate = this->get_parameter("loop_rate").as_double();
    RCLCPP_INFO(this->get_logger(), "Setting telemetry rate to %.0f Hz.", loop_rate);
    std::vector<uint8_t> telemetry_payload(2);
    uint16_t rate_hz = static_cast<uint16_t>(loop_rate);
    memcpy(telemetry_payload.data(), &rate_hz, sizeof(rate_hz));
    send_packet(CMD_SET_TELEMETRY_RATE, telemetry_payload);

    RCLCPP_INFO(this->get_logger(), "MCU Initialized.");
}

void DiffdriveControllerNode::send_packet(uint8_t command_id, const std::vector<uint8_t>& payload)
{
    if (serial_fd_ == -1) return;

    uint8_t payload_len = 2 + payload.size();
    std::vector<uint8_t> packet;
    packet.push_back(START_BYTE);
    packet.push_back(payload_len);
    packet.push_back(command_id);
    packet.insert(packet.end(), payload.begin(), payload.end());

    uint8_t checksum = 0;
    for (size_t i = 2; i < packet.size(); ++i) {
        checksum += packet[i];
    }
    packet.push_back(checksum);

    if (write(serial_fd_, packet.data(), packet.size()) < 0) {
        RCLCPP_WARN(this->get_logger(), "Error writing to serial port: %s", strerror(errno));
    }
}

void DiffdriveControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_twist_ = *msg;
}

void DiffdriveControllerNode::control_loop()
{
    // 1. --- SEND MOTOR COMMANDS ---
    double linear_x = latest_twist_.linear.x;
    double angular_z = latest_twist_.angular.z;

    double v_right_rads = (linear_x + (angular_z * wheel_separation_ / 2.0)) / wheel_radius_;
    double v_left_rads = (linear_x - (angular_z * wheel_separation_ / 2.0)) / wheel_radius_;

    int16_t v_right_ticks = static_cast<int16_t>(v_right_rads * ticks_per_rad_);
    int16_t v_left_ticks = static_cast<int16_t>(v_left_rads * ticks_per_rad_);

    std::vector<uint8_t> speed_payload(4); // 2 * int16_t
    memcpy(speed_payload.data(), &v_left_ticks, sizeof(v_left_ticks));
    memcpy(speed_payload.data() + 2, &v_right_ticks, sizeof(v_right_ticks));
    send_packet(CMD_SET_SPEEDS, speed_payload);

    // 2. --- REQUEST OTHER TELEMETRY ---
    auto now = this->get_clock()->now();
    if ((now - last_battery_request_time_).seconds() > battery_period) {
        send_packet(CMD_GET_BATTERY, {});
        last_battery_request_time_ = now;
    }

    // 3. --- READ AND PROCESS INCOMING DATA ---
    read_and_process_serial();
}

void DiffdriveControllerNode::read_and_process_serial()
{
    uint8_t buffer[256];
    int bytes_read = read(serial_fd_, buffer, sizeof(buffer));

    if (bytes_read > 0) {
        // This is a simplified parser. For robustness, you might need a more
        // stateful parser that can handle fragmented packets.
        for (int i = 0; i < bytes_read; ++i) {
            if (buffer[i] == START_BYTE) {
                if (i + 1 < bytes_read) {
                    uint8_t packet_len = buffer[i+1];
                    if (i + 1 + packet_len <= bytes_read) {
                        // We have a full packet in the buffer
                        std::vector<uint8_t> packet_data;
                        packet_data.assign(buffer + i + 2, buffer + i + 2 + packet_len);

                        uint8_t received_checksum = packet_data.back();
                        packet_data.pop_back(); // Remove checksum for calculation

                        uint8_t calculated_checksum = 0;
                        for(uint8_t byte : packet_data) {
                            calculated_checksum += byte;
                        }

                        if (received_checksum == calculated_checksum) {
                            uint8_t cmd_id = packet_data[0];
                            std::vector<uint8_t> payload(packet_data.begin() + 1, packet_data.end());
                            handle_packet(cmd_id, payload);
                        }
                        // Move index past this processed packet
                        i += 1 + packet_len;
                    }
                }
            }
        }
    } else if (bytes_read < 0 && errno != EAGAIN) {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
    }
}

void DiffdriveControllerNode::handle_packet(uint8_t cmd_id, const std::vector<uint8_t>& payload)
{
    if (cmd_id == CMD_ODOMETRY_DATA && payload.size() == 8) { // 2 * int32_t
        int32_t left_ticks, right_ticks;
        memcpy(&left_ticks, payload.data(), sizeof(left_ticks));
        memcpy(&right_ticks, payload.data() + 4, sizeof(right_ticks));
        process_odometry(this->get_clock()->now(), left_ticks, right_ticks);
    } else if (cmd_id == CMD_BATTERY_DATA && payload.size() == 4) { // 1 * float
        float voltage;
        memcpy(&voltage, payload.data(), sizeof(voltage));
        process_battery_voltage(voltage);
    } else {
        RCLCPP_WARN(this->get_logger(), "Malformed or unknown packet received. CMD_ID: %d, Size: %zu", cmd_id, payload.size());
    }
}

void DiffdriveControllerNode::process_odometry(const rclcpp::Time& current_time, int32_t left_ticks, int32_t right_ticks)
{
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) return;

    if (!last_left_ticks_.has_value() || !last_right_ticks_.has_value()) {
        last_left_ticks_ = left_ticks;
        last_right_ticks_ = right_ticks;
        last_time_ = current_time;
        return;
    }

    int32_t delta_left_ticks = left_ticks - last_left_ticks_.value();
    int32_t delta_right_ticks = right_ticks - last_right_ticks_.value();

    double dist_left = delta_left_ticks / ticks_per_meter_;
    double dist_right = delta_right_ticks / ticks_per_meter_;

    double delta_dist = (dist_right + dist_left) / 2.0;
    double delta_theta = (dist_right - dist_left) / wheel_separation_;

    x_pos_ += delta_dist * cos(theta_ + delta_theta / 2.0);
    y_pos_ += delta_dist * sin(theta_ + delta_theta / 2.0);
    theta_ += delta_theta;
    theta_ = atan2(sin(theta_), cos(theta_)); // Normalize angle

    double linear_velocity = delta_dist / dt;
    double angular_velocity = delta_theta / dt;

    publish_odometry(current_time, linear_velocity, angular_velocity);
    publish_tf(current_time);
    publish_joint_states(current_time, left_ticks, right_ticks, delta_left_ticks, delta_right_ticks, dt);

    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;
    last_time_ = current_time;
}

// The publishing functions are very similar to Python, just with C++ syntax
void DiffdriveControllerNode::publish_odometry(const rclcpp::Time& stamp, double linear_vel, double angular_vel) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;
    
    // Example covariance
    odom_msg.pose.covariance[0] = 0.1; odom_msg.pose.covariance[7] = 0.1; odom_msg.pose.covariance[35] = 0.2;
    odom_msg.twist.covariance = odom_msg.pose.covariance;

    odom_pub_->publish(odom_msg);
}

void DiffdriveControllerNode::publish_tf(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id = base_frame_id_;
    t.transform.translation.x = x_pos_;
    t.transform.translation.y = y_pos_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
}

void DiffdriveControllerNode::publish_joint_states(const rclcpp::Time& stamp, int32_t total_left_ticks, int32_t total_right_ticks, int32_t delta_left, int32_t delta_right, double dt) {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = stamp;
    joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};
    joint_state_msg.position = {
        static_cast<double>(total_left_ticks * rads_per_tick_),
        static_cast<double>(total_right_ticks * rads_per_tick_)
    };
    if (dt > 0) {
        joint_state_msg.velocity = {
            (delta_left * rads_per_tick_) / dt,
            (delta_right * rads_per_tick_) / dt
        };
    }
    joint_pub_->publish(joint_state_msg);
}

void DiffdriveControllerNode::process_battery_voltage(float voltage) {
    auto battery_msg = std::make_unique<sensor_msgs::msg::BatteryState>();
    battery_msg->header.stamp = this->get_clock()->now();
    battery_msg->voltage = voltage;
    battery_msg->present = true;
    battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_msg->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery_pub_->publish(std::move(battery_msg));
}

rcl_interfaces::msg::SetParametersResult DiffdriveControllerNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == "battery_reading_period") {
            battery_period = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated battery_reading_period to: %.2f s", battery_period);
        }
    }
    return result;
}


// --- Main Function ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffdriveControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}