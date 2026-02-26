#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <cmath>

class WheelOdomNode : public rclcpp::Node
{
public:
  WheelOdomNode() : Node("wheel_odom_node")
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WheelOdomNode::cmdCallback, this, std::placeholders::_1));

    wheel_radius_ = 0.05;
    wheel_separation_ = 0.45;
    ticks_per_rev_ = 683.0;

    openSerial();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&WheelOdomNode::update, this));
  }

private:
  int serial_fd_;
  long prev_left_ = 0;
  long prev_right_ = 0;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  double wheel_radius_;
  double wheel_separation_;
  double ticks_per_rev_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void openSerial()
  {
    // FINAL FIX 1: Permanent by-id path so it NEVER crashes the Lidar
    serial_fd_ = open("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN SERIAL PORT");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully!");

    termios tty{};
    tcgetattr(serial_fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tty.c_cc[VMIN]  = 0; 
    tty.c_cc[VTIME] = 0; 

    tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  void update()
  {
    static std::string serial_buffer;
    char buf[1];

    while (read(serial_fd_, buf, 1) > 0)
    {
      if (buf[0] == '\n')
      {
        processLine(serial_buffer);
        serial_buffer.clear();
      }
      else
      {
        serial_buffer += buf[0];
      }
    }
  }

  void processLine(const std::string & line)
  {
    if (line.find("ENC:") == std::string::npos)
      return;

    std::string data = line.substr(4);
    std::stringstream ss(data);

    std::string l, r;
    std::getline(ss, l, ',');
    std::getline(ss, r);

    try
    {
      long left = std::stol(l);
      long right = std::stol(r);

      // FINAL FIX 2: Inverting the backward encoder so the robot doesn't spin in place!
      right = -right;

      computeOdometry(left, right);
    }
    catch(...)
    {
      RCLCPP_WARN(this->get_logger(), "Parse failed: %s", line.c_str());
    }
  }

  void computeOdometry(long left_ticks, long right_ticks)
  {
    double d_left =
      (left_ticks - prev_left_) *
      (2 * M_PI * wheel_radius_ / ticks_per_rev_);

    double d_right =
      (right_ticks - prev_right_) *
      (2 * M_PI * wheel_radius_ / ticks_per_rev_);

    prev_left_ = left_ticks;
    prev_right_ = right_ticks;

    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / wheel_separation_;

    x_ += d_center * cos(theta_);
    y_ += d_center * sin(theta_);
    theta_ += d_theta;

    publishOdom(d_center, d_theta);
  }

  void publishOdom(double linear, double angular)
  {
    auto now = get_clock()->now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = cos(theta_ / 2.0);

    odom.twist.twist.linear.x = linear;
    odom.twist.twist.angular.z = angular;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation.z = sin(theta_ / 2.0);
    tf.transform.rotation.w = cos(theta_ / 2.0);

    tf_broadcaster_->sendTransform(tf);
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear = msg->linear.x;      
    double angular = msg->angular.z;   

    double v_left  = linear - (angular * wheel_separation_ / 2.0);
    double v_right = linear + (angular * wheel_separation_ / 2.0);

    double w_left  = v_left / wheel_radius_;
    double w_right = v_right / wheel_radius_;

    double max_wheel_rad_s = 10.47;   

    int left_pwm  = static_cast<int>((w_left  / max_wheel_rad_s) * 255.0);
    int right_pwm = static_cast<int>((w_right / max_wheel_rad_s) * 255.0);

    left_pwm  = std::max(-255, std::min(255, left_pwm));
    right_pwm = std::max(-255, std::min(255, right_pwm));

    std::stringstream ss;
    ss << "CMD:" << left_pwm << "," << right_pwm << "\n";

    std::string cmd = ss.str();
    write(serial_fd_, cmd.c_str(), cmd.size());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomNode>());
  rclcpp::shutdown();
  return 0;
}