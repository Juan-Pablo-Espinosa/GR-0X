#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

class BasicCommandNode : public rclcpp::Node
{
public:
  BasicCommandNode() : Node("basic_command_node")
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);

    RCLCPP_INFO(this->get_logger(),
                "Basic command node ready. Commands: grow, shrink, splits");

    input_thread_ = std::thread(&BasicCommandNode::input_loop, this);
  }

  ~BasicCommandNode()
  {
    running_ = false;
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void input_loop()
  {
    std::string input;
    while (running_ && rclcpp::ok()) {
      std::cout << "> " << std::flush;
      if (!std::getline(std::cin, input)) break;

      if (input == "grow") {
        send_grow();
      } else if (input == "shrink") {
        send_shrink();
      } else if (input == "splits") {
        splits();
      } else if (input == "ankles"){
        ankles();
      }else if (input == "lock"){
        lock();
      }else{
        std::cout << "Unknown command: Available: grow, shrink, splits" << std::endl;
      }
    }
  }

  bool wait_for_controller()
  {
    int retries = 0;
    while (publisher_->get_subscription_count() == 0 && retries < 20) {
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "Waiting for joint_trajectory_controller to subscribe...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      retries++;
    }
    if (publisher_->get_subscription_count() == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "No subscribers on joint_trajectory topic. Is the controller active?");
      return false;
    }
    return true;
  }

  void send_target(const std::vector<std::string> & joints,
                   const std::vector<double> & positions,
                   const char * label)
  {
    if (!wait_for_controller()) return;

    if (joints.size() != positions.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "send_target(%s): joints/positions size mismatch (%zu vs %zu)",
                   label, joints.size(), positions.size());
      return;
    }

    trajectory_msgs::msg::JointTrajectory msg;
    //starts it
    msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    msg.joint_names = joints;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Sent '%s' command", label);
  }


  void send_grow()
  {
    send_target({"UTR", "UTL", "LTR", "LTL"},
                {0.4, 0.4, -1.0, -1.0},
                "grow");
  }

  void send_shrink()
  {
    send_target({"UTR", "UTL", "LTR", "LTL"},
                {0.0, 0.0, 0.0, 0.0},
                "shrink");
  }
  void lock()
  {
    send_target({"HPR", "HPL","UTR", "UTL", "LTR", "LTL","FL", "FR"},
      {0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0,0.0},"lock");
  }
  void splits()
  {
    send_target({"HPR", "HPL"},
                {0.6, 0.6},
                "splits");
  }
void ankles(){
    send_target({"FL", "FR"},{0.3,0.3},"ankles");
}
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  std::thread input_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BasicCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
