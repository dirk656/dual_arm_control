#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class DualArmController : public rclcpp::Node
{
public:
  DualArmController()
  : Node("dual_arm_controller")
  {
    // Initialize move groups using shared_from_this() but delay until after construction
    // We'll use a timer to initialize them after the node is fully constructed
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DualArmController::initialize_move_groups, this));
  }

  void initialize_move_groups()
  {
    // Stop the initialization timer
    init_timer_->cancel();

    // Initialize move groups using shared_from_this() now that the node is fully constructed
    left_arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "dummy2_left_arm");
    
    right_arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "dummy2_right_arm");
    
    left_hand_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "dummy2_left_hand");
    
    right_hand_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "dummy2_right_hand");

    // Set planning parameters
    left_arm_move_group_->setPlanningTime(10.0);
    right_arm_move_group_->setPlanningTime(10.0);
    left_arm_move_group_->setNumPlanningAttempts(10);
    right_arm_move_group_->setNumPlanningAttempts(10);

    RCLCPP_INFO(this->get_logger(), "Dual Arm Controller initialized");

    // Start the control loop
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&DualArmController::control_loop, this));
  }

private:
  void control_loop()
  {
    RCLCPP_INFO(this->get_logger(), "Starting dual arm control sequence");

    // Move to home position for both arms
    move_to_home();

    // Example: Move to a target pose
    move_to_target_pose();

    // Open both grippers
    open_grippers();

    // Close both grippers
    close_grippers();
  }

  void move_to_home()
  {
    RCLCPP_INFO(this->get_logger(), "Moving both arms to home position");

    // Set joint values for home position
    std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Move left arm to home
    left_arm_move_group_->setJointValueTarget(home_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    bool left_success = (left_arm_move_group_->plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (left_success) {
      left_arm_move_group_->execute(left_plan);
      RCLCPP_INFO(this->get_logger(), "Left arm moved to home position");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan for left arm home position");
    }

    // Move right arm to home
    right_arm_move_group_->setJointValueTarget(home_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    bool right_success = (right_arm_move_group_->plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (right_success) {
      right_arm_move_group_->execute(right_plan);
      RCLCPP_INFO(this->get_logger(), "Right arm moved to home position");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan for right arm home position");
    }
  }

  void move_to_target_pose()
  {
    RCLCPP_INFO(this->get_logger(), "Moving to target pose");

    // Create target pose for left arm
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.3;

    // Move left arm to target pose
    left_arm_move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    bool left_success = (left_arm_move_group_->plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (left_success) {
      left_arm_move_group_->execute(left_plan);
      RCLCPP_INFO(this->get_logger(), "Left arm moved to target pose");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan for left arm target pose");
    }

    // Create target pose for right arm (mirrored)
    target_pose.position.y = -0.2;

    // Move right arm to target pose
    right_arm_move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    bool right_success = (right_arm_move_group_->plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (right_success) {
      right_arm_move_group_->execute(right_plan);
      RCLCPP_INFO(this->get_logger(), "Right arm moved to target pose");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan for right arm target pose");
    }
  }

  void open_grippers()
  {
    RCLCPP_INFO(this->get_logger(), "Opening both grippers");

    // Open left gripper
    std::vector<double> open_position = {0.04, 0.04}; // Example open position
    left_hand_move_group_->setJointValueTarget(open_position);
    left_hand_move_group_->move();

    // Open right gripper
    right_hand_move_group_->setJointValueTarget(open_position);
    right_hand_move_group_->move();

    RCLCPP_INFO(this->get_logger(), "Both grippers opened");
  }

  void close_grippers()
  {
    RCLCPP_INFO(this->get_logger(), "Closing both grippers");

    // Close left gripper
    std::vector<double> close_position = {0.0, 0.0}; // Example close position
    left_hand_move_group_->setJointValueTarget(close_position);
    left_hand_move_group_->move();

    // Close right gripper
    right_hand_move_group_->setJointValueTarget(close_position);
    right_hand_move_group_->move();

    RCLCPP_INFO(this->get_logger(), "Both grippers closed");
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_hand_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_hand_move_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualArmController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
