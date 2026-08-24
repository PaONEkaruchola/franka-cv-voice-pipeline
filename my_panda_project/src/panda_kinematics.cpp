#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>

class PandaKinematicsNode : public rclcpp::Node
{
public:
    PandaKinematicsNode(const rclcpp::NodeOptions& options)
    : Node("panda_kinematics", options)
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        // 1. Listen for the 3D Coordinates
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            std::bind(&PandaKinematicsNode::pose_callback, this, std::placeholders::_1),
            sub_options);

        // 2. Listen for the Block Color from Python
        color_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/target_color", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                current_color_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Active Target Color: %s", current_color_.c_str());
            }, sub_options);

        // 3. Publisher for the Hand Controller
        gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/panda_hand_controller/joint_trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "Pick & Place Node Ready. Awaiting voice target...");
    }

    void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) {
        move_group_ = move_group;
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
    std::string current_color_ = "Red"; // Default initialization

    // Helper function to actuate the fingers instantly
    void actuate_gripper(double width) {
        trajectory_msgs::msg::JointTrajectory traj;
        
        // Sync with Gazebo time for instant execution
        traj.header.stamp = this->get_clock()->now(); 
        
        traj.joint_names.push_back("panda_finger_joint1");
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(width);
        point.time_from_start.sec = 1; 
        
        traj.points.push_back(point);
        gripper_pub_->publish(traj);
        RCLCPP_INFO(this->get_logger(), "Actuating gripper to: %f", width);
    }

    // Helper function to execute poses
    bool execute_pose(geometry_msgs::msg::PoseStamped target_pose, const std::string& step_name) {
        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Executing: %s", step_name.c_str());
            move_group_->move();
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan: %s", step_name.c_str());
            return false;
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!move_group_) return;
        RCLCPP_INFO(this->get_logger(), "Target received! Starting Pick and Place Sequence...");

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world"; // Restore the critical TF math!
        target_pose.header.stamp = this->get_clock()->now();

        // Perfect Orthogonal Downward Grasp (Fingers parallel to Y-axis)
        target_pose.pose.orientation.x = 0.9239; 
        target_pose.pose.orientation.y = -0.3827;
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 0.0;
        
        target_pose.pose.position.x = msg->pose.position.x;
        target_pose.pose.position.y = msg->pose.position.y;

        // Step 1: Hover above the block
        target_pose.pose.position.z = msg->pose.position.z + 0.15; 
        if (!execute_pose(target_pose, "Hover")) return;

        // Step 2: Descend to the block (0.08 to prevent table bounce)
        target_pose.pose.position.z = msg->pose.position.z + 0.08; 
        if (!execute_pose(target_pose, "Descend to Block")) return;

        // Close Gripper to firmly grasp the block
        actuate_gripper(0.024); 
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Step 3: Lift the block up
        target_pose.pose.position.z = msg->pose.position.z + 0.20; 
        if (!execute_pose(target_pose, "Lift Block")) return;

        // Step 4: Move to the Drop-off Table (Dynamically sorted by color)
        target_pose.pose.position.y = 0.50; 
        
        if (current_color_ == "Red") {
            target_pose.pose.position.x = 0.10; // Center
        } else if (current_color_ == "Blue") {
            target_pose.pose.position.x = 0.20; // Offset to the right
        } else if (current_color_ == "Green") {
            target_pose.pose.position.x = 0.30; // Offset to the left
        } else {
            target_pose.pose.position.x= 0.50; // Fallback
        }
        
        if (!execute_pose(target_pose, "Move to Drop-off Table")) return;

        // Step 5: Lower onto the Drop-off Table (0.58 to prevent table bounce)
        target_pose.pose.position.z = 0.58; 
        if (!execute_pose(target_pose, "Lower to Table")) return;

        // Open Gripper to release the block
        actuate_gripper(0.04); 
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Step 6: Return to a safe clearance hover
        target_pose.pose.position.z = 0.65; 
        execute_pose(target_pose, "Return to Safe Clearance");
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<PandaKinematicsNode>(options);
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");
    node->set_move_group(move_group);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}