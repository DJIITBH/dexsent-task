#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <custom_service/srv/fibonacci.hpp>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group_l = moveit::planning_interface::MoveGroupInterface(node, "fanuc_l"); //node, 'name of move group'
    auto arm_move_group_r = moveit::planning_interface::MoveGroupInterface(node, "fanuc_r");
    //access and send cmd and view status of particular move_group

    std::vector<double> arm_joint_goal {1.57,-0.8,0.0,0.0,0.0,0.0};

    bool arm_within_bounds_l = arm_move_group_l.setJointValueTarget(arm_joint_goal); //returns a boolean value
    bool arm_within_bounds_r = arm_move_group_r.setJointValueTarget(arm_joint_goal); //returns a boolean value

    if(!arm_within_bounds_l)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits of left arm");
        return ;
    }
    if(!arm_within_bounds_r)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits of right arm");
        return ;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_l;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_r;

   bool arm_plan_success_l = (arm_move_group_l.plan(arm_plan_l) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned
   bool arm_plan_success_r = (arm_move_group_r.plan(arm_plan_r) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

   if(arm_plan_success_l && arm_plan_success_r)
   {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, moving both the arms....!");
        arm_move_group_l.move();
        arm_move_group_r.move();
   }
   else
   {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
        return;
   }

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
    move_robot(node);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}