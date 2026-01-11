#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>
#include <streambuf>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <iostream>
#include <Eigen/Core>

using namespace std::chrono_literals;
using namespace std;

class CartesianController : public rclcpp::Node {
public:
    CartesianController() : Node("cartesian_controller")
    {
        joint_traj_pub_r = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_cont_r/joint_trajectory", 10);
        joint_traj_pub_l = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_cont_l/joint_trajectory", 10);
        pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/desired_pose", 10, std::bind(&CartesianController::poseCallback, this, std::placeholders::_1));
        urdf_parser();
        RCLCPP_INFO(this->get_logger(), "Dual-arm Initialized");

    }

    void poseCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 14)
        {
            RCLCPP_ERROR(this->get_logger(), "Expected 12D pose vector");
            return;
        }
        auto &d = msg->data;
        std::vector<double> pose_vec;

        for(int i=0; i<14; i++)
        {
            pose_vec.push_back(d[i]);
        }
        vector<double> j_angles = getJointAngles(pose_vec);
        RCLCPP_INFO(this->get_logger(), "Printing Joint Angles...");
        for(int i=0; i<j_angles.size(); i++)
        {
            cout<<j_angles[i]<<endl;
        }
        move_arms(j_angles);

    }

    void urdf_parser() {
        RCLCPP_INFO(this->get_logger(), "Received URDF, parsing...");
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("ur_description_pkg");
        std::string urdf_path = package_share_dir + "/urdf/dual_arm.urdf.xacro";
        std::string urdf;

        std::string command = "xacro " + urdf_path;
        std::array<char, 128> buffer;
        std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
        
        if (!pipe) {
            throw std::runtime_error("Failed to run xacro command");
        }

        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            urdf += buffer.data();
        }


        if (!kdl_parser::treeFromString(urdf, tree_r)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree");
            return;
        }


        if (!tree_r.getChain("triangular_mount", "tool0_r", chain_r)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from base_link to right_end_link");
            return;
        }

    
        if (!kdl_parser::treeFromString(urdf, tree_l)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree");
            return;
        }


        if (!tree_l.getChain("triangular_mount", "tool0_l", chain_l)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from base_link to left_end_link");
            return;
        }

    
        Eigen::Matrix<double, 6, 1> weights;
        weights << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;  
        solver_r = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_r, weights,1E-5, 10000);
        solver_l = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_l, weights,1E-5, 10000);

        std::cout << "Chain extracted successfully!" << std::endl;

  }

  vector<double> getJointAngles(vector<double> pose_vec) {

    KDL::Vector p_left = KDL::Vector(pose_vec[0], pose_vec[1], pose_vec[2]);
    KDL::Rotation q_left = KDL::Rotation::Quaternion(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);

    KDL::Vector p_right = KDL::Vector(pose_vec[7], pose_vec[8], pose_vec[9]);
    KDL::Rotation q_right = KDL::Rotation::Quaternion(pose_vec[10], pose_vec[11], pose_vec[12], pose_vec[13]);
    
    // KDL::Frame p_in();
    KDL::Frame pose_left(q_left, p_left);
    KDL::Frame pose_right(q_right, p_right);
    KDL::JntArray q_out_right(chain_r.getNrOfJoints()); //output of right
    KDL::JntArray q_out_left(chain_l.getNrOfJoints());  //output of left


      if (!solver_r) 
      {
        RCLCPP_ERROR(this->get_logger(), "IK solver not initialized!");
        std::vector<double> vec(12, 0.0);
        return vec;
      }

      // Initial guess
      KDL::JntArray q_init_right(chain_r.getNrOfJoints());
      q_init_right(0) = 0.0;
      q_init_right(1) = 0.0;
      q_init_right(2) = 0.0;
      q_init_right(3) = 0.0;
      q_init_right(4) = 0.0;
      q_init_right(5) = 0.0;

    //   right solving
      int result_right = solver_r->CartToJnt(q_init_right, pose_right, q_out_right);
      if (result_right < 0) {
        RCLCPP_ERROR(this->get_logger(), "IK Right failed !");
        std::vector<double> vec(12, 0.0);
        return vec;
      }

      // Initial guess
      KDL::JntArray q_init_left(chain_l.getNrOfJoints());
      q_init_left(0) = 0.0;
      q_init_left(1) = 0.0;
      q_init_left(2) = 0.0;
      q_init_left(3) = 0.0;
      q_init_left(4) = 0.0;
      q_init_left(5) = 0.0;


    //   left solving
      int result_left = solver_l->CartToJnt(q_init_left, pose_left, q_out_left);
      if (result_left < 0) {
          RCLCPP_ERROR(this->get_logger(), 
                      "IK Left failed !");
      }
      vector<double> j_angles;
      for (int i=0; i<6; i++)
      {
        j_angles.push_back(q_out_left(i));
      }
      for (int i=0; i<6; i++)
      {
        j_angles.push_back(q_out_right(i));
      }
      RCLCPP_INFO(this->get_logger(), "Joint Angle Solution Found for Both Arms....");
      return j_angles;
    }

    void move_arms(vector<double> j_angles)
    {
        RCLCPP_INFO(this->get_logger(), "Arms Are Moving....");
         //#################### Moving Right ###################################
        auto msg_rb = trajectory_msgs::msg::JointTrajectory();
        msg_rb.joint_names = {"joint_1_r", "joint_2_r", "joint_3_r", "joint_4_r", "joint_5_r", "joint_6_r"};

        auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
        point_rb.positions = {j_angles[6], j_angles[7], j_angles[8], j_angles[9], j_angles[10], j_angles[11]};  // Target joint positions
        
        builtin_interfaces::msg::Duration duration;
        duration.sec = 4;         // Full seconds part
        duration.nanosec = 0;  
        point_rb.time_from_start = duration;
        msg_rb.points.push_back(point_rb);
        
        joint_traj_pub_r->publish(msg_rb);

        //#################### Moving Left ###################################
        auto msg_lf = trajectory_msgs::msg::JointTrajectory();
        msg_lf.joint_names = {"joint_1_l", "joint_2_l", "joint_3_l", "joint_4_l", "joint_5_l", "joint_6_l"};

        auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
        point_lf.positions = {j_angles[0], j_angles[1], j_angles[2], j_angles[3], j_angles[4], j_angles[5]}; // Target joint positions
        
        point_lf.time_from_start = duration;
        msg_lf.points.push_back(point_lf);

        joint_traj_pub_l->publish(msg_lf);

        double time_to_wait = (4) * 1000;  // Convert to milliseconds
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
        RCLCPP_INFO(this->get_logger(), "Arms Have Moved!");
    }



private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_r;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_l;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_sub_;
    KDL::Tree tree_l;
    KDL::Tree tree_r;
    KDL::Chain chain_l;
    KDL::Chain chain_r;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_l;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_r;


};

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CartesianController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
