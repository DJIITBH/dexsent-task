#include <rclcpp/rclcpp.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

class KDLTestNode : public rclcpp::Node
{
public:
  KDLTestNode() : Node("kdl_test_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting KDL test...");

    // 1. Create a simple chain: base -> link1 -> link2
    KDL::Chain chain;
    chain.addSegment(
      KDL::Segment("link1",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(1.0, 0.0, 0.0))
      )
    );

    chain.addSegment(
      KDL::Segment("link2",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(1.0, 0.0, 0.0))
      )
    );

    // 2. FK solver
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // 3. Joint values
    KDL::JntArray q(chain.getNrOfJoints());
    q(0) = 0.5;   // rad
    q(1) = 0.5;

    // 4. Compute FK
    KDL::Frame ee_frame;
    int status = fk_solver.JntToCart(q, ee_frame);

    if (status >= 0)
    {
      double x = ee_frame.p.x();
      double y = ee_frame.p.y();
      double z = ee_frame.p.z();

      RCLCPP_INFO(this->get_logger(),
        "FK success! EE position: x=%.3f y=%.3f z=%.3f",
        x, y, z);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "FK computation failed");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLTestNode>());
  rclcpp::shutdown();
  return 0;
}
