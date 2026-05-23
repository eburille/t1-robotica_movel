#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Controlador: public rclcpp::Node
{
  public:
      Controlador(void);

  private:

      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPose_;


      void odomPubCB(const nav_msgs::msg::Odometry::SharedPtr odomPub);

      void goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalPose);
      
};

Controlador::Controlador(void): Node ("controlaor")
{
    using std::placeholders::_1;

    odomPub_=create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&Controlador::odomPubCB,this,_1));

    goalPose_=create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&Controlador::goalPoseCB,this,_1));
}

void Controlador::odomPubCB(const nav_msgs::msg::Odometry::SharedPtr odomPub)
{
    printf("Odometria recebida");
}

void Controlador::goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalPose)
{
    printf("Goal_pose recebido");
}

int main(int argc, char ** argv)
{
  printf("hello world T1 package\n");
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Controlador>());
  rclcpp::shutdown();
  return 0;
}
