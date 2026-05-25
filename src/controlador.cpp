#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>

class Controlador: public rclcpp::Node
{
  public:
      Controlador(void);
      void angVelPublisher(void);

  private:
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double xRef = 0.0;
        double yRef = 0.0;
        double thetaRef = 0.0;
        double vel_x = 0.0;
        double vel_y = 0.0;
        double vel_lin = 0.0;
        double alpha = 0.0;
        rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr angVelPub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPose_;

        rclcpp::TimerBase::SharedPtr timer_;

        void odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub);

        void goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalPose);

        void torqueCTRL();
      
};

Controlador::Controlador(void): Node ("controlaor")
{   
    printf("inicio construtor");
    using std::placeholders::_1;

    odomSub_=create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&Controlador::odomSubCB,this,_1));

    goalPose_=create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&Controlador::goalPoseCB,this,_1));


    angVelPub_=create_publisher<control_msgs::msg::MultiDOFCommand>("joint_velocity_controller/reference",100);
    printf("Antes int");
    int rate = 100;
    timer_=rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0/rate),std::bind(&Controlador::angVelPublisher,this));

    printf("fim construtor");
}

void Controlador::angVelPublisher(void)
{   
    // RCLCPP_INFO(this->get_logger(), "\n ang vel publicada\n");

    static double xAlvo = 0.0;    
    static double yAlvo = 0.0;
    static double thetaAlvo = 0.0; 
    
    static double xAntigo = 0.0;    
    static double yAntigo = 0.0;
    static double thetaAntigo = 0.0;    

    xAlvo = xAntigo * alpha + xRef * (1-alpha);    
    yAlvo = yAntigo * alpha + yRef * (1-alpha);
    thetaAlvo = thetaAntigo * alpha + thetaRef * (1-alpha);    

    double y_dot[2] = {xAlvo - x, yAlvo - y};

    xAntigo = xAlvo;
    yAntigo = yAlvo;
    thetaAntigo = thetaAlvo;


    double Einv[2][2] = {
        {cos(theta) - sin(theta),   sin(theta) + cos(theta)},
        {         -sin(theta)   ,         cos(theta)       }
    };

    // v = posicao referencia - posicao real
    // u = {v_lin, v_ang}
    double u[2] = {
        y_dot[0]*Einv[0][0] + y_dot[1]*Einv[0][1],
        y_dot[0]*Einv[1][0] + y_dot[1]*Einv[1][1] 
    };

    // vel_ang_r
    double w1 = (u[0] + u[1] * 0.322 / 2) / 0.075;
    double w2 = (u[0] - u[1] * 0.322 / 2) / 0.075;


    control_msgs::msg::MultiDOFCommand msg;
    msg.dof_names = {"right_wheel_joint", "left_wheel_joint"};
    msg.values = {w1, w2};

    // RCLCPP_INFO(this->get_logger(), "--- Vel Atual do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "V:     %.4f metros/s", w1);
    // RCLCPP_INFO(this->get_logger(), "W:     %.4f metros/s", w2);

    angVelPub_->publish(msg);
}


void Controlador::odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub)
{
    x = odomSub->pose.pose.position.x;
    y = odomSub->pose.pose.position.y;

    double z_orientation = odomSub->pose.pose.orientation.z;
    double w_orientation = odomSub->pose.pose.orientation.w;
    theta = 2*atan2(z_orientation,w_orientation);

    vel_x = odomSub->twist.twist.linear.x; 
    vel_y = odomSub->twist.twist.linear.x; 

    vel_lin = std::sqrt(vel_x*vel_x + vel_y*vel_y);

    // RCLCPP_INFO(this->get_logger(), "\nOdometria recebida\n");
    // RCLCPP_INFO(this->get_logger(), "--- Posição Atual do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "X:     %.4f metros", x);
    // RCLCPP_INFO(this->get_logger(), "Y:     %.4f metros", y);
    // RCLCPP_INFO(this->get_logger(), "Theta: %.4f radianos (%.2f°)", theta, theta * (180.0 / M_PI));

    // RCLCPP_INFO(this->get_logger(), "--- Posição Ref do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "X:     %.4f metros", xRef);
    // RCLCPP_INFO(this->get_logger(), "Y:     %.4f metros", yRef);
    // RCLCPP_INFO(this->get_logger(), "Theta: %.4f radianos (%.2f°)", thetaRef, thetaRef * (180.0 / M_PI));

}

void Controlador::goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalPose)
{   
    xRef = goalPose->pose.position.x;
    yRef = goalPose->pose.position.y;

    double z_orientation = goalPose->pose.orientation.z;
    double w_orientation = goalPose->pose.orientation.w;
    
    double dRef = std::sqrt(xRef*xRef + yRef*yRef);
    double d = std::sqrt(x*x + y*y);

    thetaRef = 2*atan2(z_orientation,w_orientation);

    double ts = (0.1 + (dRef - d) * 17) / 4;
    alpha = std::exp(-0.01 / ts);
}

void Controlador::torqueCTRL()
{

}

int main(int argc, char ** argv)
{
  printf("hello world T1 package hehehe\n");
  rclcpp::init(argc,argv);
  printf("aqui");
  rclcpp::spin(std::make_shared<Controlador>());
  rclcpp::shutdown();
  return 0;
}
