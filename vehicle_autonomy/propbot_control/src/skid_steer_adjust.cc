#include <string>

#include "gazebo_msgs/GetJointProperties.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <ros/console.h>



geometry_msgs::Twist latest_rcv_cmd;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & msg)
{
   latest_rcv_cmd.linear.x = msg->linear.x;
   latest_rcv_cmd.angular.z = msg->angular.z;
//   latest_rcv_cmd = msg;
}

/* Publish wheel speed from gazebo
 *
 * This node calls the ---- service from gazebo and publishes wheel speeds extracted from it
 * 
 */
int main(int argc, char** argv) {
  // Initiation node called send mission
  ros::init(argc, argv, "skid_steer_adjustor");
  ros::NodeHandle nh;

  // Create publishers
  ros::Publisher cmd_vel_pub =
      nh.advertise<geometry_msgs::Twist>(
          "cmd_vel", 100);

  ros::Publisher right_wheel_pub =
      nh.advertise<std_msgs::Float32>(
          "right_wheel", 100);

  ros::Publisher left_wheel_pub =
      nh.advertise<std_msgs::Float32>(
          "left_wheel", 100);

  ros::Subscriber sub = nh.subscribe("/move_base/cmd_vel", 10, cmdVelCallback);

  ros::Rate loop_rate(5);
  geometry_msgs::Twist latest_sent_cmd;
  int num_consec_opposite = 0;
  int state = 0; //0 stop, 1 forward, 2 turning

  while (nh.ok()) {

   //latest_rcv_cmd.linear.x = -0.2;
   //latest_rcv_cmd.angular.z = -0.1;
   
    std_msgs::Float32 lw;
    std_msgs::Float32 rw;
    lw.data = 1;
    rw.data = -1;

    if(std::abs(latest_rcv_cmd.linear.x) > 0.1){
        if (state != 1){
          num_consec_opposite++;
          ROS_INFO("INCREMENTING FOR FORWARD COND");
        }
        
        if(num_consec_opposite > 5){
          state = 1;
          num_consec_opposite = 0;
          ROS_INFO("SETTING STATE TO 1");
        }

    } else if(std::abs(latest_rcv_cmd.angular.z) > 0.15){
        
        if (state != 2){
          num_consec_opposite++;
        }
        
        if(num_consec_opposite > 5){
          state = 2;
          num_consec_opposite = 0;
        }
        
    } else{
        num_consec_opposite = 0;
        state = 0;
    }


    if(state == 0){
        latest_sent_cmd.linear.x = 0.0;
        latest_sent_cmd.angular.z = 0.0;

    } else if (state == 1) {
        latest_sent_cmd.linear.x = latest_rcv_cmd.linear.x;
        latest_sent_cmd.angular.z = 0.0;

    }else if (state = 2){

        latest_sent_cmd.linear.x = 0.0;
        latest_sent_cmd.angular.z = latest_rcv_cmd.angular.z;


    }

    // Publish pose
    left_wheel_pub.publish(lw);
    right_wheel_pub.publish(rw);
    cmd_vel_pub.publish(latest_sent_cmd);

    // Sleep
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
