#include <string>

#include "gazebo_msgs/GetJointProperties.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>


/* Publish wheel speed from gazebo
 *
 * This node calls the ---- service from gazebo and publishes wheel speeds extracted from it
 * 
 */
int main(int argc, char** argv) {
  // Initiation node called send mission
  ros::init(argc, argv, "wheel_speed_publisher");
  ros::NodeHandle nh;  

  // Create publishers
  ros::Publisher left_wheel_pub = 
      nh.advertise<std_msgs::Int32>(
          "left_wheel", 100);

  ros::Publisher right_wheel_pub = 
      nh.advertise<std_msgs::Int32>(
          "right_wheel", 100);

  ros::Rate loop_rate(50);

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/GetJointProperties", true);

  if (!client){
    ROS_WARN("Failed to establish persistent connection to gazebo service");

  }

  while (nh.ok()) {
    
    if (!client){
        ROS_INFO("Attempting to reconnect to gazebo service");
        client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/GetJointProperties", true);
        continue;
    }

    std_msgs::Int32 lw;
    std_msgs::Int32 rw;  

    gazebo_msgs::GetJointProperties msg_lw;
    gazebo_msgs::GetJointProperties msg_rw;

    msg_lw.request.joint_name = "left_wheel";
    msg_rw.request.joint_name = "right_wheel";

    client.call(msg_lw);
    client.call(msg_rw);

    //do some translation//

    // Publish pose
    left_wheel_pub.publish(lw);
    right_wheel_pub.publish(rw);


    // Sleep
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
