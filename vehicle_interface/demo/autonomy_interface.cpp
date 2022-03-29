#include "autonomy_interface.h"

ros::NodeHandle node;
std_msgs::UInt8 ws_left;
std_msgs::UInt8 ws_right;

//Defining Publisher and callback
ros::Publisher talker_left("pwm_left", &ws_left);
ros::Publisher talker_right("pwm_right", &ws_right);
    
void callback_left ( const std_msgs::UInt8& msg_left){
  ws_left.data = msg_left.data;
  talker_left.publish( &ws_left );
}

void callback_right ( const std_msgs::UInt8& msg_right){
  ws_right.data = msg_right.data;
  talker_right.publish( &ws_right );
}

//Defining Subscriber
ros::Subscriber<std_msgs::UInt8> listener_left("left_wheel", callback_left);
ros::Subscriber<std_msgs::UInt8> listener_right("right_wheel", callback_right);

void setup_rosserial(){

      //Initializing node
      node.initNode();
      
      //Configure Subscriber and Publisher
      node.advertise(talker_left);
      node.subscribe(listener_left);
      node.advertise(talker_right);
      node.subscribe(listener_right);
}
    
